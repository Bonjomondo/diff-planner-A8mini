#!/usr/bin/env python3
"""任务初始化的机载编排：按依赖顺序把飞行前置链 bring-up 到位。

设计原则：
1. 地面初始化重启飞行链路；任务网关与 ROS master 保持运行。
2. 按依赖顺序执行，每一步等"就绪信号"而不是固定 sleep。
3. 异步——初始化立即返回，进度写入 phase/detail,由遥测上报给地面站。
4. 失败即停——某一步超时就中止，不启动依赖它的后续步骤。
"""
import os
import math
import json
import signal
import subprocess
import threading
import time

import rospy
from mavros_msgs.msg import State, ExtendedState
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64
from multipoint.msg import MissionStatus

# 录制话题：与现场手工命令保持一致
BAG_TOPICS = [
    '/tag_detections', '/landing_camera/landing_pose', '/landing_camera/status',
    '/landing_camera/set_landing', '/debugPx4ctrl', '/mavros/imu/data',
    '/mavros/setpoint_raw/attitude', '/mavros/setpoint_raw/target_attitude',
    '/mission/remote/status', '/mavros/state', '/mavros/extended_state', '/mavros/rc/in',
    '/mavros/battery', '/ekf/ekf_odom', '/laserMapping/odometry', '/px4ctrl/state',
    '/px4ctrl/mission_ready', '/px4ctrl/takeoff_ready', '/px4ctrl/automatic_control_ok',
    '/px4ctrl/takeoff_height', '/px4ctrl/takeoff_land', '/px4ctrl/abort_automatic',
    '/goal', '/setpoints_cmd', '/planning/yaw', '/planning/stop',
    '/drone_0_planning/trajectory', '/drone_0_traj_server/heartbeat',
    '/drone_0_traj_server/output_ready', '/indoor_mission/fence', '/indoor_mission/map_ready',
    '/rosout_agg',
]

RC_RATE_CMD = ['rosrun', 'mavros', 'mavcmd', 'long', '511', '147', '200000', '0', '0', '0', '0', '0']


def running(pattern):
    try:
        result = subprocess.run(['pgrep', '-f', pattern], stdout=subprocess.PIPE,
                                stderr=subprocess.DEVNULL, text=True, timeout=5.)
    except (OSError, subprocess.SubprocessError):
        return []
    return [int(item) for item in result.stdout.split() if item.isdigit()]


def spawn(args, log_path):
    log = open(log_path, 'ab', buffering=0)
    subprocess.Popen(['setsid'] + args, stdin=subprocess.DEVNULL,
                     stdout=log, stderr=log, close_fds=True)


class Initializer:
    """把飞行前置链拉到就绪状态；进度通过 properties 暴露给遥测。"""

    def __init__(self, clear_executor=None):
        self.lock = threading.Lock()
        self.phase = 'idle'
        self.detail = ''
        self.ok = False
        self.thread = None
        self.clear_executor = clear_executor

    # ── 对外接口 ────────────────────────────────────────────────
    def snapshot(self):
        with self.lock:
            return self.phase, self.detail, self.ok

    def busy(self):
        with self.lock:
            return self.phase not in ('idle', 'ready', 'failed')

    def start(self, bounds, project_dir, takeoff_height=None):
        with self.lock:
            if self.phase not in ('idle', 'ready', 'failed', 'awaiting_calibration'):
                return False, '初始化正在进行中'
            resume = self.phase == 'awaiting_calibration'
            self.phase = 'starting'
            self.detail = '准备中'
            self.ok = False
        self.thread = threading.Thread(target=self._run, args=(bounds, project_dir, takeoff_height, resume),
                                       daemon=True)
        self.thread.start()
        return True, '已开始初始化'

    # ── 内部实现 ────────────────────────────────────────────────
    def _set(self, phase, detail, ok=False):
        with self.lock:
            self.phase, self.detail, self.ok = phase, detail, ok
        rospy.loginfo('初始化[%s] %s', phase, detail)

    def _wait_message(self, topic, cls, timeout):
        try:
            rospy.wait_for_message(topic, cls, timeout=timeout)
            return True
        except rospy.ROSException:
            return False

    def _wait_bool(self, topic, want, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            try:
                msg = rospy.wait_for_message(topic, Bool, timeout=min(1., timeout))
            except rospy.ROSException:
                continue
            if bool(msg.data) == want:
                return True
        return False

    def _run(self, bounds, project_dir, takeoff_height, resume=False):
        try:
            self._prepare_ground_services()
            self._require_ground()
            if self.clear_executor is not None:
                self.clear_executor()
            if resume:
                self._finish_calibration(bounds, project_dir)
            else:
                self._restart_all(project_dir, takeoff_height)
        except Exception as error:      # noqa: BLE001 - 任何异常都要落到 failed
            self._set('failed', '初始化异常：%s' % str(error)[:160])

    def _stop_launchers(self, pattern, name, timeout=15.):
        """Stop only the named launchers; do not kill unrelated user processes."""
        pids = running(pattern)
        for pid in pids:
            if pid == os.getpid():
                raise RuntimeError('拒绝停止任务网关自身')
            try:
                os.kill(pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + timeout
        while running(pattern) and time.monotonic() < deadline:
            time.sleep(.2)
        if running(pattern):
            raise RuntimeError('%s 未正常退出，请检查进程' % name)

    def _require_ground(self):
        try:
            state = rospy.wait_for_message('/mavros/state', State, timeout=3.)
            landed = rospy.wait_for_message('/mavros/extended_state', ExtendedState, timeout=3.)
        except rospy.ROSException:
            raise RuntimeError('无法确认飞控已落地并上锁')
        if not state.connected or state.armed or landed.landed_state != ExtendedState.LANDED_STATE_ON_GROUND:
            raise RuntimeError('飞控未确认落地并上锁，禁止重启')

    def _prepare_ground_services(self):
        """Start missing passive status publishers before the ground interlock."""
        self._ensure_mavros()
        self._require_ground()
        if not self._wait_message('/mission/remote/status', MissionStatus, 2.):
            if running('devel/lib/multipoint/multipointplan'):
                raise RuntimeError('任务执行器正在运行但没有状态，未重复启动')
            spawn(['roslaunch', 'multipoint', 'reinitialized_components.launch',
                   'precision_landing:=false'], '/tmp/reinitialized_components.log')
            if not self._wait_message('/mission/remote/status', MissionStatus, 25.):
                raise RuntimeError('任务执行器未发布状态')

    def _ensure_mavros(self):
        self._set('mavros', '正在检查飞控连接')
        try:
            state = rospy.wait_for_message('/mavros/state', State, timeout=2.)
        except rospy.ROSException:
            state = None
        if state is None and not running('mavros_node'):
            workspace = os.path.realpath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))
            helper = os.path.join(workspace, 'tools', 'prepare_mapping_profile.py')
            output = os.path.expanduser('~/.local/state/agri-gcs/mapping-start/profiles')
            result = subprocess.run(['/usr/bin/python3', helper, '--output-dir', output],
                                    stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                    text=True, timeout=20., check=True)
            with open(os.path.join(result.stdout.strip(), 'profile.json'), encoding='utf-8') as handle:
                fcu_url = json.load(handle)['fcu_url']
            spawn(['roslaunch', 'mavros', 'px4.launch', 'gcs_url:=', 'fcu_url:=' + fcu_url],
                  '/tmp/mavros.init.log')
        deadline = time.monotonic() + 25.
        while (state is None or not state.connected) and time.monotonic() < deadline:
            try:
                state = rospy.wait_for_message('/mavros/state', State, timeout=2.)
            except rospy.ROSException:
                state = None
        if state is None or not state.connected:
            raise RuntimeError('飞控连接未恢复，未重启飞行服务')

    def _restart_all(self, project_dir, takeoff_height):
        self._require_ground()
        if not os.path.isabs(project_dir):
            raise RuntimeError('项目目录必须是绝对路径')
        # Scripts and binaries must belong to this same workspace, not an arbitrary
        # path provided by the client.
        workspace = os.path.realpath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))
        mapping_script = os.path.join(workspace, 'sh_files', 'start_mapping_only.sh')
        if not os.path.isfile(mapping_script):
            raise RuntimeError('找不到建图启动脚本：%s' % mapping_script)
        precision = bool(rospy.get_param('/multipointplan/precision_landing_enabled', False))
        if takeoff_height is None:
            try:
                takeoff_height = rospy.wait_for_message('/px4ctrl/takeoff_height', Float64, timeout=2.).data
            except rospy.ROSException:
                takeoff_height = float(rospy.get_param('~init_takeoff_height', .5))

        self._set('restart', '正在停止旧录包和飞行服务')
        self._stop_launchers('rosbag record.*roundtrip_', '旧录包')
        self._stop_launchers('roslaunch.*planner.launch', '规划器')
        self._stop_launchers('roslaunch.*run_ctrl_indoor_automatic.launch', '飞行控制器')
        self._stop_launchers('roslaunch.*ekf_lidar.launch', 'EKF')
        self._stop_launchers('roslaunch.*reinitialized_components.launch', '任务组件')
        # The original roslaunch also owns the gateway; stop only its other ROS
        # nodes. The launcher itself and /gcs_gateway must remain alive.
        nodes = ['/multipointplan', '/siyi_a8_mini', '/apriltag_ros_continuous_node', '/gimbal_landing']
        listed = subprocess.run(['rosnode', 'list'], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                text=True, timeout=5., check=True).stdout.splitlines()
        for node in nodes:
            if node in listed:
                subprocess.run(['rosnode', 'kill', node], stdout=subprocess.DEVNULL,
                               stderr=subprocess.PIPE, text=True, timeout=10., check=True)
        deadline = time.monotonic() + 10.
        while time.monotonic() < deadline:
            active = subprocess.run(['rosnode', 'list'], stdout=subprocess.PIPE,
                                    stderr=subprocess.PIPE, text=True, timeout=5., check=True).stdout.splitlines()
            if not any(node in active for node in nodes):
                break
            time.sleep(.2)
        else:
            raise RuntimeError('旧任务或视觉节点未正常退出')
        self._require_ground()
        self._stop_launchers('roslaunch.*mapping_mid360.launch', '雷达建图')
        self._stop_launchers('roslaunch.*mavros.*px4.launch', 'MAVROS')
        # Never silently reuse an independently launched flight component.
        # Its owner must be identified before it can be stopped safely.
        for pattern, name in [('px4ctrl_node', '飞行控制器'),
                              ('run_mapping_online', '雷达建图'),
                              ('livox_ros_driver2_node', '雷达驱动'),
                              ('mavros_node', 'MAVROS')]:
            if running(pattern):
                raise RuntimeError(name + '仍在运行，启动方式无法确认，请检查后重试')

        self._set('mapping', '正在重新连接飞控并启动雷达建图')
        result = subprocess.run(['/bin/bash', mapping_script], stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, text=True, timeout=110.)
        if result.returncode:
            raise RuntimeError('建图重启失败：' + result.stdout[-500:])
        # Some mapping profiles do not use MAVROS IMU and therefore do not
        # start MAVROS. Restore it before any flight component is started.
        self._ensure_mavros()
        self._require_ground()
        try:
            subprocess.run(RC_RATE_CMD, stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL, timeout=8.)
        except (OSError, subprocess.SubprocessError):
            rospy.logwarn('mavcmd 配置遥控速率失败，继续执行')
        self._set('ekf', '正在重新启动 EKF')
        spawn(['roslaunch', 'ekf', 'ekf_lidar.launch'], '/tmp/ekf.launch.log')
        if not self._wait_message('/ekf/ekf_odom', Odometry, 25.):
            raise RuntimeError('EKF 未在 25 秒内输出里程计')
        self._set('px4ctrl', '正在重新启动飞行控制器')
        if not self._ensure_controller(takeoff_height):
            return
        self._set('components', '正在重新启动任务和二维码组件')
        spawn(['roslaunch', 'multipoint', 'reinitialized_components.launch',
               'precision_landing:=' + ('true' if precision else 'false')], '/tmp/reinitialized_components.log')
        self._set('bag', '正在开始新的飞行录包')
        log_dir = os.path.expanduser('~/flight_logs')
        os.makedirs(log_dir, exist_ok=True)
        bag = os.path.join(log_dir, 'roundtrip_%s.bag' % time.strftime('%Y%m%d_%H%M%S'))
        spawn(['rosbag', 'record', '-O', bag] + BAG_TOPICS, '/tmp/rosbag.log')
        if not self._wait_message('/mission/remote/status', MissionStatus, 25.):
            raise RuntimeError('任务执行器未重新发布状态')
        deadline = time.monotonic() + 5.
        while not running('rosbag record.*roundtrip_') and time.monotonic() < deadline:
            time.sleep(.2)
        if not running('rosbag record.*roundtrip_'):
            raise RuntimeError('新的 rosbag 未启动，请检查 /tmp/rosbag.log')
        self._set('awaiting_calibration', '服务已重启，请刷新新地图位置并重新确认围栏与起降点')

    def _finish_calibration(self, bounds, project_dir):
        self._require_ground()
        self._set('planner', '正在按重新确认的围栏启动规划器')
        if not self._restart_planner(bounds, project_dir):
            return
        if not self._wait_bool('/drone_0_traj_server/output_ready', True, 30.):
            raise RuntimeError('规划器 30 秒内未输出轨迹')
        if not self._wait_bool('/indoor_mission/map_ready', True, 30.):
            raise RuntimeError('地图坐标校验 30 秒内未通过')
        self._set('ready', '初始化完成，请重新标定起降点', ok=True)

    def _sequence(self, bounds, project_dir, takeoff_height=None):
        # 1) 飞控链路必须在
        self._set('mavros', '检查飞控链路')
        if not self._wait_message('/mavros/state', State, 5.):
            self._set('failed', '飞控链路未就绪（/mavros/state 无数据），请先启动 mavros')
            return

        # 2) 遥控通道速率（失败只告警，不阻断整条链）
        self._set('rc', '配置遥控通道速率')
        try:
            subprocess.run(RC_RATE_CMD, stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL, timeout=8.)
        except (OSError, subprocess.SubprocessError):
            rospy.logwarn('mavcmd 配置遥控速率失败，继续执行')

        # 3) EKF：没跑才启动；等里程计出现
        self._set('ekf', '检查 EKF')
        if not running('ekf_lidar.launch'):
            spawn(['roslaunch', 'ekf', 'ekf_lidar.launch'], '/tmp/ekf.launch.log')
        if not self._wait_message('/ekf/ekf_odom', Odometry, 25.):
            self._set('failed', 'EKF 未在 25 s 内输出 /ekf/ekf_odom')
            return

        # 4) 高度在控制器启动时读取；已运行且高度相同则不重启。
        self._set('px4ctrl', '检查飞行控制器和起飞高度')
        if not self._ensure_controller(takeoff_height):
            return

        # 5) 规划器：总是重启（围栏要换）；等轨迹输出与地图校验同时就绪
        self._set('planner', '按新围栏重启规划器')
        if not self._restart_planner(bounds, project_dir):
            return
        if not self._wait_bool('/drone_0_traj_server/output_ready', True, 30.):
            self._set('failed', '规划器 30 s 内未输出轨迹（output_ready）')
            return
        if not self._wait_bool('/indoor_mission/map_ready', True, 30.):
            self._set('failed', '地图坐标校验 30 s 内未通过（map_ready）')
            return

        # 6) 任务适配器就是当前进程，不重启，只确认端口在手
        self._set('gateway', '任务适配器在线（当前进程）')

        # 7) 录制
        self._set('bag', '启动数据录制')
        if not running('rosbag record'):
            log_dir = os.path.expanduser('~/flight_logs')
            os.makedirs(log_dir, exist_ok=True)
            bag = os.path.join(log_dir, 'roundtrip_%s.bag' % time.strftime('%Y%m%d_%H%M%S'))
            spawn(['rosbag', 'record', '-O', bag] + BAG_TOPICS, '/tmp/rosbag.log')
            time.sleep(1.)
            if not running('rosbag record'):
                self._set('failed', 'rosbag 启动失败，见 /tmp/rosbag.log')
                return

        self._set('ready', '初始化完成：规划器已按新围栏重启，请重新标定起降点', ok=True)

    # ── 子步骤 ──────────────────────────────────────────────────
    def _ensure_controller(self, requested):
        active = bool(running('px4ctrl_node'))
        if active and requested is not None:
            try:
                actual = rospy.wait_for_message('/px4ctrl/takeoff_height', Float64, timeout=5.).data
            except rospy.ROSException:
                self._set('failed', '无法读取已运行控制器的起飞高度，未更改控制器')
                return False
            if not math.isfinite(actual) or abs(actual - requested) > 0.001:
                # 仅接管本初始化流程使用的启动方式，避免覆盖自定义控制器配置。
                launchers = running('roslaunch.*run_ctrl_indoor_automatic.launch')
                if len(launchers) != 1:
                    self._set('failed', '控制器高度不一致且启动方式无法确认，请在地面检查控制器')
                    return False
                try:
                    state = rospy.wait_for_message('/mavros/state', State, timeout=3.)
                    landed = rospy.wait_for_message('/mavros/extended_state', ExtendedState, timeout=3.)
                except rospy.ROSException:
                    self._set('failed', '无法确认飞控已落地并锁定，未更改控制器')
                    return False
                if not state.connected or state.armed or landed.landed_state != ExtendedState.LANDED_STATE_ON_GROUND:
                    self._set('failed', '飞控未确认落地并锁定，未更改控制器')
                    return False
                self._set('px4ctrl', '起飞高度已更改，正在重启控制器')
                try:
                    os.kill(launchers[0], signal.SIGINT)
                except OSError as error:
                    self._set('failed', '无法停止旧控制器：%s' % error)
                    return False
                deadline = time.monotonic() + 10.
                while time.monotonic() < deadline and (running('px4ctrl_node') or
                                                       running('roslaunch.*run_ctrl_indoor_automatic.launch')):
                    time.sleep(.2)
                if running('px4ctrl_node') or running('roslaunch.*run_ctrl_indoor_automatic.launch'):
                    self._set('failed', '旧控制器未退出，请检查控制器进程')
                    return False
                active = False
        if not active:
            height = requested if requested is not None else float(rospy.get_param('~init_takeoff_height', 0.5))
            spawn(['roslaunch', 'px4ctrl', 'run_ctrl_indoor_automatic.launch',
                   'takeoff_height:=%.6f' % height], '/tmp/px4ctrl.launch.log')
        if not self._wait_message('/px4ctrl/takeoff_ready', Bool, 25.):
            self._set('failed', 'px4ctrl 未在 25 s 内发布 /px4ctrl/takeoff_ready')
            return False
        if requested is not None:
            try:
                actual = rospy.wait_for_message('/px4ctrl/takeoff_height', Float64, timeout=5.).data
            except rospy.ROSException:
                self._set('failed', '无法确认控制器的实际起飞高度')
                return False
            if not math.isfinite(actual) or abs(actual - requested) > 0.001:
                self._set('failed', '控制器起飞高度未生效（期望 %.2f m，实际 %.2f m）' % (requested, actual))
                return False
        return True

    def _restart_planner(self, bounds, project_dir):
        if not isinstance(bounds, list) or len(bounds) != 6:
            self._set('failed', '围栏参数无效')
            return False
        if not os.path.isabs(project_dir):
            self._set('failed', '项目目录必须是绝对路径')
            return False
        launch = None
        tried = []
        for candidate in (os.path.join(project_dir, 'point-mission', 'manual', 'planner.launch'),
                          os.path.join(project_dir, 'agri-gcs', 'point-mission', 'manual',
                                       'planner.launch')):
            tried.append(candidate)
            if os.path.isfile(candidate):
                launch = candidate
                break
        if launch is None:
            self._set('failed', '找不到规划器启动文件，已尝试：' + ' 与 '.join(tried))
            return False
        fence = '[%s]' % ','.join('%g' % value for value in bounds)
        for pid in running('roslaunch.*planner.launch'):
            try:
                os.kill(pid, signal.SIGINT)
            except OSError:
                pass
        deadline = time.monotonic() + 10.
        while time.monotonic() < deadline and running('roslaunch.*planner.launch'):
            time.sleep(0.2)
        for pid in running('roslaunch.*planner.launch'):
            try:
                os.kill(pid, signal.SIGKILL)
            except OSError:
                pass
        subprocess.run(['pkill', '-f', 'devel/lib/diff_planner/'],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        spawn(['roslaunch', launch, 'bounds:=' + fence,
               'floor:=%g' % bounds[2], 'ceiling:=%g' % bounds[5]], '/tmp/planner.launch.log')
        return True
