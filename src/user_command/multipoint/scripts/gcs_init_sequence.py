#!/usr/bin/env python3
"""任务初始化的机载编排：按依赖顺序把飞行前置链 bring-up 到位。

设计原则：
1. 幂等——已经在跑的进程不重复启动；网关自己所在的 multipoint 绝不重启。
2. 按依赖顺序执行，每一步等"就绪信号"而不是固定 sleep。
3. 异步——初始化立即返回，进度写入 phase/detail,由遥测上报给地面站。
4. 失败即停——某一步超时就中止，不启动依赖它的后续步骤。
"""
import os
import math
import signal
import subprocess
import threading
import time

import rospy
from mavros_msgs.msg import State, ExtendedState
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64

# 录制话题：与现场手工命令保持一致
BAG_TOPICS = [
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

    def __init__(self):
        self.lock = threading.Lock()
        self.phase = 'idle'
        self.detail = ''
        self.ok = False
        self.thread = None

    # ── 对外接口 ────────────────────────────────────────────────
    def snapshot(self):
        with self.lock:
            return self.phase, self.detail, self.ok

    def busy(self):
        with self.lock:
            return self.phase not in ('idle', 'ready', 'failed')

    def start(self, bounds, project_dir, takeoff_height=None):
        with self.lock:
            if self.phase not in ('idle', 'ready', 'failed'):
                return False, '初始化正在进行中'
            self.phase = 'starting'
            self.detail = '准备中'
            self.ok = False
        self.thread = threading.Thread(target=self._run, args=(bounds, project_dir, takeoff_height),
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

    def _run(self, bounds, project_dir, takeoff_height):
        try:
            self._sequence(bounds, project_dir, takeoff_height)
        except Exception as error:      # noqa: BLE001 - 任何异常都要落到 failed
            self._set('failed', '初始化异常：%s' % str(error)[:160])

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
