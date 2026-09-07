#!/usr/bin/env python3
"""Read-only analysis of the archived run; write derived evidence beside this script."""
from pathlib import Path
from datetime import datetime, timezone, timedelta
import collections
import csv
import hashlib
import json
import math
import re
import statistics

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
RUN = ROOT / 'flight_logs/20260907_154114'
TZ = timezone(timedelta(hours=8))


def stats(values):
    a = sorted(v for v in values if v is not None and math.isfinite(v))
    return dict(n=len(a), min=a[0], median=statistics.median(a),
                p95=a[int(.95 * (len(a) - 1))], max=a[-1]) if a else None


def jsonl(name):
    return [json.loads(line) for line in (RUN / name).read_text().splitlines()]


def clock(sec):
    return datetime.fromtimestamp(sec, TZ).isoformat(timespec='milliseconds')


def main():
    host = [r for r in jsonl('host_metrics.jsonl') if r['event'] == 'sample']
    tegra = [r for r in jsonl('tegrastats.jsonl') if r['event'] == 'output']
    pings = []
    for row in jsonl('camera_ping.jsonl'):
        m = re.search(r'icmp_seq=(\d+).*time=([\d.]+)', row.get('text', ''))
        if m:
            pings.append((row['unix_sec'], int(m[1]), float(m[2])))
    text = (RUN / 'console.log').read_text(errors='replace')
    fps, diagnostic, events = [], [], []
    for line in text.splitlines():
        stamps = re.findall(r'2026-09-07T[\d:.]+\+08:00', line)
        sec = datetime.fromisoformat(stamps[-1]).timestamp() if stamps else None
        m = re.search(r'\[STATS\] ffmpeg FPS ([\d.]+) CAP ([\d.]+) ms YOLO ([\d.]+) ms AGE ([\d.]+)', line)
        if m:
            fps.append((sec, *map(float, m.groups())))
        if '[DIAG] ' in line:
            diagnostic.append((sec, json.loads(line.split('[DIAG] ', 1)[1])))
        m = re.search(r'\[(178876\d+\.\d+)\]: (.*(?:Waypoint \d+ arrived|Published waypoint|gimbal done|AUTO_TAKEOFF|AUTO_LAND|AUTO_HOVER|MANUAL_CTRL).*)', line)
        if m:
            events.append({'unix_sec': float(m[1]), 'event': re.sub(r'\x1b\[[0-9;]*m', '', m[2])})

    import rosbag
    odom, batteries = [], []
    topics = ['/ekf/ekf_odom', '/laserMapping/odometry', '/mavros/imu/data',
              '/mavros/setpoint_raw/attitude', '/mavros/local_position/odom',
              '/mavros/state', '/mavros/extended_state', '/mavros/battery',
              '/mission/gimbal_task', '/mission/gimbal_done', '/goal', '/px4ctrl/takeoff_land']
    times = collections.defaultdict(list)
    changes, previous = [], {}
    with rosbag.Bag(str(RUN / 'flight_debug.bag')) as bag:
        bag_summary = {'start': clock(bag.get_start_time()), 'end': clock(bag.get_end_time()),
                       'messages': bag.get_message_count(),
                       'topic_counts': {k: v.message_count for k, v in bag.get_type_and_topic_info().topics.items()}}
        (OUT / 'rosbag_info.rechecked.txt').write_text(str(bag) + '\n')
        for topic, message, t in bag.read_messages(topics=topics):
            sec = t.to_sec()
            times[topic].append(sec)
            if topic == '/ekf/ekf_odom':
                p, v = message.pose.pose.position, message.twist.twist.linear
                odom.append([sec, p.x, p.y, p.z, v.x, v.y, v.z])
            if topic == '/mavros/battery':
                batteries.append([sec, message.voltage, message.current, message.percentage])
            value = None
            if topic == '/mavros/state':
                value = [message.connected, message.armed, message.mode]
            elif topic == '/mavros/extended_state':
                value = message.landed_state
            elif topic.startswith('/mission/gimbal'):
                value = list(message.data) if topic.endswith('task') else message.data
            elif topic == '/goal':
                value = [message.pose.position.x, message.pose.position.y, message.pose.position.z]
            elif topic == '/px4ctrl/takeoff_land':
                value = message.takeoff_land_cmd
            if value is not None and previous.get(topic) != value:
                changes.append({'unix_sec': sec, 'wall_time': clock(sec), 'topic': topic, 'value': value})
                previous[topic] = value

    # Stable round boundaries matching the observed armed/ON_GROUND transition samples.
    air_start, air_end = 1788766971.625, 1788767077.134
    phases = {'preflight': (1788766946, 1788766971), 'airborne': (air_start, air_end),
              'landed': (air_end, 1788767166), 'scan1': (1788767003.853, 1788767014.075),
              'scan2': (1788767026.754, 1788767036.975), 'all_streaming': (1788766944, 1788767166)}
    summary = {'run': RUN.name, 'bag': bag_summary, 'state_changes': changes,
               'detector_final': diagnostic[-1][1], 'host_sample_count': len(host),
               'detector_stats_count': len(fps), 'diag_count': len(diagnostic),
               'stream_errors': {key: text.count(key) for key in ('Stream timeout triggered',
                   'Could not find ref', 'Error constructing the frame RPS', '[STATS] RTSP:', 'SIYI command 0x0E timed out')},
               'detector_samples': {name: stats(r[i] for r in fps) for i, name in enumerate(['time', 'fps', 'cap_ms', 'yolo_ms', 'local_age_ms']) if i},
               'detector_window_peaks': {key: max(r[1]['window_max'][key] for r in diagnostic) for key in ('cap_ms', 'yolo_ms', 'local_age_ms')},
               'phases': {}, 'sampler_cost_ms': stats(r['sample_cost_ms'] for r in host),
               'sampler_interval_sec': stats(r['sample_interval_sec'] for r in host[1:]),
               'ping_rtt_ms': stats(p[2] for p in pings),
               'ping_missing_sequences': sorted(set(range(pings[0][1], pings[-1][1] + 1)) - {p[1] for p in pings}),
               'eth0_counter_delta': {k: host[-1]['network']['eth0'][k] - host[0]['network']['eth0'][k]
                   for k in ('rx_bytes', 'tx_bytes', 'rx_errors', 'rx_dropped', 'tx_errors', 'tx_dropped')},
               'eth0_states': sorted({(r['network']['eth0']['operstate'], r['network']['eth0']['carrier'], r['network']['eth0']['speed']) for r in host}),
               'kernel_records': jsonl('kernel.jsonl')}
    for name, (lo, hi) in phases.items():
        hh = [r for r in host if lo <= r['unix_sec'] <= hi]
        ff = [r for r in fps if lo <= r[0] <= hi]
        tt = [r for r in tegra if lo <= r['unix_sec'] <= hi]
        fields = {'fps': stats(r[1] for r in ff), 'yolo_ms': stats(r[3] for r in ff),
                  'cpu_percent': stats(r['cpu']['cpu']['busy_percent'] for r in hh),
                  'iowait_percent': stats(r['cpu']['cpu']['iowait_percent'] for r in hh),
                  'eth0_rx_mbps': stats(r['network']['eth0']['rx_bytes_per_sec'] * 8 / 1e6 for r in hh)}
        for key, pattern in [('gpu_percent', r'GR3D_FREQ (\d+)%'), ('RAM_MB', r'RAM (\d+)/'),
                              ('swap_MB', r'SWAP (\d+)/'), ('cpu_C', r'CPU@([\d.]+)C'),
                              ('gpu_C', r'GPU@([\d.]+)C'), ('tj_C', r'tj@([\d.]+)C')]:
            fields[key] = stats(float(m[1]) for r in tt for m in [re.search(pattern, r['text'])] if m)
        summary['phases'][name] = fields
    summary['flight_topic_rates'] = {}
    for topic, ts in times.items():
        ts = [t for t in ts if air_start <= t <= air_end]
        if len(ts) < 3:
            continue
        gap, at = max((b - a, a) for a, b in zip(ts, ts[1:]))
        summary['flight_topic_rates'][topic] = {'n': len(ts), 'hz': (len(ts) - 1) / (ts[-1] - ts[0]),
                                               'max_gap_ms': gap * 1000, 'max_gap_at': clock(at)}
    op = [r for r in odom if 1788766981.587 <= r[0] <= 1788767064.43]
    summary['mission_height_m'] = stats(r[3] for r in op)
    summary['mission_peak_speed_mps'] = max(math.sqrt(sum(v*v for v in r[4:7])) for r in op)
    summary['positions'] = {}
    for name, sec in [('return_arrived', 1788767053.702), ('land_command', 1788767064.431), ('on_ground', air_end)]:
        summary['positions'][name] = min(odom, key=lambda r: abs(r[0] - sec))[:4]
    summary['battery_in_flight'] = {name: stats(r[i] for r in batteries if air_start <= r[0] <= air_end)
                                    for i, name in enumerate(['time', 'voltage', 'current', 'percentage']) if i}
    active = [r for r in host if 1788766944 <= r['unix_sec'] <= 1788767166]
    summary['processes'] = {}
    for pid in (12279, 13723, 13846, 13726, 12809, 13062, 12492):
        pp = [p for r in active for p in r['processes'] if p['pid'] == pid]
        summary['processes'][pid] = {'name': pp[0]['name'], 'cpu_one_core_percent': stats(p['cpu_percent_one_core'] for p in pp),
                                    'rss_MiB': stats(p['rss_bytes']/1048576 for p in pp), 'threads': sorted({p['threads'] for p in pp})}
    manifest = json.loads((RUN / 'detector_runtime_13723/manifest.json').read_text())
    summary['source_hashes_verified'] = {name: hashlib.sha256((RUN / 'detector_runtime_13723' / (Path(name).name + '.snapshot')).read_bytes()).hexdigest() == info['sha256']
                                         for name, info in manifest['sources'].items()}
    summary['land_republish_warning_count'] = text.count('Landing remains latched;')
    source_csv = Path('/tmp/a8mini_mission_timestamps.csv')
    if source_csv.exists():
        rows = list(csv.DictReader(source_csv.open()))
        done = {c['value']: c['unix_sec'] for c in changes if c['topic'] == '/mission/gimbal_done'}
        if len(rows) == 2 and all(abs(float(r['gimbal_done_time']) - done.get(int(r['waypoint_id']), 0)) < .01 for r in rows):
            (OUT / 'mission_timestamps.recovered.csv').write_bytes(source_csv.read_bytes())
            summary['mission_csv_recovery'] = {'source': str(source_csv), 'verified_against': '/mission/gimbal_done in bag', 'sha256': hashlib.sha256(source_csv.read_bytes()).hexdigest()}
    (OUT / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n')
    with (OUT / 'event_timeline.csv').open('w') as f:
        writer = csv.writer(f)
        writer.writerow(['wall_time', 'unix_sec', 'source', 'event'])
        for e in sorted(events + [{'unix_sec': c['unix_sec'], 'event': c['topic'] + ': ' + str(c['value'])} for c in changes], key=lambda e:e['unix_sec']):
            writer.writerow([clock(e['unix_sec']), '%.6f' % e['unix_sec'], 'console/rosbag', e['event']])

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import matplotlib.dates as dates
    fig, axes = plt.subplots(5, 1, figsize=(13, 12), sharex=True, constrained_layout=True)
    date = lambda sec: datetime.fromtimestamp(sec, TZ)
    axes[0].plot([date(r[0]) for r in fps], [r[1] for r in fps], label='Detection FPS', color='#2766a7')
    capture_rate = [(diagnostic[i][0], (diagnostic[i][1]['capture']['frames_received'] - diagnostic[i-1][1]['capture']['frames_received']) / (diagnostic[i][0] - diagnostic[i-1][0])) for i in range(1, len(diagnostic))]
    axes[0].plot([date(t) for t,v in capture_rate], [v for t,v in capture_rate], label='Capture FPS (counter delta)', color='#2e8b57', alpha=.75)
    axes[0].set_ylabel('Frames / sec'); axes[0].legend(loc='upper left', ncol=2)
    axes[1].plot([date(t) for t,r in diagnostic], [r['window_max']['yolo_ms'] for t,r in diagnostic], label='YOLO window max', color='#e6842a')
    axes[1].plot([date(t) for t,r in diagnostic], [r['window_max']['local_age_ms'] for t,r in diagnostic], label='Local age window max', color='#7c519f')
    axes[1].set_ylabel('Milliseconds'); axes[1].legend(loc='upper left', ncol=2)
    axes[2].plot([date(r['unix_sec']) for r in host], [r['cpu']['cpu']['busy_percent'] for r in host], label='Host CPU %', color='#2766a7')
    gpu = [(r['unix_sec'], float(re.search(r'GR3D_FREQ (\d+)', r['text'])[1])) for r in tegra if re.search(r'GR3D_FREQ (\d+)', r['text'])]
    axes[2].plot([date(t) for t,v in gpu], [v for t,v in gpu], label='Jetson GPU %', color='#b54648', alpha=.7)
    axes[2].set_ylabel('Utilization %'); axes[2].legend(loc='upper left', ncol=2)
    axes[3].plot([date(r[0]) for r in pings], [r[2] for r in pings], color='#2e8b57')
    axes[3].set_ylabel('Ping RTT / ms')
    short = odom[::17]
    axes[4].plot([date(r[0]) for r in short], [r[3] for r in short], label='EKF z / m', color='#2766a7')
    axes[4].plot([date(r[0]) for r in short], [math.hypot(r[1],r[2]) for r in short], label='Distance from origin / m', color='#888888', alpha=.8)
    axes[4].set_ylabel('Meters'); axes[4].legend(loc='upper left', ncol=2)
    for ax in axes:
        ax.axvspan(date(air_start), date(air_end), color='#dedede', alpha=.3)
        for phase in ('scan1', 'scan2'):
            ax.axvspan(date(phases[phase][0]), date(phases[phase][1]), color='#eed393', alpha=.4)
        ax.grid(alpha=.2)
        ax.set_xlim(date(1788766940), date(1788767167))
    axes[-1].xaxis.set_major_formatter(dates.DateFormatter('%H:%M:%S', tz=TZ))
    axes[-1].set_xlabel('2026-09-07 local time (UTC+08:00); gray: airborne, yellow: gimbal scans')
    fig.suptitle('20260907_154114: continuous capture, no RTSP reconnects')
    fig.savefig(OUT / 'flight_diagnostics.png', dpi=140)
    plt.close(fig)
    print('Wrote summary.json, event_timeline.csv, bag recheck, trend figure and verified recovered CSV')


if __name__ == '__main__':
    main()
