#!/usr/bin/env python3
"""Validate one aircraft profile and render immutable Livox/Faster-LIO settings."""
import argparse
import hashlib
import ipaddress
import json
import math
import os
from pathlib import Path
import pwd
import re
import tempfile

import yaml


class ProfileError(ValueError):
    pass


def choose_profile(root, selection=None, user=None):
    selection = selection or os.environ.get('AGRI_MAPPING_PROFILE')
    if not selection:
        selection = user or pwd.getpwuid(os.getuid()).pw_name
    path = Path(selection).expanduser()
    if not path.is_absolute():
        if not re.fullmatch(r'[A-Za-z0-9_-]+(?:\.yaml)?', selection):
            raise ProfileError('Use a profile name or an absolute YAML path')
        path = root / 'config/mapping_profiles' / (selection if selection.endswith('.yaml') else selection + '.yaml')
    if not path.is_file():
        raise ProfileError('No mapping profile: {}. Create one or set AGRI_MAPPING_PROFILE.'.format(path))
    return path.resolve()


def fields(value, required, where):
    if not isinstance(value, dict) or set(value) != set(required):
        raise ProfileError('{} must contain exactly: {}'.format(where, ', '.join(required)))
    return value


def text(value, where):
    if not isinstance(value, str) or not value or len(value) > 256 or any(c.isspace() for c in value):
        raise ProfileError('{} must be a nonempty string without whitespace'.format(where))
    return value


def vector(value, length, where):
    if not isinstance(value, list) or len(value) != length:
        raise ProfileError('{} must contain {} numbers'.format(where, length))
    if any(type(x) not in (int, float) or not math.isfinite(x) for x in value):
        raise ProfileError('{} must contain finite numbers'.format(where))
    return [float(x) for x in value]


def validate(data):
    fields(data, ('version', 'lidar', 'imu', 'extrinsics'), 'profile')
    if type(data['version']) is not int or data['version'] != 1:
        raise ProfileError('Only profile version 1 is supported')
    lidar = fields(data['lidar'], ('model', 'ip', 'host_ip', 'broadcast_code', 'publish_freq'), 'lidar')
    if lidar['model'] not in ('MID360', 'Mid360s'):
        raise ProfileError('lidar.model must be MID360 or Mid360s (SDK configuration format)')
    for name in ('ip', 'host_ip'):
        if not isinstance(lidar[name], str):
            raise ProfileError('lidar.{} must be an IPv4 string'.format(name))
        try:
            addr = ipaddress.IPv4Address(lidar[name])
        except (ValueError, TypeError, ipaddress.AddressValueError):
            raise ProfileError('lidar.{} must be an IPv4 address'.format(name))
        if addr.is_unspecified or addr.is_multicast or addr.is_loopback:
            raise ProfileError('lidar.{} must be a device/interface address'.format(name))
    if lidar['ip'] == lidar['host_ip']:
        raise ProfileError('Lidar and host IP addresses must differ')
    if not re.fullmatch(r'[A-Za-z0-9]{1,32}', text(lidar['broadcast_code'], 'lidar.broadcast_code')):
        raise ProfileError('Invalid broadcast code')
    if type(lidar['publish_freq']) not in (int, float) or lidar['publish_freq'] not in (5, 10, 20, 50, 100):
        raise ProfileError('Unsupported lidar.publish_freq (5, 10, 20, 50, 100 Hz)')
    imu = fields(data['imu'], ('topic', 'fcu_url'), 'imu')
    if imu['topic'] not in ('/mavros/imu/data', '/livox/imu'):
        raise ProfileError('imu.topic must be /mavros/imu/data or /livox/imu')
    text(imu['fcu_url'], 'imu.fcu_url')
    ext = fields(data['extrinsics'], ('extrinsic_T', 'extrinsic_R'), 'extrinsics')
    vector(ext['extrinsic_T'], 3, 'extrinsic_T')
    rot = vector(ext['extrinsic_R'], 9, 'extrinsic_R')
    rows = [rot[i:i+3] for i in (0, 3, 6)]
    for i in range(3):
        for j in range(3):
            if abs(sum(rows[i][k] * rows[j][k] for k in range(3)) - (1 if i == j else 0)) > .01:
                raise ProfileError('extrinsic_R must be an orthonormal rotation matrix (row-major)')
    a,b,c,d,e,f,g,h,i = rot
    if abs(a*(e*i-f*h) - b*(d*i-f*g) + c*(d*h-e*g) - 1) > .01:
        raise ProfileError('extrinsic_R determinant must be +1')
    return data


def render(root, profile_path, output_root):
    try:
        data = validate(yaml.safe_load(profile_path.read_text()))
    except yaml.YAMLError as exc:
        raise ProfileError('Invalid YAML: {}'.format(exc))
    lidar = data['lidar']
    template = root / 'src/livox_ros_driver2/config' / ('MID360_config.json' if lidar['model'] == 'MID360' else 'MID360s_config.json')
    base = root / 'src/realflight_modules/faster-lio/config/mid360.yaml'
    driver = json.loads(template.read_text())
    net = driver[lidar['model']]['host_net_info']
    if lidar['model'] == 'MID360':
        for field in ('cmd_data_ip', 'push_msg_ip', 'point_data_ip', 'imu_data_ip'):
            net[field] = lidar['host_ip']
    else:
        for interface in net:
            interface['host_ip'] = lidar['host_ip']
    if len(driver['lidar_configs']) != 1:
        raise ProfileError('Single-Lidar profiles require a single-device driver template')
    driver['lidar_configs'][0]['ip'] = lidar['ip']
    lio = yaml.safe_load(base.read_text())
    lio['common']['imu_topic'] = data['imu']['topic']
    for key in ('extrinsic_T', 'extrinsic_R'):
        lio['mapping'][key] = data['extrinsics'][key]
    lio['publish']['tf_world_frame'] = 'world'
    effective = dict(driver=driver, lio=lio, profile=data, source=str(profile_path))
    digest = hashlib.sha256(json.dumps(effective, sort_keys=True, allow_nan=False).encode()).hexdigest()
    meta = dict(profile=str(profile_path), digest=digest,
                broadcast_code=lidar['broadcast_code'], publish_freq=lidar['publish_freq'],
                imu_topic=data['imu']['topic'], fcu_url=data['imu']['fcu_url'])
    files = {'lidar.json': json.dumps(driver, indent=2)+'\n',
             'lio.yaml': yaml.safe_dump(lio, sort_keys=False),
             'profile.json': json.dumps(meta, indent=2)+'\n'}
    # A new profile/base-settings combination gets a separate directory. Never
    # overwrite files belonging to a currently running mapping process.
    output_root.mkdir(parents=True, exist_ok=True)
    dest = output_root / digest
    if dest.exists():
        for name, content in files.items():
            if (dest/name).read_text() != content:
                raise ProfileError('Generated runtime configuration has been altered: '+str(dest/name))
        return dest
    with tempfile.TemporaryDirectory(prefix='.profile-', dir=str(output_root)) as tmp:
        tmp = Path(tmp)
        for name, content in files.items():
            (tmp/name).write_text(content)
        tmp.rename(dest)
    return dest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', help='Profile name or absolute YAML path; default: AGRI_MAPPING_PROFILE, then login user')
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    try:
        path = choose_profile(root, args.profile)
        result = render(root, path, args.output_dir.resolve())
    except (OSError, ValueError, TypeError, KeyError) as exc:
        parser.exit(1, 'ERROR: Mapping profile: {}\n'.format(exc))
    print(result)


if __name__ == '__main__':
    main()
