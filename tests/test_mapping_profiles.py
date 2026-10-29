#!/usr/bin/env python3
"""Offline profile tests: no ROS master, driver or physical device is used."""
import copy
import importlib.util
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import yaml

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('profiles', ROOT/'tools/prepare_mapping_profile.py')
profiles = importlib.util.module_from_spec(spec)
spec.loader.exec_module(profiles)


class ProfileTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.out = Path(self.tmp.name)/'generated'
        self.data = yaml.safe_load((ROOT/'config/mapping_profiles/zzz.yaml').read_text())

    def tearDown(self):
        self.tmp.cleanup()

    def test_user_selection_and_no_fallback(self):
        with patch.dict(os.environ, {}, clear=True):
            self.assertEqual(profiles.choose_profile(ROOT, user='zzz').name, 'zzz.yaml')
            with self.assertRaises(profiles.ProfileError):
                profiles.choose_profile(ROOT, user='new_unconfigured_user')

    def test_environment_and_explicit_override(self):
        with patch.dict(os.environ, {'AGRI_MAPPING_PROFILE': 'ly'}):
            self.assertEqual(profiles.choose_profile(ROOT, user='zzz').name, 'ly.yaml')
            self.assertEqual(profiles.choose_profile(ROOT, 'zzz').name, 'zzz.yaml')

    def test_absolute_path_and_reject_ambiguous_relative_paths(self):
        path = (ROOT/'config/mapping_profiles/ly.yaml').resolve()
        self.assertEqual(profiles.choose_profile(ROOT, str(path)), path)
        with self.assertRaises(profiles.ProfileError):
            profiles.choose_profile(ROOT, '../ly.yaml')

    def test_both_aircraft_generate_their_own_network_and_calibration(self):
        for user, model, ip in [('ly', 'MID360', '192.168.1.190'), ('zzz', 'Mid360s', '192.168.1.193')]:
            path = ROOT/'config/mapping_profiles'/(user+'.yaml')
            original = path.read_bytes()
            data = yaml.safe_load(original)
            result = profiles.render(ROOT, path, self.out)
            driver = json.loads((result/'lidar.json').read_text())
            lio = yaml.safe_load((result/'lio.yaml').read_text())
            self.assertEqual(driver['lidar_configs'][0]['ip'], ip)
            host = driver[model]['host_net_info']
            if model == 'MID360':
                self.assertEqual(host['point_data_ip'], '192.168.1.5')
            else:
                self.assertEqual(host[0]['host_ip'], '192.168.1.5')
            self.assertEqual(lio['mapping']['extrinsic_T'], data['extrinsics']['extrinsic_T'])
            self.assertEqual(lio['mapping']['extrinsic_R'], data['extrinsics']['extrinsic_R'])
            self.assertEqual(lio['common']['imu_topic'], data['imu']['topic'])
            self.assertEqual(lio['publish']['tf_world_frame'], 'world')
            self.assertEqual(json.loads((result/'profile.json').read_text())['fcu_url'], '/dev/ttyACM0:57600')
            self.assertEqual(path.read_bytes(), original)

    def test_config_is_immutable_and_changes_get_new_directory(self):
        path = Path(self.tmp.name)/'custom.yaml'
        path.write_text(yaml.safe_dump(self.data))
        first = profiles.render(ROOT, path, self.out)
        saved = (first/'lio.yaml').read_bytes()
        self.assertEqual(profiles.render(ROOT, path, self.out), first)
        self.data['extrinsics']['extrinsic_T'][0] += .01
        path.write_text(yaml.safe_dump(self.data))
        self.assertNotEqual(profiles.render(ROOT, path, self.out), first)
        self.assertEqual((first/'lio.yaml').read_bytes(), saved)

    def test_changed_runtime_files_are_detected(self):
        path = ROOT/'config/mapping_profiles/zzz.yaml'
        result = profiles.render(ROOT, path, self.out)
        (result/'profile.json').write_text('{}')
        with self.assertRaises(profiles.ProfileError):
            profiles.render(ROOT, path, self.out)

    def test_reject_invalid_values_and_unknown_fields(self):
        bad = []
        for field, value in [('model','unknown'), ('ip','not-an-ip'), ('ip',123), ('host_ip','127.0.0.1'),
                             ('host_ip','192.168.1.193'), ('publish_freq',True), ('publish_freq',0),
                             ('broadcast_code','code with spaces')]:
            item=copy.deepcopy(self.data);item['lidar'][field]=value;bad.append(item)
        item=copy.deepcopy(self.data);item['imu']['topic']='/unknown_imu';bad.append(item)
        item=copy.deepcopy(self.data);item['imu']['fcu_url']='';bad.append(item)
        item=copy.deepcopy(self.data);item['extrinsics']['extrinsic_T']=[0,0,float('nan')];bad.append(item)
        item=copy.deepcopy(self.data);item['extrinsics']['extrinsic_R']=[0]*9;bad.append(item)
        item=copy.deepcopy(self.data);item['extrinsics']['extrinsic_R']=[-1,0,0,0,1,0,0,0,1];bad.append(item)
        item=copy.deepcopy(self.data);item['extrinsics']['extrinsic_R']=[1,0,0];bad.append(item)
        item=copy.deepcopy(self.data);item['typo']='value';bad.append(item)
        for data in bad:
            with self.subTest(data=data), self.assertRaises(profiles.ProfileError):
                profiles.validate(data)

    def test_lidar_imu_is_rendered_without_old_mavros_topic(self):
        self.data['imu']['topic']='/livox/imu'
        # The calibration must be supplied for this IMU separately in real use.
        path=Path(self.tmp.name)/'lidar-imu.yaml'
        path.write_text(yaml.safe_dump(self.data))
        result=profiles.render(ROOT,path,self.out)
        self.assertEqual(yaml.safe_load((result/'lio.yaml').read_text())['common']['imu_topic'], '/livox/imu')


if __name__ == '__main__':
    unittest.main()
