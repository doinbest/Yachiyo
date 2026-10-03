import json
from pathlib import Path
import sys
import unittest
import tempfile
import threading
import os
import shutil
import subprocess
from unittest.mock import patch
from urllib.request import Request, urlopen

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
import serial_bridge
import serve


class RadarServiceTests(unittest.TestCase):
    def test_radar_reads_and_scoped_cancel_classification(self):
        for command in ('radar status','radar get','radar fetch cloud 7 33','radar nav status'):
            self.assertIsNotNone(serial_bridge.READ_COMMAND.fullmatch(command),command)
        for command in ('radar stop','radar nav cancel'):
            self.assertIsNotNone(serial_bridge.PRIORITY_STOP.fullmatch(command),command)
        self.assertIsNone(serial_bridge.READ_COMMAND.fullmatch('radar task start 100'))

    def test_same_c_planner_offline_returns_station_path_without_a_serial_owner(self):
        result=serve.plan_radar_offline({'mask':0,'params':{'zero_deg':180},'points':[]})
        self.assertEqual(result['algorithm'],'radar_map_c_v1')
        self.assertTrue(result['valid'])
        self.assertEqual(result['points'][0][:2],[2250,150])
        stations=[point[2] for point in result['points'] if point[2]]
        self.assertEqual(stations,[1,5,4,2,3,4,2,3,1])

    def test_offline_cloud_filter_uses_c_scan_core(self):
        result=serve.plan_radar_offline({'mask':0,'params':{'lidar_x_mm':2250,'lidar_y_mm':150,'zero_deg':180,'energy_min':50},'points':[[1800,1000,20]]})
        self.assertEqual(sum(result['counts']),0)

    def test_invalid_offline_points_are_not_passed_as_shell_commands(self):
        with self.assertRaises(ValueError):
            serve.plan_radar_offline({'mask':0,'points':[['1; echo unexpected',5,2]]})

    def test_integer_filter_params_accept_json_integral_floats(self):
        result=serve.plan_radar_offline({'params':{'threshold':3.0,'angle_max_tenths':3600.0}})
        self.assertTrue(result['valid'])
        with self.assertRaises(ValueError):
            serve.plan_radar_offline({'params':{'threshold':3.5}})

    def test_shared_c_profiles_include_absolute_docking_and_first_scan_heading(self):
        result=serve.plan_radar_offline({'mask':0})
        self.assertIn('route_profiles',result)
        profiles=result['route_profiles']
        fixed=profiles['fixed'];planned=profiles['planned']
        self.assertEqual(fixed['speed_mm_s'],200)
        self.assertEqual(len(fixed['points']),16)
        self.assertEqual(fixed['points'][0],[2250,150,1,0])
        self.assertEqual(fixed['points'][-2:],[ [2100,300,0,0], [2250,150,1,8] ])
        self.assertEqual(planned['points'],result['points'])
        first=planned['phases'][0]
        self.assertEqual(first,dict(kind='move',phase='move',direction='diagonal',segment=1,
                                   station=5,visit=1,target=[2100,1200,90],speed_mm_s=200))
        docking=[p for p in fixed['phases'] if p['segment']==6]
        self.assertEqual([p['phase'] for p in docking],['turn','dock_approach','arrival_turn','dock'])
        self.assertEqual([p['target'] for p in docking],[[1200,1200,180],[425,1200,180],[425,1200,90],[350,1200,90]])
        self.assertEqual([p['speed_mm_s'] for p in docking],[0,200,0,50])
        leaving=[p for p in fixed['phases'] if p['segment']==7]
        self.assertEqual([p['phase'] for p in leaving],['undock','turn','move'])
        self.assertEqual([p['target'] for p in leaving],[[425,1200,90],[425,1200,180],[1200,1200,180]])
        keys={'kind','phase','direction','segment','station','visit','target','speed_mm_s'}
        for profile in profiles.values():
            for phase in profile['phases']:
                self.assertEqual(set(phase),keys)
        self.assertEqual(planned['phases'][-1]['target'],[2250,150,90])

    def test_planned_detour_keeps_first_scan_heading_until_qr_visit(self):
        result=serve.plan_radar_offline({'mask':1<<9})
        self.assertTrue(result['valid'])
        self.assertIn('route_profiles',result)
        profile=result['route_profiles']['planned'];qr_index=next(
            i for i,point in enumerate(profile['points']) if point[2:]==[5,1])
        self.assertGreater(qr_index,1)
        first_leg=[p for p in profile['phases'] if p['segment']<=qr_index]
        self.assertTrue(first_leg)
        self.assertTrue(all(p['target'][2]==90 for p in first_leg))
        self.assertEqual(first_leg[-1]['station'],5)
        self.assertEqual(first_leg[-1]['visit'],1)

    def test_optional_speed_changes_main_phases_and_keeps_docking_cap(self):
        for speed in (10,37.5,5000):
            result=serve.plan_radar_offline({'speed_mm_s':speed})
            self.assertIn('route_profiles',result)
            for profile in result['route_profiles'].values():
                self.assertEqual(profile['speed_mm_s'],speed)
                for phase in profile['phases']:
                    expected=0 if phase['kind']=='turn' else min(speed,50) if phase['phase'] in ('dock','undock') else speed
                    self.assertEqual(phase['speed_mm_s'],expected)
        for speed in (9.99,5000.01,0,-1,True,'200',float('nan'),float('inf')):
            with self.subTest(speed=speed),self.assertRaises(ValueError):
                serve.plan_radar_offline({'speed_mm_s':speed})

    def test_unreachable_plan_keeps_fixed_preview_and_empty_planned_phases(self):
        result=serve.plan_radar_offline({'mask':1<<14})
        self.assertFalse(result['valid'])
        self.assertIn('route_profiles',result)
        self.assertEqual(result['route_profiles']['planned']['phases'],[])
        self.assertEqual(result['route_profiles']['planned']['points'],[])
        self.assertTrue(result['route_profiles']['fixed']['phases'])

    def test_cli_preserves_radar1_stdin_and_validates_optional_speed(self):
        serve.plan_radar_offline({}) # Build the production CLI through its normal consumer.
        executable=ROOT.parents[2]/'.embeddedskills/tests/radar_plan_cli.exe'
        request='RADAR1 0 0\n2170 230 180 100 40000 0 3600 3 0 255\n'
        for args,speed in (([],200),(['--speed','37.5'],37.5)):
            result=subprocess.run([str(executable),*args],input=request,capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            output=json.loads(result.stdout);self.assertIn('route_profiles',output)
            self.assertEqual(output['route_profiles']['fixed']['speed_mm_s'],speed)
        for args in (['--speed','NaN'],['--speed','9'],['--speed','5001'],['--speed'],['--unknown']):
            result=subprocess.run([str(executable),*args],input=request,capture_output=True,text=True)
            self.assertEqual(result.returncode,2)
            self.assertIn('error',json.loads(result.stdout))

    def test_offline_rebuild_tracks_profile_source_and_header(self):
        # Changing the shared C core must invalidate a cached host executable.
        workspace=ROOT.parents[2]
        files=('Code/tools/radar_plan_cli.c','Code/template/App/radar_map.c','Code/template/App/radar_map.h',
               'Code/template/App/chassis_route_profile.c','Code/template/App/chassis_route_profile.h',
               'Code/template/App/chassis_config.h','Code/template/App/motor_id_config.h')
        with tempfile.TemporaryDirectory() as directory:
            clone=Path(directory)
            for name in files:
                destination=clone/name;destination.parent.mkdir(parents=True,exist_ok=True)
                shutil.copy2(workspace/name,destination)
            with patch.object(serve,'__file__',str(clone/'Code/upper-computer-web/local-car/serve.py')):
                serve.plan_radar_offline({})
                executable=clone/'.embeddedskills/tests/radar_plan_cli.exe'
                for name in ('Code/template/App/chassis_route_profile.c','Code/template/App/chassis_route_profile.h'):
                    path=clone/name;stamp=executable.stat().st_mtime+2
                    os.utime(path,(stamp,stamp))
                    with patch.object(serve.subprocess,'run',wraps=subprocess.run) as run:
                        serve.plan_radar_offline({})
                    compile_calls=[call for call in run.call_args_list if '-std=c99' in call.args[0]]
                    self.assertEqual(len(compile_calls),1,name)
                    self.assertIn(str(clone/'Code/template/App/chassis_route_profile.c'),compile_calls[0].args[0])

    def test_cloud_transport_does_not_fill_terminal_jsonl_history(self):
        with tempfile.TemporaryDirectory() as directory:
            bridge=serial_bridge.SerialBridge(directory)
            try:
                line=b'@RADAR {"v":1,"k":"cloud","points":[[900,1000,50]]}\r\n'
                bridge._handle_rx(line)
                self.assertTrue(any(e['kind']=='radar' for e in bridge.events))
                self.assertEqual(Path(bridge.rx_file.name).read_bytes(),line)
                self.assertNotIn('RADAR',Path(bridge.event_file.name).read_text(encoding='utf-8'))
                self.assertNotIn(line.hex(),Path(bridge.event_file.name).read_text(encoding='utf-8'))
            finally:
                bridge.close()

    def test_http_offline_endpoint_uses_shared_c_without_serial_commands(self):
        with tempfile.TemporaryDirectory() as directory:
            bridge=serial_bridge.SerialBridge(directory)
            server=serve.make_server(bridge,ROOT,0)
            thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
            try:
                request=Request(server.console_origin+'/api/radar/plan',data=json.dumps({'mask':0,'params':{},'points':[],'speed_mm_s':37.5}).encode(),headers={'Content-Type':'application/json','Origin':server.console_origin,'X-Console-Token':bridge.auth_token})
                with urlopen(request) as response:
                    result=json.load(response)
                self.assertEqual(result['algorithm'],'radar_map_c_v1')
                self.assertTrue(result['valid'])
                self.assertEqual(result['points'][0][:2],[2250,150])
                self.assertEqual(result['route_profiles']['planned']['speed_mm_s'],37.5)
                self.assertFalse(bridge.connected)
                self.assertEqual(len(bridge.commands),0)
                self.assertFalse(any(event['kind']=='tx' for event in bridge.events))
            finally:
                server.shutdown();server.server_close();thread.join(1);bridge.close()

if __name__=='__main__':
    unittest.main()
