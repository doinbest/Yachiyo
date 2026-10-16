import json
import math
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
    def test_formal_linear_speed_caps_match_firmware_and_keep_cosine(self):
        profiles=serve.plan_radar_offline({'mask':0,'speed_mm_s':1000})['route_profiles']
        task=profiles['task_fixed']['phases']
        self.assertEqual([p['speed_mm_s'] for p in task[:3]],[300,0,1000])
        raw=[p for p in task if p['segment']==3 and p['kind']=='move']
        self.assertEqual(raw[0]['speed_mm_s'],700)
        rough=[p for p in task if p['segment']==4 and p['kind']=='move']
        self.assertEqual(rough[0]['speed_mm_s'],1000)
        self.assertEqual(task[-1]['speed_mm_s'],300)
        cosine=profiles['task_fixed_cosine']['phases']
        self.assertEqual(cosine[0]['phase'],'entry_curve')
        self.assertEqual(cosine[-1]['speed_mm_s'],1000)

    def test_fifteen_circle_shared_core_and_rolling_expiry(self):
        circles = [[[0, 1000, 80]] for _ in range(3)] + [[] for _ in range(12)]
        result = serve.plan_radar_offline({'circles': circles})
        self.assertEqual(result['algorithm'], 'radar_stable_c_v2')
        self.assertEqual(result['grid_version'], 4)
        self.assertEqual(result['raw_points'], 3)
        self.assertEqual(len(result['stable_points']), 1)
        self.assertEqual(result['stable_points'][0][2], 3)
        self.assertEqual(result['geometry']['origin_q16'], [149407997, 20569846])
        # The same fixed neighborhood loses support as circles leave the window.
        expired = serve.plan_radar_offline({'circles': circles + [[] for _ in range(3)]})
        self.assertFalse(expired['stable_points'])
        self.assertEqual(expired['raw_points'], 0)
        self.assertEqual(expired['window'], {'first': 4, 'last': 18, 'circles': 15})

    def test_one_dense_circle_is_not_three_distinct_circles(self):
        result = serve.plan_radar_offline({'circles': [[[0, 1000, 80]] * 100] + [[] for _ in range(14)]})
        self.assertFalse(result['stable_points'])
        self.assertEqual(result['raw_points'], 100)
        with self.assertRaisesRegex(ValueError, '至少15'):
            serve.plan_radar_offline({'circles': [[]] * 14})

    def test_default_keeps_single_return_obstacle_like_firmware(self):
        result=serve.plan_radar_offline({'points':[[400,1500,30]]})
        self.assertEqual(result['counts'][12],1)
        self.assertEqual(result['mask'],0x50140|(1<<12))

    def test_partial_cloud_cannot_replace_map_but_existing_mask_can_still_plan(self):
        with patch.object(serve.subprocess, 'run', return_value=subprocess.CompletedProcess([],0,stdout='{}',stderr='')) as run:
            with self.assertRaisesRegex(ValueError, '完整点云'):
                serve.plan_radar_offline({'cloud_complete':False,'points':[[100,800,80]]})
            run.assert_not_called()
        result=serve.plan_radar_offline({'cloud_complete':False,'mask':0x50140|(1<<13)})
        self.assertEqual(result['mask'],0x50140|(1<<13))

    def test_corner_centers_and_full_circle_share_firmware_defaults(self):
        lower=serve.plan_radar_offline({'start_mode':'lower'})
        upper=serve.plan_radar_offline({'start_mode':'upper'})
        self.assertEqual(lower['pose'],[2250,150,180])
        self.assertEqual(upper['pose'],[2250,2250,270])
        # Use a central sensor to distinguish the angle filter from field clipping.
        accepted=serve.plan_radar_offline({'params':{'lidar_x_mm':1200,'lidar_y_mm':1200},
                                         'points':[[1800,500,80],[1801,500,80]]})
        self.assertEqual(sum(accepted['counts']),2)
        self.assertEqual(accepted['counts'][13],2)
        rejected=serve.plan_radar_offline({'params':{'lidar_x_mm':1200,'lidar_y_mm':1200,'angle_max_tenths':900},
                                         'points':[[1800,500,80],[1801,500,80]]})
        self.assertEqual(sum(rejected['counts']),0)

    def test_full_recorded_circle_counts_all_filtered_inset_points_without_background_deletion(self):
        fixture=ROOT.parents[2]/'Code/tests/fixtures/radar-field-20261004.json'
        source=json.loads(fixture.read_text(encoding='utf-8'))
        result=serve.plan_radar_offline({'points':source['points'],'start_mode':'lower'})
        # Independent double-precision projection and literal inset grid edges.
        expected=[0]*25
        edges=[150,550,1000,1400,1850,2250]
        for angle,distance,energy in source['points']:
            if not 100<=distance<=40000 or not 0<=energy<=255:
                continue
            radians=math.radians(180-angle/10)
            x,y=2250+distance*math.cos(radians),150+distance*math.sin(radians)
            if not (150<x<=2250 and 150<=y<2250):
                continue
            col=next(i for i in range(5) if x<=edges[i+1])
            row=next(i for i in range(5) if y<edges[i+1])
            expected[row*5+col]+=1
        self.assertEqual(result['counts'],expected)
        mask=0x50140|sum(1<<i for i,n in enumerate(expected) if n and i not in (2,4,10,14,22))
        self.assertEqual(result['mask'],mask)

    def test_field_boundary_uses_projected_endpoint_not_side_length_as_range_limit(self):
        def sample(x,y):
            dx,dy=x-2250,y-150
            return [round(((180-math.degrees(math.atan2(dy,dx)))%360)*10)%3600,
                    round(math.hypot(dx,dy)),70]
        diagonal=sample(350,2000)
        self.assertGreater(diagonal[1],2400)
        result=serve.plan_radar_offline({'points':[diagonal,diagonal]})
        self.assertEqual(result['counts'][20],2)
        outside=sample(-350,2000)
        result=serve.plan_radar_offline({'points':[outside,outside]})
        self.assertEqual(sum(result['counts']),0)

    def test_independent_long_surface_inside_field_remains_an_obstacle(self):
        points=[]
        for x in range(1050,1751,50):
            dx,dy=x-2250,1300-150
            points.append([round(((180-math.degrees(math.atan2(dy,dx)))%360)*10)%3600,
                           round(math.hypot(dx,dy)),70])
        result=serve.plan_radar_offline({'points':sorted(points)})
        self.assertEqual(sum(result['counts']),len(points))
        self.assertEqual(result['mask']&((1<<12)|(1<<13)),(1<<12)|(1<<13))

    def corner_evidence(self, start):
        # Synthetic clutter in another station, inside the real CW quadrant.
        # It conflicts with QR/raw only in the wrong corner hypothesis; the
        # selected corner is identifiable but its mission need not be reachable.
        x,y,zero=(2250,150,180) if start=='lower' else (2250,2250,270)
        tx,ty=(1200,350) if start=='lower' else (350,1200)
        angle=round(((zero-math.degrees(math.atan2(ty-y,tx-x)))%360)*10)%3600
        distance=round(math.hypot(tx-x,ty-y))
        return [[angle,distance,50]]*3

    def test_dual_start_offline_classifies_before_clearing_and_keeps_driving_fixed(self):
        for name,selected,y in (('upper',1,2250),('lower',2,150)):
            with self.subTest(start=name):
                result=serve.plan_radar_offline({'start_mode':'auto','points':self.corner_evidence(name)})
                self.assertEqual(result['start']['selected'],selected)
                self.assertEqual(result['start']['reason'],'clear_regions')
                self.assertTrue(result['valid']) # F1 task-cell protection retains evidence but clears scan occupancy.
                empty=serve.plan_radar_offline({'start_mode':name})
                self.assertEqual(empty['points'][0][:2],[2250,y])
                self.assertEqual(empty['points'][-1][:2],[2250,y])
                self.assertEqual(result['mask'] & ((1<<14)|(1<<22)),0)
                self.assertEqual(result['route_profiles']['fixed']['points'][0][:2],[2250,150])
                self.assertEqual(result['plan_only'],name=='upper')
                if name=='upper':
                    self.assertFalse(result['route_profiles']['planned']['phases'])

    def test_unresolved_dual_start_never_silently_builds_lower_path(self):
        result=serve.plan_radar_offline({'start_mode':'auto','points':[]})
        self.assertEqual(result['start']['selected'],0)
        self.assertEqual(result['start']['reason'],'ambiguous')
        self.assertFalse(result['valid'])
        self.assertFalse(result['points'])
        self.assertFalse(result['route_profiles']['planned']['phases'])
        selected=serve.plan_radar_offline({'start_mode':'upper','points':[]})
        self.assertEqual(selected['start']['reason'],'manual_selection')
        self.assertEqual(selected['points'][0][:2],[2250,2250])
        with self.assertRaises(ValueError):
            serve.plan_radar_offline({'start_mode':'invalid'})

    def test_manual_selection_keeps_raw_conflicts_but_clears_only_task_front_cells(self):
        points=sorted(self.corner_evidence('upper')+self.corner_evidence('lower'))
        unresolved=serve.plan_radar_offline({'start_mode':'auto','points':points})
        self.assertEqual(unresolved['start']['reason'],'both_conflict')
        self.assertFalse(unresolved['valid'])
        selected=serve.plan_radar_offline({'start_mode':'lower','points':points})
        self.assertEqual(selected['start']['conflicts'],[1<<14,1<<22])
        self.assertEqual(selected['start']['qr'],[3,0])
        self.assertTrue(selected['valid'])
        self.assertEqual(selected['mask'] & ((1<<14)|(1<<22)),0)
        self.assertEqual(selected['counts'][22],3)
        override=serve.plan_radar_offline({'start_mode':'lower','points':points,'mask':1<<14})
        self.assertFalse(override['valid']) # An explicit offline obstacle remains authoritative.

    def test_radar_reads_and_scoped_cancel_classification(self):
        for command in ('radar status','radar get','radar fetch cloud 7 33','radar nav status'):
            self.assertIsNotNone(serial_bridge.READ_COMMAND.fullmatch(command),command)
        for command in ('radar stop','radar nav cancel'):
            self.assertIsNotNone(serial_bridge.PRIORITY_STOP.fullmatch(command),command)
        self.assertIsNone(serial_bridge.READ_COMMAND.fullmatch('radar task start 100'))

    def test_f407_host_transport_is_read_only_only_for_queries(self):
        for command in ('radar host status','radar host get active','radar host get draft',
                        'radar host live status','radar host fetch config:1 0','radar host fetch cloud:7 33'):
            self.assertIsNotNone(serial_bridge.READ_COMMAND.fullmatch(command),command)
        for command in ('radar host stop','radar host live stop'):
            self.assertIsNotNone(serial_bridge.PRIORITY_STOP.fullmatch(command),command)
        for command in ('radar host start','radar host scan','radar host live start',
                        'radar host draft reset','radar host set threshold 1','radar host apply 1'):
            self.assertIsNone(serial_bridge.READ_COMMAND.fullmatch(command),command)

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
        self.assertEqual(fixed['turn_speed_deg_s'],100)
        self.assertEqual(planned['turn_speed_deg_s'],100)
        self.assertEqual(len(fixed['points']),16)
        self.assertEqual(fixed['points'][0],[2250,150,1,0])
        self.assertEqual(fixed['points'][-2:],[ [2100,300,0,0], [2250,150,1,8] ])
        self.assertEqual(planned['points'],result['points'])
        self.assertEqual(planned['points'][:3],[[2250,150,1,0],[2100,300,0,0],[2100,1200,5,1]])
        first=planned['phases'][0]
        self.assertEqual(first,dict(kind='move',phase='move',direction='diagonal',segment=1,
                                   station=0,visit=0,target=[2100,300,180],speed_mm_s=200))
        for profile in (fixed,planned):
            self.assertEqual([p['target'] for p in profile['phases'][:3]],[[2100,300,180],[2100,300,90],[2100,1200,90]])
            self.assertEqual([p['direction'] for p in profile['phases'][:3]],['diagonal','turn','forward'])
        self.assertTrue(all(p['segment']==1 and p['station']==5 and p['visit']==1 for p in fixed['phases'][:3]))
        self.assertEqual([(p['segment'],p['station'],p['visit']) for p in planned['phases'][:3]],[(1,0,0),(1,0,0),(2,5,1)])
        docking=[p for p in fixed['phases'] if p['segment']==6]
        self.assertEqual([p['phase'] for p in docking],['turn','dock_approach','arrival_turn','dock'])
        self.assertEqual([p['target'] for p in docking],[[1200,1200,180],[425,1200,180],[425,1200,90],[350,1200,90]])
        self.assertEqual([p['speed_mm_s'] for p in docking],[0,200,0,50])
        leaving=[p for p in fixed['phases'] if p['segment']==7]
        self.assertEqual([p['phase'] for p in leaving],['undock','turn','move'])
        self.assertEqual([p['target'] for p in leaving],[[425,1200,90],[425,1200,180],[1200,1200,180]])
        keys={'kind','phase','direction','segment','station','visit','target','speed_mm_s'}
        for profile in (fixed,planned):
            for phase in profile['phases']:
                self.assertEqual(set(phase),keys)
        self.assertEqual(planned['phases'][-1]['target'],[2250,150,90])

    def test_rear_arm_task_profiles_use_shared_work_headings(self):
        profiles=serve.plan_radar_offline({'mask':0})['route_profiles']
        task=profiles['task_fixed']
        self.assertEqual([p['target'] for p in task['phases'][:3]],
                         [[2100,300,180],[2100,300,90],[2100,1200,90]])
        self.assertEqual([p['kind'] for p in task['phases'][:3]],['move','turn','move'])
        cosine=profiles['task_fixed_cosine']
        self.assertEqual(cosine['phases'][0]['phase'],'entry_curve')
        self.assertEqual(cosine['phases'][0]['path'][8][:3],[2125,350,180])
        self.assertEqual(cosine['phases'][1:],task['phases'][3:])
        self.assertTrue(profiles['task_planned_cosine']['phases'])
        raw=[p for p in task['phases'] if p['segment']==3]
        self.assertEqual([p['phase'] for p in raw],['work_approach','work_turn'])
        self.assertEqual(raw[0]['target'],[1200,2100,90])
        self.assertEqual(raw[0]['direction'],'lateral')
        self.assertEqual(raw[0]['arm_pose'],'rear')
        self.assertEqual(raw[-1]['target'],[1200,2100,0])
        self.assertEqual(raw[-1]['direction'],'turn')
        self.assertTrue(all('path' not in p for p in raw))
        for station,yaw in [(2,180),(3,90)]:
            docks=[p for p in task['phases'] if p['station']==station and p['phase']=='work_dock']
            self.assertEqual(len(docks),2)
            if station==2:self.assertEqual([p['target'] for p in docks],[[1200,350,180]]*2)
            self.assertTrue(all(p['target'][2]==yaw and p['speed_mm_s']==50 for p in docks))
        processing=[p for p in task['phases'] if p['station']==2 and p['phase']=='work_turn']
        self.assertEqual([p['target'] for p in processing],[[1200,380,180]]*2)
        for name in ('task_fixed','task_planned','task_fixed_cosine','task_planned_cosine'):
            phases=profiles[name]['phases']
            docks=[i for i,p in enumerate(phases) if p['station']==2 and p['phase']=='work_dock']
            self.assertEqual(len(docks),2)
            for i in docks:
                self.assertEqual(phases[i]['target'],[1200,350,180])
                self.assertEqual(phases[i+1]['phase'],'undock')
                self.assertEqual(phases[i+1]['target'],[1200,425,180])
                self.assertEqual(phases[i+1]['speed_mm_s'],50)
        temporary=[p for p in task['phases'] if p['segment']==6]
        self.assertEqual([p['target'] for p in temporary],[[1200,1200,0],[425,1200,0],[425,1200,90],[350,1200,90]])
        self.assertEqual(temporary[1]['direction'],'reverse')
        self.assertEqual(task['phases'][-1]['target'],[2250,150,90])
        self.assertEqual(task['phases'][-1]['arm_pose'],'rear')
        self.assertTrue(profiles['task_planned']['phases'])

    def test_planned_detour_keeps_first_scan_heading_until_qr_visit(self):
        result=serve.plan_radar_offline({'mask':1<<9})
        self.assertTrue(result['valid'])
        self.assertIn('route_profiles',result)
        profile=result['route_profiles']['planned'];qr_index=next(
            i for i,point in enumerate(profile['points']) if point[2:]==[5,1])
        self.assertGreater(qr_index,1)
        first_leg=[p for p in profile['phases'] if p['segment']<=qr_index]
        self.assertTrue(first_leg)
        self.assertEqual(first_leg[0]['kind'],'move')
        self.assertEqual(first_leg[0]['target'][2],180)
        self.assertTrue(all(p['target'][2]==90 for p in first_leg[1:]))
        self.assertEqual(first_leg[-1]['station'],5)
        self.assertEqual(first_leg[-1]['visit'],1)

    def test_optional_speed_changes_main_phases_and_keeps_docking_cap(self):
        for speed in (10,37.5,5000):
            result=serve.plan_radar_offline({'speed_mm_s':speed})
            self.assertIn('route_profiles',result)
            for name,profile in result['route_profiles'].items():
                self.assertEqual(profile['speed_mm_s'],speed)
                for phase in profile['phases']:
                    expected=0 if phase['kind']=='turn' else min(speed,50) if phase['phase'] in ('dock','undock','work_dock') else speed
                    if name in ('task_fixed','task_planned') and phase['kind']=='move':
                        self.assertGreater(phase['speed_mm_s'],0)
                        self.assertLessEqual(phase['speed_mm_s'],min(expected,1000))
                    else:
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
        # Direct CLI defaults follow firmware; the offline HTTP service passes
        # its independent 200 mm/s preview default explicitly via --speed.
        for args,speed in (([],1000),(['--speed','37.5'],37.5)):
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
               'Code/template/App/radar_geometry.c','Code/template/App/radar_geometry.h',
               'Code/template/App/radar_stability.c','Code/template/App/radar_stability.h',
               'Code/template/App/radar_start.c','Code/template/App/radar_start.h',
               'Code/template/App/chassis_route_profile.c','Code/template/App/chassis_route_profile.h','Code/template/App/chassis_path.h',
               'Code/template/App/chassis_config.h','Code/template/App/motor_id_config.h')
        with tempfile.TemporaryDirectory() as directory:
            clone=Path(directory)
            for name in files:
                destination=clone/name;destination.parent.mkdir(parents=True,exist_ok=True)
                shutil.copy2(workspace/name,destination)
            with patch.object(serve,'__file__',str(clone/'Code/upper-computer-web/local-car/serve.py')):
                serve.plan_radar_offline({})
                executable=clone/'.embeddedskills/tests/radar_plan_cli.exe'
                for name in ('Code/template/App/chassis_route_profile.c','Code/template/App/chassis_route_profile.h','Code/template/App/chassis_path.h',
                             'Code/template/App/radar_geometry.c','Code/template/App/radar_stability.h',
                             'Code/template/App/radar_start.c','Code/template/App/radar_start.h'):
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
