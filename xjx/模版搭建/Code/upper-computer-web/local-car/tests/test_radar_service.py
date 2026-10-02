import json
from pathlib import Path
import sys
import unittest
import tempfile
import threading
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
                request=Request(server.console_origin+'/api/radar/plan',data=json.dumps({'mask':0,'params':{},'points':[]}).encode(),headers={'Content-Type':'application/json','Origin':server.console_origin,'X-Console-Token':bridge.auth_token})
                with urlopen(request) as response:
                    result=json.load(response)
                self.assertEqual(result['algorithm'],'radar_map_c_v1')
                self.assertTrue(result['valid'])
                self.assertEqual(result['points'][0][:2],[2250,150])
                self.assertFalse(bridge.connected)
                self.assertEqual(len(bridge.commands),0)
                self.assertFalse(any(event['kind']=='tx' for event in bridge.events))
            finally:
                server.shutdown();server.server_close();thread.join(1);bridge.close()

if __name__=='__main__':
    unittest.main()
