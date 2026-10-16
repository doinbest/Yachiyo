"""Run the existing host regressions selected for the ID8 parallel-transfer plan."""
import ast
from pathlib import Path
import runpy

runner = Path(__file__).with_name("run_firmware_tests.py")
suite = runpy.run_path(str(runner))
selected = {
    "grab_task_test", "grab_parallel_transfer_test", "grab_first_tray_clearance_test",
    "grab_alignment_handoff_test", "grab_processing_continuity_test",
    "grab_held_place_test", "grab_station_align_height_test", "grab_car_pick_test",
    "grab_processed_store_test", "grab_track_pick_test", "grab_track_store_test",
    "grab_raw_handoff_test", "grab_transfer_test", "grab_pose_test", "grab_grace_test",
    "turntable_test", "turntable_bus_fairness_test", "contest_choreography_test",
    "contest_mission_test", "mission_grab_integration_test", "mission_stop_feedback_test",
    "contest_imu_pause_test",
}
done = set()
for call in ast.walk(ast.parse(runner.read_text(encoding="utf-8"))):
    if (isinstance(call, ast.Call) and isinstance(call.func, ast.Name)
            and call.func.id == "build_test" and call.args
            and isinstance(call.args[0], ast.Constant) and call.args[0].value in selected):
        arguments = [ast.literal_eval(value) for value in call.args]
        print("CHECK", arguments[0], flush=True)
        suite["build_test"](*arguments)
        done.add(arguments[0])
assert done == selected, "Selected tests missing from run_firmware_tests.py: " + str(selected - done)
print(f"ID8 parallel transfer: {len(done)} host test groups PASS", flush=True)
