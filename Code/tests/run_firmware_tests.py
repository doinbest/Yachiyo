"""Run firmware host regressions; products stay under ignored .embeddedskills/tests."""
from pathlib import Path
import os
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT.parent / ".embeddedskills" / "tests"
OUT.mkdir(parents=True, exist_ok=True)
BASE = ["gcc", "-std=c99", "-Wall", "-Wextra", "-Werror",
        "-Itests/usb_stubs", "-Itemplate/Hardware", "-Itemplate/App"]


def run(args):
    env = dict(os.environ, PYTHONIOENCODING="utf-8")
    result = subprocess.run([str(arg) for arg in args], cwd=ROOT, text=True, encoding="utf-8", env=env,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    print(result.stdout, end="", flush=True)
    with (OUT / "host-tests.log").open("a", encoding="utf-8") as log:
        log.write("$ " + " ".join(map(str, args)) + "\n" + result.stdout)
    if result.returncode:
        raise SystemExit(result.returncode)


def build_test(name, *sources):
    exe = OUT / (name + ".exe")
    if ("template/App/contest_mission.c" in sources or name in ("contest_choreography_test", "contest_heading_test")) and "template/App/station_heading.c" not in sources and "tests/mission_heading_stub.c" not in sources:
        sources = (*sources, "tests/mission_heading_stub.c")
    if any(source in sources for source in ("template/App/radar_map.c", "template/App/radar_scan.c")):
        sources = (*sources, "template/App/radar_geometry.c", "template/App/radar_stability.c")
    flags = [BASE[0], "-Itests/radar_stubs", *BASE[1:]] if name in ("test_radar_scan", "test_radar_probe", "test_radar_host", "radar_host_console_test") else BASE
    if "tests/mission_heading_stub.c" in sources:
        if name == "mission_grab_integration_test":
            flags = [*flags, "-DMISSION_TEST_HAS_GRAB_TASK"]
        if "template/App/chassis_motion.c" in sources or name == "contest_heading_test":
            flags = [*flags, "-DMISSION_TEST_HAS_MOTION"]
    if name in ("qr_control_test", "grab_pose_test", "grab_task_test", "grab_calibration_task_test", "grab_transfer_test", "grab_grace_test", "grab_command_sim_test", "grab_tuning_sim_test", "grab_track_task_test", "grab_track_pick_test", "grab_stack_test"):
        sources = (*sources, "template/App/grab_tracking.c", "template/App/chassis_observer.c", "template/App/chassis_model.c")
    if "template/App/mecanum_chassis.c" in sources and "template/App/chassis_model.c" not in sources:
        sources = (*sources, "template/App/chassis_model.c")
    # Existing virtual UART fixtures freeze time at ACK; the dedicated settle
    # test exercises the production 2 ms handover and emergency bypass.
    if name not in ("motor_bus_reply_settle_test", "motor_bus_probe_test", "chassis_feedback_alignment_test"):
        flags = [*flags, "-DMOTOR_BUS_REPLY_SETTLE_MS=0"]
    run(flags + ["tests/" + name + ".c", *sources, "-lm", "-o", exe])
    run([exe])


if __name__ == "__main__":
    run([sys.executable, "tools/sync_grab_parameters.py", "--check"])
    if (OUT / "host-tests.log").exists():
        from datetime import datetime
        import shutil
        shutil.copy2(OUT / "host-tests.log", OUT / ("host-tests-" + datetime.now().strftime("%Y%m%d-%H%M%S-%f") + ".log"))
    (OUT / "host-tests.log").write_text("", encoding="utf-8")
    build_test("camera_usb_test", "template/Hardware/Camera.c")
    build_test("station_heading_test", "template/App/station_heading.c")
    build_test("grab_heading_source_test", "template/App/grab_tracking.c", "template/App/chassis_observer.c", "template/App/chassis_model.c")
    build_test("chassis_heading_test", "template/App/chassis_motion.c", "template/App/chassis_localization.c", "template/App/chassis_observer.c", "template/App/chassis_model.c", "template/App/chassis_heading.c")
    build_test("contest_heading_test", "template/Hardware/QR.c")
    build_test("qr_test", "template/Hardware/QR.c")
    build_test("contest_screen_test", "template/App/contest_screen.c")
    build_test("hwt101_i2c_test", "template/Hardware/hwt101_i2c.c")
    build_test("hwt101_calibration_test", "template/App/hwt101_calibration.c")
    build_test("hwt101_recovery_integration_test", "template/Hardware/hwt101_i2c.c", "template/App/hwt101_calibration.c")
    build_test("hwt101_read_recovery_test", "template/Hardware/hwt101_i2c.c", "template/App/hwt101_calibration.c")
    build_test("console_rx_test", "template/App/console_rx.c")
    build_test("console_priority_test", "template/App/console_tx.c")
    build_test("console_tx_test", "template/App/console_tx.c")
    build_test("console_tx_pacing_test", "template/App/console_tx.c")
    build_test("chassis_model_test", "template/App/chassis_model.c")
    build_test("chassis_motion_test", "template/App/chassis_motion.c", "template/App/chassis_model.c")
    build_test("chassis_heading_pd_test", "template/App/chassis_motion.c", "template/App/chassis_model.c")
    build_test("chassis_heading_deadband_test", "template/App/chassis_motion.c", "template/App/chassis_model.c")
    build_test("chassis_stop_test", "template/App/chassis_motion.c", "template/App/chassis_model.c")
    build_test("chassis_control_period_test", "template/App/chassis_motion.c", "template/App/chassis_model.c")
    build_test("chassis_test_tuning_test", "template/App/chassis_motion.c", "template/App/chassis_model.c")
    build_test("chassis_asymmetric_accel_test", "template/App/chassis_motion.c", "template/App/chassis_model.c")
    build_test("chassis_acceleration_test", "template/App/chassis_motion.c", "template/App/chassis_model.c")
    build_test("chassis_distance_test", "template/App/chassis_motion.c", "template/App/chassis_model.c")
    build_test("chassis_cosine_distance_test", "template/App/chassis_motion.c", "template/App/chassis_model.c")
    build_test("chassis_cosine_continuity_test", "template/App/chassis_motion.c", "template/App/chassis_model.c")
    build_test("chassis_curve_motion_test", "template/App/chassis_motion.c", "template/App/chassis_model.c")
    run(["node", "tests/check_curve_footprint.mjs"])
    build_test("chassis_curve_profile_test", "template/App/chassis_route_profile.c")
    build_test("chassis_turn_test", "template/App/chassis_motion.c", "template/App/chassis_model.c")
    build_test("chassis_route_profile_test", "template/App/chassis_route_profile.c")
    build_test("chassis_task_profile_test", "template/App/chassis_route_profile.c")
    build_test("chassis_task_route_test", "template/App/chassis_route.c", "template/App/chassis_route_profile.c")
    build_test("chassis_route_continuity_test", "template/App/chassis_route.c", "template/App/chassis_route_profile.c")
    build_test("chassis_route_test", "template/App/chassis_route.c", "template/App/chassis_route_profile.c")
    build_test("grab_route_test", "template/App/GrabRoute.c", "template/Hardware/QR.c")
    build_test("mission_navigation_test", "template/App/mission_navigation.c")
    build_test("contest_mission_test", "template/App/contest_mission.c", "template/Hardware/QR.c")
    build_test("contest_choreography_test", "template/Hardware/QR.c")
    build_test("contest_mission_scan_test", "template/App/contest_mission.c", "template/Hardware/QR.c")
    build_test("contest_mission_prepare_test", "template/App/contest_mission.c", "template/Hardware/QR.c")
    build_test("contest_mission_cancel_test", "template/App/contest_mission.c", "template/Hardware/QR.c")
    build_test("contest_mission_recovery_test", "template/App/contest_mission.c", "template/Hardware/QR.c")
    build_test("mission_stop_feedback_test", "template/App/contest_mission.c", "template/App/chassis_motion.c", "template/App/chassis_model.c", "template/Hardware/QR.c")
    build_test("chassis_route_pause_feedback_test", "template/App/chassis_route.c", "template/App/chassis_route_profile.c", "template/App/chassis_motion.c", "template/App/chassis_model.c")
    build_test("mission_grab_integration_test", "template/App/contest_mission.c", "template/App/grab_tracking.c", "template/App/chassis_observer.c", "template/App/chassis_model.c")
    build_test("grab_mission_pause_test", "template/App/grab_tracking.c", "template/App/chassis_observer.c", "template/App/chassis_model.c")
    build_test("grab_choreography_test", "template/App/grab_tracking.c", "template/App/chassis_observer.c", "template/App/chassis_model.c")
    build_test("grab_bus_recovery_test", "template/App/grab_tracking.c", "template/App/chassis_observer.c", "template/App/chassis_model.c")
    build_test("grab_pose_test")
    build_test("grab_time_saving_test", "template/App/grab_tracking.c", "template/App/chassis_observer.c", "template/App/chassis_model.c")
    build_test("grab_axis_acceleration_test", "template/App/grab_tracking.c", "template/App/chassis_observer.c", "template/App/chassis_model.c")
    build_test("grab_arrival_fallback_test", "template/App/grab_tracking.c", "template/App/chassis_observer.c", "template/App/chassis_model.c")
    build_test("test_radar_parser", "template/Hardware/lds_parser.c")
    build_test("test_radar_stability", "template/App/radar_geometry.c", "template/App/radar_stability.c")
    build_test("test_radar_map", "template/App/radar_map.c")
    build_test("test_radar_g4", "template/App/radar_map.c", "template/App/radar_start.c")
    build_test("test_radar_point_map", "template/App/radar_map.c", "template/App/radar_host_map.c")
    build_test("test_radar_host_map", "template/App/radar_host_map.c", "template/App/radar_map.c")
    build_test("test_radar_host", "template/App/radar_host.c", "template/App/radar_host_map.c", "template/App/radar_map.c")
    build_test("radar_host_console_test", "template/App/radar_host_console.c")
    build_test("test_radar_start", "template/App/radar_start.c", "template/App/radar_map.c")
    build_test("test_radar_scan", "template/Hardware/lds_parser.c", "template/Hardware/radar_uart.c",
               "template/App/radar_scan.c", "template/App/radar_map.c", "template/App/radar_start.c", "template/App/radar_host_map.c")
    build_test("test_radar_probe", "template/Hardware/lds_parser.c", "template/Hardware/radar_uart.c")
    build_test("radar_console_test", "template/App/radar_console.c", "template/App/radar_map.c")
    build_test("qr_control_test")
    build_test("grab_task_test")
    build_test("grab_calibration_task_test", "template/App/grab_calibration.c")
    build_test("grab_stack_test")
    build_test("grab_tracking_test", "template/App/grab_tracking.c")
    build_test("grab_track_task_test")
    build_test("grab_track_pick_test")
    build_test("grab_track_store_test", "template/App/grab_tracking.c", "template/App/chassis_observer.c", "template/App/chassis_model.c")
    build_test("grab_car_pick_test", "template/App/grab_tracking.c", "template/App/chassis_observer.c", "template/App/chassis_model.c")
    build_test("grab_held_place_test", "template/App/grab_tracking.c", "template/App/chassis_observer.c", "template/App/chassis_model.c")
    build_test("grab_processed_store_test", "template/App/grab_tracking.c", "template/App/chassis_observer.c", "template/App/chassis_model.c")
    build_test("grab_processing_continuity_test", "template/App/grab_tracking.c", "template/App/chassis_observer.c", "template/App/chassis_model.c")
    build_test("grab_station_align_height_test", "template/App/grab_tracking.c", "template/App/chassis_observer.c", "template/App/chassis_model.c")
    build_test("grab_transfer_test")
    build_test("grab_grace_test")
    build_test("grab_command_sim_test", "template/App/console_tx.c")
    build_test("grab_tuning_sim_test")
    build_test("steer_route_guard_test")
    build_test("lidar_servo_test", "-Itemplate/Core/Inc")
    build_test("chassis_localization_test", "template/App/chassis_localization.c", "template/App/chassis_observer.c", "template/App/chassis_model.c")
    build_test("chassis_observer_test", "template/App/chassis_observer.c", "template/App/chassis_model.c")
    build_test("chassis_observer_alignment_test", "template/App/chassis_observer.c", "template/App/chassis_model.c")
    build_test("chassis_feedback_alignment_test", "template/App/mecanum_chassis.c", "template/Hardware/motor_bus.c", "template/App/chassis_localization.c", "template/App/chassis_observer.c", "template/App/chassis_model.c")
    for delay, profile in ((8, 0), (2, 1), (2, 2)):
        run([OUT / "chassis_feedback_alignment_test.exe", str(delay), str(profile)])
    build_test("chassis_telemetry_test", "template/App/chassis_telemetry.c", "template/App/chassis_localization.c", "template/App/chassis_observer.c", "template/App/chassis_model.c")
    build_test("motor_bus_stop_wire_test", "template/Hardware/motor_bus.c")
    build_test("motor_bus_ack_timing_test", "template/Hardware/motor_bus.c")
    build_test("motor_bus_reply_settle_test")
    build_test("motor_bus_ack_fairness_test", "template/Hardware/motor_bus.c")
    build_test("motor_bus_test", "template/Hardware/motor_bus.c")
    build_test("motor_arrival_test", "template/Hardware/motor_bus.c")
    build_test("motor_bus_probe_test", "template/Hardware/motor_bus.c")
    build_test("turntable_test", "template/App/turntable.c", "template/Hardware/motor_bus.c")
    build_test("turntable_bus_fairness_test", "template/App/turntable.c", "template/Hardware/motor_bus.c")
    build_test("motor_recovery_test", "template/Hardware/motor_bus.c")
    build_test("motor_bus_diagnostic_test", "template/Hardware/motor_bus.c")
    build_test("motor_bus_startup_test", "template/App/mechanical_arm.c", "template/Hardware/Emm_V5.c", "template/Hardware/motor_bus.c", "-Wno-old-style-declaration")
    build_test("motor_bus_lazy_config_test", "template/App/mechanical_arm.c", "template/App/turntable.c", "template/Hardware/Emm_V5.c", "template/Hardware/motor_bus.c", "-Wno-old-style-declaration")
    run([OUT / "motor_bus_lazy_config_test.exe", "turntable_home"])
    for scenario in ("restart_fail", "stop_fail", "dma_fail", "overflow", "active_error", "active_ne", "active_ore", "active_missing"):
        run([OUT / "motor_bus_startup_test.exe", scenario])
    build_test("mecanum_bus_test", "template/App/mecanum_chassis.c", "template/Hardware/motor_bus.c")
    build_test("mecanum_feedback_test", "template/App/mecanum_chassis.c", "template/Hardware/motor_bus.c", "template/App/chassis_localization.c", "template/App/chassis_observer.c", "template/App/chassis_model.c")
    build_test("mecanum_tracking_bus_test", "template/App/mecanum_chassis.c", "template/Hardware/motor_bus.c", "template/App/chassis_localization.c", "template/App/chassis_observer.c", "template/App/chassis_model.c")
    run([OUT / "mecanum_tracking_bus_test.exe", "2"])
    run([OUT / "mecanum_tracking_bus_test.exe", "2", "explicit80"])
    build_test("mechanical_arm_bus_test", "template/App/mechanical_arm.c", "template/Hardware/Emm_V5.c", "template/Hardware/motor_bus.c", "-Wno-old-style-declaration")
    build_test("mechanical_arm_emergency_chain_test", "template/App/mechanical_arm.c", "template/Hardware/Emm_V5.c", "template/Hardware/motor_bus.c", "-Wno-old-style-declaration")
    run([OUT / "mechanical_arm_emergency_chain_test.exe", "first_tx_error"])
    for axis in range(3):
        run([OUT / "mechanical_arm_emergency_chain_test.exe", f"single_{axis}"])
    build_test("motor_cancel_readback_test", "template/Hardware/motor_bus.c")
    build_test("motor_bus_circular_rx_test", "template/App/mechanical_arm.c", "template/Hardware/Emm_V5.c", "template/Hardware/motor_bus.c", "-Wno-old-style-declaration")
    build_test("mecanum_heading_test", "-Itemplate/System", "template/App/chassis_motion.c", "template/App/chassis_model.c")
    build_test("w25q128_test", "template/Hardware/w25q128.c")
    build_test("grab_calibration_test")
    build_test("key_test", "template/Hardware/key.c")
    build_test("arm_console_host_test")
    # UI supplies synthetic snapshots; retain the production visual-state classifier.
    camera_object = OUT / "camera_ui.o"
    run(BASE + ["-DCamera_SnapshotGet=Camera_TestDriverSnapshotGet", "-c",
                "template/Hardware/Camera.c", "-o", camera_object])
    run([sys.executable, "tests/display_schedule_test.py"])
    build_test("oled_ui_test", "template/App/oled_ui.c", str(camera_object))
    for name in ["grab_camera_protocol_test.py", "tracking_report_test.py", "hwt101_upload_test.py", "w25q128_startup_test.py", "arm_console_test.py", "usb_camera_test.py", "usb_cdc_tx_test.py", "chassis_map_contract_test.py", "radar_capture_unpack_test.py", "radar_host_wire_contract_test.py"]:
        run([sys.executable, ROOT / "tests" / name])
    print("Firmware host regression suite: PASS")
