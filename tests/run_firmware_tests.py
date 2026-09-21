"""Run firmware host regressions; products stay under ignored .embeddedskills/tests."""
from pathlib import Path
import os
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / ".embeddedskills" / "tests"
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
    run(BASE + ["tests/" + name + ".c", *sources, "-lm", "-o", exe])
    run([exe])


if __name__ == "__main__":
    (OUT / "host-tests.log").write_text("", encoding="utf-8")
    build_test("camera_usb_test", "template/Hardware/Camera.c")
    build_test("qr_test", "template/Hardware/QR.c")
    build_test("hwt101_i2c_test", "template/Hardware/hwt101_i2c.c")
    build_test("hwt101_calibration_test", "template/App/hwt101_calibration.c")
    build_test("console_tx_test", "template/App/console_tx.c")
    build_test("console_tx_pacing_test", "template/App/console_tx.c")
    build_test("chassis_model_test", "template/App/chassis_model.c")
    build_test("chassis_motion_test", "template/App/chassis_motion.c", "template/App/chassis_model.c")
    build_test("chassis_stop_test", "template/App/chassis_motion.c", "template/App/chassis_model.c")
    build_test("chassis_distance_test", "template/App/chassis_motion.c", "template/App/chassis_model.c")
    build_test("chassis_route_test", "template/App/chassis_route.c")
    build_test("steer_route_guard_test")
    build_test("chassis_localization_test", "template/App/chassis_localization.c", "template/App/chassis_observer.c", "template/App/chassis_model.c")
    build_test("chassis_observer_test", "template/App/chassis_observer.c", "template/App/chassis_model.c")
    build_test("chassis_telemetry_test", "template/App/chassis_telemetry.c", "template/App/chassis_localization.c", "template/App/chassis_observer.c", "template/App/chassis_model.c")
    build_test("motor_bus_test", "template/Hardware/motor_bus.c")
    build_test("mecanum_bus_test", "template/App/mecanum_chassis.c", "template/Hardware/motor_bus.c")
    build_test("mecanum_feedback_test", "template/App/mecanum_chassis.c", "template/Hardware/motor_bus.c", "template/App/chassis_localization.c", "template/App/chassis_observer.c", "template/App/chassis_model.c")
    build_test("mechanical_arm_bus_test", "template/App/mechanical_arm.c", "template/Hardware/Emm_V5.c", "template/Hardware/motor_bus.c", "-Wno-old-style-declaration")
    build_test("motor_bus_circular_rx_test", "template/App/mechanical_arm.c", "template/Hardware/Emm_V5.c", "template/Hardware/motor_bus.c", "-Wno-old-style-declaration")
    build_test("mecanum_heading_test", "-Itemplate/System")
    build_test("w25q128_test", "template/Hardware/w25q128.c")
    build_test("material_vision_test")
    build_test("arm_vision_test")
    build_test("key_test", "template/Hardware/key.c")
    build_test("arm_console_host_test")
    # UI supplies synthetic snapshots; retain the production visual-state classifier.
    camera_object = OUT / "camera_ui.o"
    run(BASE + ["-DCamera_SnapshotGet=Camera_TestDriverSnapshotGet", "-c",
                "template/Hardware/Camera.c", "-o", camera_object])
    build_test("oled_ui_test", "template/App/oled_ui.c", str(camera_object))
    for name in ["hwt101_upload_test.py", "w25q128_startup_test.py", "arm_console_test.py", "usb_camera_test.py", "usb_cdc_tx_test.py", "chassis_map_contract_test.py"]:
        run([sys.executable, ROOT / "tests" / name])
    print("Firmware host regression suite: PASS")
