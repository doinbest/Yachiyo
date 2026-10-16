"""Focused feedback scheduling regressions; runs the actual task/bus sources."""
from pathlib import Path
import json
from run_firmware_tests import build_test

GRAB=("template/App/grab_tracking.c", "template/App/chassis_observer.c", "template/App/chassis_model.c")
TESTS=[
    ("grab_feedback_schedule_test",GRAB),
    ("turntable_feedback_schedule_test",()),
    ("grab_arrival_fallback_test",GRAB),
    ("grab_pose_test",()),
    ("grab_transfer_test",()),
    ("grab_car_pick_test",GRAB),
    ("grab_first_tray_clearance_test",GRAB),
    ("grab_static_frame_test",()),
    ("grab_alignment_handoff_test",GRAB),
    ("grab_raw_handoff_test",GRAB),
    ("grab_track_pick_test",()),
    ("grab_mission_pause_test",GRAB),
    ("grab_bus_recovery_test",GRAB),
    ("grab_z_unified_test",GRAB),
    ("turntable_test",("template/App/turntable.c","template/Hardware/motor_bus.c")),
    ("turntable_bus_fairness_test",("template/App/turntable.c","template/Hardware/motor_bus.c")),
    ("motor_bus_reply_settle_test",()),
    ("motor_arrival_test",("template/Hardware/motor_bus.c",)),
]

if __name__=="__main__":
    results=[]
    for name,sources in TESTS:
        print("RUN "+name,flush=True)
        try:
            build_test(name,*sources)
            results.append({"test":name,"status":"pass"})
        except SystemExit as error:
            results.append({"test":name,"status":"fail","exit":error.code})
    target=Path(__file__).resolve().parents[2]/".embeddedskills"/"tests"/"feedback-schedule-results.json"
    target.write_text(json.dumps(results,indent=2),encoding="utf-8")
    print(json.dumps(results,indent=2))
    raise SystemExit(any(row["status"]!="pass" for row in results))
