"""Receive mechanisms: real bus/configuration, arrival evidence and choreography."""
import ctypes
import json
import os
from run_firmware_tests import build_test, run, OUT

GRAB=("template/App/grab_tracking.c","template/App/chassis_observer.c","template/App/chassis_model.c")
BUS=("template/Hardware/motor_bus.c",)
ARM=(*BUS,"template/Hardware/Emm_V5.c","template/App/mechanical_arm.c","-Wno-old-style-declaration")
TESTS=[
    ("mechanism_receive_test",BUS),
    ("grab_receive_arrival_test",GRAB),
    ("motor_arrival_test",BUS),
    ("motor_bus_lazy_config_test",(*ARM,"template/App/turntable.c")),
    ("mechanical_arm_bus_test",ARM),
    ("grab_feedback_schedule_test",GRAB),
    ("grab_arrival_fallback_test",GRAB),
    ("grab_car_pick_test",GRAB),
    ("grab_first_tray_clearance_test",GRAB),
    ("grab_alignment_handoff_test",GRAB),
    ("grab_bus_recovery_test",GRAB),
    ("grab_recovery_stream_test",GRAB),
    ("grab_choreography_test",GRAB),
    ("grab_raw_handoff_test",GRAB),
    ("grab_mission_pause_test",GRAB),
    ("grab_z_unified_test",GRAB),
    ("grab_none_profile_test",GRAB),
    ("grab_pose_test",()),
    ("grab_track_pick_test",()),
    ("grab_track_task_test",()),
    ("grab_static_frame_test",()),
    ("turntable_test",(*BUS,"template/App/turntable.c")),
    ("turntable_feedback_schedule_test",()),
    ("turntable_bus_fairness_test",(*BUS,"template/App/turntable.c")),
    ("motor_bus_reply_settle_test",()),
    ("chassis_none_bus_test",(*BUS,"template/App/mecanum_chassis.c")),
]
if __name__=="__main__":
    if os.name=="nt":ctypes.windll.kernel32.SetErrorMode(0x8003)
    results=[]
    for name,sources in TESTS:
        try:
            build_test(name,*sources)
            results.append({"test":name,"status":"pass"})
        except SystemExit as error:
            results.append({"test":name,"status":"fail","exit":error.code})
    for name,arg in [("motor_bus_lazy_config_test",c) for c in ("tracking","tracking_stop","turntable_home")]+[("chassis_none_bus_test","receive")]:
        try:
            run([OUT/(name+".exe"),arg]);results.append({"test":name+":"+arg,"status":"pass"})
        except SystemExit as error:
            results.append({"test":name+":"+arg,"status":"fail","exit":error.code})
    (OUT/"mechanism-receive-results.json").write_text(json.dumps(results,indent=2),encoding="utf-8")
    failed=[row for row in results if row["status"]!="pass"]
    print(f"mechanism Receive suite: {len(results)-len(failed)}/{len(results)} PASS")
    if failed:print(json.dumps(failed,indent=2))
    raise SystemExit(bool(failed))
