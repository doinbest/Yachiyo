"""Execute the main-loop optional-display gate with each real blocking condition."""
from pathlib import Path
import re
import subprocess

root = Path(__file__).resolve().parents[1]
source = (root/'template/Core/Src/main.c').read_text(encoding='utf-8-sig')
match = re.search(r'static bool OptionalDisplayReady\(void\)\s*\{.*?\n\}', source, re.S)
assert match, 'Optional display scheduling must have a single tested gate'
assert 'if (OptionalDisplayReady()) OledUi_Process();' in source
names = ['HWT101_Cal_IsBusy', 'ChassisMotion_IsBusy', 'ChassisRoute_IsBusy',
         'GrabTask_IsBusy', 'GrabRoute_IsBusy', 'MechanicalArm_IsBusy',
         'ArmVision_IsBusy', 'MaterialVision_IsBusy', 'Mecanum_IsBusy',
         'ChassisMotion_StopPending', 'ArmConsole_OperationBusy']
fixture = '''#include <assert.h>
#include <stdbool.h>
static unsigned blocked;
typedef struct { bool active; } MotorBus_Diagnostic_t;
typedef struct { bool active; } MotorBus_Recovery_t;
void MotorBus_DiagnosticGet(MotorBus_Diagnostic_t *s){s->active=blocked&(1U<<11);}
void MotorBus_RecoveryGet(MotorBus_Recovery_t *s){s->active=blocked&(1U<<12);}
bool RadarConsole_ScanBusy(void){return blocked&(1U<<13);}
bool Turntable_IsBusy(void){return blocked&(1U<<14);}
'''
fixture += ''.join(f'bool {name}(void){{return blocked&(1U<<{i});}}\n' for i,name in enumerate(names))
fixture += match.group() + '''
int main(void){assert(OptionalDisplayReady());
for(unsigned i=0;i<15;i++){blocked=1U<<i;assert(!OptionalDisplayReady());}
blocked=0;assert(OptionalDisplayReady());return 0;}
'''
out = root.parent/'.embeddedskills/tests'
test = out/'display_schedule_test.c'
test.write_text(fixture, encoding='utf-8')
exe = out/'display_schedule_test.exe'
subprocess.run(['gcc','-std=c99','-Wall','-Wextra','-Werror',str(test),'-o',str(exe)],check=True)
subprocess.run([str(exe)],check=True)
print('display_schedule: all 15 control/transaction/capture conditions defer optional OLED I/O PASS')
