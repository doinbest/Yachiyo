"""Run only the standalone F103 turntable tests; write logs under .embeddedskills."""
from pathlib import Path
import subprocess
ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT.parents[1] / '.embeddedskills' / 'turntable'
OUT.mkdir(parents=True, exist_ok=True)
sources = [ROOT/'tests/turntable_test.c', *sorted((ROOT/'App').glob('*.c')),
           *sorted((ROOT/'Hardware').glob('*.c'))]
cmd = ['gcc', '-std=c99', '-Wall', '-Wextra', '-Werror',
       '-I'+str(ROOT/'tests/stubs'), '-I'+str(ROOT/'App'), '-I'+str(ROOT/'Hardware'),
       *map(str, sources), '-o', str(OUT/'turntable_test.exe')]
with (OUT/'host-tests.log').open('w', encoding='utf-8') as log:
    boot_cmd = ['gcc', '-std=c99', '-Wall', '-Wextra', '-Werror',
                '-I'+str(ROOT/'tests/stubs'), '-I'+str(ROOT/'App'),
                str(ROOT/'tests/boot_test.c'), str(ROOT/'App/turntable_boot.c'),
                '-o', str(OUT/'boot_test.exe')]
    for args in [cmd, [str(OUT/'turntable_test.exe')], boot_cmd, [str(OUT/'boot_test.exe')]]:
        result = subprocess.run(args, capture_output=True, text=True, encoding="utf-8", errors="replace")
        print(result.stdout + result.stderr, end='')
        log.write(result.stdout + result.stderr)
        if result.returncode: raise SystemExit(result.returncode)

