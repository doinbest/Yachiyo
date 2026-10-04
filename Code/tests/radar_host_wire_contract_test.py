"""Compile the real C host serializer and consume every emitted page in real JS."""
from pathlib import Path
import json
import os
import subprocess


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT.parent / ".embeddedskills" / "tests"
OUT.mkdir(parents=True, exist_ok=True)
EXE = OUT / "radar_host_wire_fixture.exe"
MODEL = ROOT / "upper-computer-web" / "local-car" / "radar-host-model.mjs"


def run(command, *, input_text=None):
    result = subprocess.run(command, cwd=ROOT, input=input_text, text=True, encoding="utf-8",
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            env={**os.environ, "PYTHONIOENCODING": "utf-8"})
    if result.returncode:
        raise AssertionError("command failed:\n" + result.stdout)
    return result.stdout


compile_output = run([
    "gcc", "-std=c99", "-Wall", "-Wextra", "-Wno-unused-function", "-DRADAR_HOST_WIRE_FIXTURE",
    "-Itests/radar_stubs", "-Itests/usb_stubs", "-Itemplate/Hardware", "-Itemplate/App",
    "tests/radar_host_console_test.c", "template/App/radar_host_console.c",
    "template/App/radar_host_map.c", "template/App/radar_map.c", "-lm", "-o", str(EXE),
])
assert not compile_output.strip(), compile_output
wire = run([str(EXE)])
lines = [line for line in wire.splitlines() if line]
assert lines and all(line.startswith("@RADAR ") for line in lines)
assert all(len((line + "\r\n").encode("ascii")) <= 256 for line in lines)

consumer = r"""
import fs from 'node:fs';
import {pathToFileURL} from 'node:url';
const {RadarHostModel}=await import(pathToFileURL(process.argv[1]));
const model=new RadarHostModel();model.connection(true);
for(const line of fs.readFileSync(0,'utf8').split(/\r?\n/))if(line)model.accept(line);
const s=model.snapshots;
console.log(JSON.stringify({
  config:s.config&&{threshold:s.config.threshold,protected_mask:s.config.protected_mask,radar_x:s.config.radar_x,radar_y:s.config.radar_y},
  map:s.map&&{mask:s.map.mask,counts:s.map.counts},
  path:s.path&&{count:s.path.points.length,first:s.path.points[0],last:s.path.points.at(-1)},
  cloud:s.cloud&&{crc32:s.cloud.crc32,points:s.cloud.points},
  errors:model.lastError,
}));
"""
summary = json.loads(run(["node", "--input-type=module", "-e", consumer, str(MODEL)], input_text=wire))
assert summary["errors"] is None, summary
assert summary["config"] == {
    "threshold": 1, "protected_mask": 0x00404405, "radar_x": 230, "radar_y": 230,
}
assert summary["map"]["mask"] == 0x00050140
assert summary["map"]["counts"] == [0] * 25
assert summary["path"]["count"] == 33
assert summary["path"]["first"][:2] == [230, 230]
assert summary["path"]["last"][:2] == [230, 230]
assert summary["cloud"]["points"] == [
    {"angle_deg": 0, "distance_mm": 100, "energy": 7},
    {"angle_deg": 90.1, "distance_mm": 65535, "energy": 255},
    {"angle_deg": 359.9, "distance_mm": 230, "energy": 1},
]
print(f"Radar host C/JS wire contract: PASS ({len(lines)} complete pages, max {max(len((line + chr(13) + chr(10)).encode('ascii')) for line in lines)} bytes)")
