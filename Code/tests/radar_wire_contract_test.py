"""Read real C-formatted radar pages through the production web snapshot model."""
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[2]
EXE = ROOT / ".embeddedskills/tests/radar_console_test.exe"
MODEL = ROOT / "Code/upper-computer-web/local-car/radar-model.mjs"
result = subprocess.run([str(EXE)], capture_output=True, text=True, encoding="utf-8", check=True)
lines = [line for line in result.stdout.splitlines() if line.startswith("@RADAR ")]
assert lines and all(len(line.encode("utf-8")) + 2 <= 256 for line in lines)
script = f"""
import assert from 'node:assert/strict';
import {{RadarModel}} from {json.dumps(MODEL.as_uri())};
let input='';for await(const part of process.stdin)input+=part;
const model=new RadarModel();model.connection(true);
for(const line of JSON.parse(input))assert.equal(model.accept(line,0),true);
assert.equal(model.snapshots.map.mask,328000);
assert.equal(model.snapshots.map.counts.length,25);
assert.deepEqual(model.snapshots.map.pose,[2170,230,180]);
assert.equal(model.snapshots.path.valid,true);
assert.deepEqual(model.snapshots.path.points.filter(p=>p[2]).map(p=>p.slice(2)),
  [[1,0],[5,1],[4,2],[2,3],[3,4],[4,5],[2,6],[3,7],[1,8]]);
assert.equal(model.snapshots.points.points.length,360);
assert.deepEqual(model.snapshots.points.points.at(-1),[3590,65535,255]);
assert.deepEqual(model.snapshots.points.pose,[2170,230,180]);
assert.equal(model.paramsLive,true);
assert.equal(Object.keys(model.params).length,10);
assert.equal(model.params.threshold,3);
console.log('radar_wire_contract_test: PASS (real C pages -> web model, map/path/cloud/params)');
"""
subprocess.run(["node", "--input-type=module", "-e", script], input=json.dumps(lines),
               text=True, encoding="utf-8", check=True, cwd=ROOT)
