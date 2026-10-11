"""Read real C-formatted radar pages through the production web snapshot model."""
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[2]
EXE = ROOT / ".embeddedskills/tests/radar_console_test.exe"
MODEL = ROOT / "Code/upper-computer-web/local-car/radar-model.mjs"
result = subprocess.run([str(EXE)], capture_output=True, text=True, encoding="utf-8", check=True)
legacy, stable = result.stdout.split("NEW_STABLE_BEGIN", 1)
lines = [line for line in legacy.splitlines() if line.startswith("@RADAR ")]
stable_lines = [line for line in stable.splitlines() if line.startswith("@RADAR ")]
assert lines and all(len(line.encode("utf-8")) + 2 <= 256 for line in lines)
assert stable_lines and all(len(line.encode("utf-8")) + 2 <= 256 for line in stable_lines)
script = f"""
import assert from 'node:assert/strict';
import {{RadarModel}} from {json.dumps(MODEL.as_uri())};
let input='';for await(const part of process.stdin)input+=part;
const model=new RadarModel();model.connection(true);
const streams=JSON.parse(input);
for(const line of streams.legacy){{const f=JSON.parse(line.slice(7));if(!['live','evidence'].includes(f.k))assert.equal(model.accept(line,0),true,line);}}
assert.equal(model.snapshots.map.mask,328000);
assert.equal(model.snapshots.map.counts.length,25);
assert.deepEqual(model.snapshots.map.pose,[2250,150,180]);
assert.equal(model.snapshots.path.valid,true);
assert.deepEqual(model.snapshots.path.points.filter(p=>p[2]).map(p=>p.slice(2)),
  [[1,0],[5,1],[4,2],[2,3],[3,4],[4,5],[2,6],[3,7],[1,8]]);
assert.equal(model.snapshots.points.points.length,360);
assert.deepEqual(model.snapshots.points.points.at(-1),[3590,65535,255]);
assert.deepEqual(model.snapshots.points.pose,[2250,150,180]);
assert.equal(model.paramsLive,true);
assert.equal(Object.keys(model.params).length,10);
assert.equal(model.params.threshold,1);
const stableModel=new RadarModel();stableModel.connection(true);
for(const line of streams.stable)assert.equal(stableModel.accept(line,0),true,line);
assert.equal(stableModel.snapshots.map.g,4);
assert.deepEqual(stableModel.snapshots.map.geometry,[65809,680,-407,66623,149407997,20569846,1800]);
assert.deepEqual(stableModel.snapshots.map.window,[1,15,15]);
assert.equal(stableModel.snapshots.stable.points.length,256);
assert.equal(stableModel.snapshots.stable.points[0][2],175);
assert.equal(stableModel.params.tx_q16,149407997);
console.log('radar_wire_contract_test: PASS (real C pages -> web model, map/path/cloud/params)');
"""
subprocess.run(["node", "--input-type=module", "-e", script], input=json.dumps({"legacy":lines,"stable":stable_lines}),
               text=True, encoding="utf-8", check=True, cwd=ROOT)
