"""Compile the real C host serializer and consume every emitted page in real JS."""
from pathlib import Path
import json
import os
import subprocess
import zlib


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
    "template/App/radar_host_map.c", "template/App/radar_map.c", "template/App/radar_geometry.c", "-lm", "-o", str(EXE),
])
assert not compile_output.strip(), compile_output
wire = run([str(EXE)])
lines = [line for line in wire.splitlines() if line]
assert lines and all(line.startswith("@RADAR ") for line in lines)
assert all(len((line + "\r\n").encode("ascii")) <= 256 for line in lines)
for line in lines:
    frame = json.loads(line[7:])
    if frame["k"] in ("host_map", "host_path"):
        checksum = frame.pop("crc32", None)
        protected = json.dumps(frame, separators=(",", ":")).encode("ascii")
        assert checksum == f'{zlib.crc32(protected):08X}', frame
    if frame["k"] in ("r", "e"):
        checksum = frame.pop("c")
        assert checksum == f'{zlib.crc32(json.dumps(frame, separators=(",", ":")).encode("ascii")):08X}', frame

consumer = r"""
import fs from 'node:fs';
import {pathToFileURL} from 'node:url';
const {RadarHostModel}=await import(pathToFileURL(process.argv[1]));
const {RadarHostClient}=await import(new URL('radar-host-client.mjs',pathToFileURL(process.argv[1])));
const model=new RadarHostModel();model.connection(true);
const errors=[],previews=[],lines=fs.readFileSync(0,'utf8').split(/\r?\n/).filter(Boolean);let legacyPath=null;
for(const line of lines){
  model.accept(line);if(model.lastError)errors.push(model.lastError);
  if(model.lastEvent.key==='preview')previews.push(model.lastEvent.snapshot);
  if(model.lastEvent.key==='path')legacyPath=model.lastEvent.snapshot.points;
  if(model.lastEvent.key==='result'&&JSON.stringify(model.snapshots.path.points)!==JSON.stringify(legacyPath))throw new Error('Compact waypoints differ from original C text result');
}
const s=model.snapshots;
const stream=lines.filter(line=>['host_map','host_path','host_result_end'].includes(JSON.parse(line.slice(7)).k));
const marker=JSON.parse(stream.at(-1).slice(7));
const missing=['host_map:1',`host_path:${marker.pp-1}`],commands=[];
const repaired=new RadarHostModel();repaired.connection(true);let client;
client=new RadarHostClient({model:repaired,send:async command=>{
  commands.push(command);
  if(command==='radar host result 4')queueMicrotask(()=>{for(const line of stream){
    const f=JSON.parse(line.slice(7));if(!missing.includes(`${f.k}:${f.page}`))client.receive(line);
  }});
  const match=/^radar host fetch (map|path):\d+ (\d+)$/.exec(command);
  if(match){const line=stream.find(line=>{const f=JSON.parse(line.slice(7));return f.k===`host_${match[1]}`&&f.page===Number(match[2]);});
    if(!line)throw new Error('Requested a nonexistent C page');queueMicrotask(()=>client.receive(line));}
  return {};
}});
client.receive('@RADAR '+JSON.stringify({...marker,k:'host_status',ss:4,a:0,l:1,ls:2,reason:'complete'}));
for(let i=0;i<30&&!commands.includes('radar host ack 4');i++)await new Promise(setImmediate);
if(!commands.includes('radar host ack 4')||client.lastError)throw new Error('Real C stream not acknowledged: '+JSON.stringify({commands,error:client.lastError}));
const compact=lines.filter(line=>['r','e'].includes(JSON.parse(line.slice(7)).k)),compactCommands=[];
const configStart=lines.findIndex(line=>{const f=JSON.parse(line.slice(7));return f.k==='host_config'&&f.data==='source,capture';});
const configPages=JSON.parse(lines[configStart].slice(7)).pages;
const captureConfig=lines.slice(configStart,configStart+configPages);
const last=JSON.parse(compact[0].slice(7)).n-1;let fast;
fast=new RadarHostClient({fastResults:true,send:async command=>{
  compactCommands.push(command);
  const match=/^radar host bundle 4 (.+)$/.exec(command);
  if(match)queueMicrotask(()=>{for(const line of compact){const f=JSON.parse(line.slice(7));if(f.k==='e'||match[1].split(',').includes(String(f.p)))fast.receive(line);}});
  if(/^radar host fetch stable:2 0$/.test(command))queueMicrotask(()=>fast.receive(lines.find(line=>JSON.parse(line.slice(7)).k==='host_stable')));
  const cfg=/^radar host fetch config:1 (\d+)$/.exec(command);
  if(cfg)queueMicrotask(()=>fast.receive(captureConfig.find(line=>JSON.parse(line.slice(7)).page===Number(cfg[1]))));
  return {};
}});
for(const line of compact){const f=JSON.parse(line.slice(7));if(f.p!==0&&f.p!==last)fast.receive(line);}
for(let i=0;i<30&&!compactCommands.includes('radar host ack 4');i++)await new Promise(setImmediate);
if(JSON.stringify(fast.model.snapshots.path?.points)!==JSON.stringify(legacyPath)||fast.model.snapshots.map.receivedAt!==fast.model.snapshots.path.receivedAt)throw new Error('Real C compact pair not published together');
const expected=[`radar host bundle 4 0,${last}`,...captureConfig.map(line=>`radar host fetch config:1 ${JSON.parse(line.slice(7)).page}`),'radar host fetch stable:2 0','radar host ack 4'];
if(JSON.stringify(compactCommands)!==JSON.stringify(expected))throw new Error('Real C compact repair not batched: '+JSON.stringify(compactCommands));
console.log(JSON.stringify({
  config:s.config&&{threshold:s.config.threshold,protected_mask:s.config.protected_mask,radar_x:s.config.radar_x,radar_y:s.config.radar_y},
  map:s.map&&{mask:s.map.mask,counts:s.map.counts},
  path:s.path&&{count:s.path.points.length,first:s.path.points[0],last:s.path.points.at(-1)},
  cloud:s.cloud&&{crc32:s.cloud.crc32,points:s.cloud.points},
  previews:previews.map(p=>({id:p.live_id,count:p.points.length,first:p.points[0],last:p.points.at(-1)})),
  stream:{mp:marker.mp,pp:marker.pp,commands,repairedPoints:repaired.snapshots.path.points.length},
  compact:{pages:JSON.parse(compact[0].slice(7)).n,bytes:compact.reduce((sum,line)=>sum+line.length+2,0),commands:compactCommands},
  errors,
}));
"""
summary = json.loads(run(["node", "--input-type=module", "-e", consumer, str(MODEL)], input_text=wire))
assert summary["errors"] == [], summary
assert summary["config"] == {
    "threshold": 1, "protected_mask": 0x00404414, "radar_x": 2280, "radar_y": 314,
}
assert summary["map"]["mask"] == 0x00050140
assert summary["map"]["counts"] == [0] * 25
assert summary["path"]["count"] == 33
assert summary["path"]["first"][:2] == [2050, 350]
assert summary["path"]["last"][:2] == [2050, 350]
assert summary["stream"]["commands"] == [
    "radar host result 4", "radar host fetch map:3 1",
    f'radar host fetch path:4 {summary["stream"]["pp"]-1}', "radar host ack 4",
]
assert summary["stream"]["repairedPoints"] == 33
assert summary["compact"]["pages"] == 7
assert summary["compact"]["bytes"] < 1800
assert summary["cloud"]["points"] == [
    {"angle_deg": 0, "distance_mm": 100, "energy": 7},
    {"angle_deg": 90.1, "distance_mm": 65535, "energy": 255},
    {"angle_deg": 359.9, "distance_mm": 230, "energy": 1},
]
assert summary["previews"] == [
    {"id": 10, "count": 120, "first": {"angle_deg": 0, "distance_mm": 1000},
     "last": {"angle_deg": 119, "distance_mm": 1119}},
    {"id": 11, "count": 113, "first": {"angle_deg": 0, "distance_mm": 5000},
     "last": {"angle_deg": 112, "distance_mm": 5112}},
]
print(f"Radar host C/JS wire contract: PASS ({len(lines)} complete pages, max {max(len((line + chr(13) + chr(10)).encode('ascii')) for line in lines)} bytes)")
