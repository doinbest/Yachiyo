import fs from 'node:fs';
import assert from 'node:assert/strict';
import {footprintDiagnostic} from '../upper-computer-web/local-car/route-model.mjs';
const lines=fs.readFileSync(new URL('../../.embeddedskills/tests/curve-plant.csv',import.meta.url),'utf8').trim().split('\n');
for(const line of lines){
  const [kind,time,x,y,yaw]=line.split(',').map(Number);
  const issue=footprintDiagnostic({x_mm:x,y_mm:y,yaw_rad:yaw*Math.PI/180});
  assert.equal(issue,null,JSON.stringify({kind,time,x,y,yaw,issue}));
}
console.log(`curve plant: ${lines.length} full-body footprint samples PASS`);
