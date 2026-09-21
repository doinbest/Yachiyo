// UI intent is separate from TX (serial write) and RX (device reply).
export async function trackAttempt(wire, operation, changed) {
  changed({wire,state:'pending'});
  let written=false;
  try { written=await operation()===true; }
  catch { written=false; }
  changed({wire,state:written?'written':'failed'});
  return written;
}

export function createAttemptTracker(changed) {
  let revision=0;
  return {
    supersede() { revision++; },
    run(wire,operation) {
      const own=++revision;
      return trackAttempt(wire,operation,state=>changed({...state,current:own===revision}));
    }
  };
}
