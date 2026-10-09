// Reach study of the mobile arm for a source pallet on the ASRS lane (support plane 0.610).
import { ARM, PALLET, DECK, DEG } from '../../mobman/js/config.js';
import * as A from '../../mobman/js/arm.js';
import * as B from '../../mobman/js/base.js';
import { palletBoxes, deckSlots } from '../../mobman/js/recipes.js';
import { SKUS, PORT } from '../js/config.js';
const QREF = [0, -0.6, 1.5, 0.6, Math.PI / 2, 0];
const pose = { x: 0, y: 0, th: 0 };
let n = 0, bad = 0, minE = 9, minS = 9, zLo = 9, zHi = 0;
const probe = (p, yaw) => {
  n++;
  let ok = false;
  for (const y of [yaw, yaw + Math.PI]) {
    const q = A.ik(B.worldToArm(pose, p), A.toolDown(B.yawToArm(pose, y)), QREF, ARM.branch);
    if (q) { ok = true; const s = A.singularity(q); minE = Math.min(minE, Math.abs(s.elbow)); minS = Math.min(minS, Math.abs(s.shoulder)); break; }
  }
  if (!ok) { bad++; if (bad < 10) console.log('unreachable', p.map((v) => v.toFixed(3))); }
  zLo = Math.min(zLo, p[2]); zHi = Math.max(zHi, p[2]);
};
for (const S of SKUS) {
  const rc = { ...S.rc, layers: S.srcLayers };
  for (const dx of [-0.015, 0.015]) for (const dy of [-0.015, 0.015]) for (const b of palletBoxes(rc)) {
    const top = PORT.z + b.z + rc.h / 2, p = [b.x + dx, 1.25 + b.y + dy];
    for (const raise of [0, 0.12]) { probe([...p, top], b.yaw); probe([...p, top + rc.h + 0.08 + raise], b.yaw); probe([...p, Math.max(top, DECK.z + rc.h) + 0.12 + raise], b.yaw); }
  }
  for (const s of deckSlots(rc)) for (const zz of [DECK.z + rc.h, PORT.z + PALLET.h + S.srcLayers * rc.h + rc.h + 0.08 + 0.12]) probe([s.x, s.y, zz], s.yaw);
}
console.log(`${n} probes, ${bad} unreachable, TCP z ${zLo.toFixed(3)} … ${zHi.toFixed(3)}, min |sin q3| ${minE.toFixed(3)}, min shoulder ${minS.toFixed(3)} m`);
