// Headless verification of the turntable wrapper model: film geometry, wrap program, gripper
// geometry, load stability, the safety monitor and a long production run. `npm run verify:wrapper`

import { PALLET, SKUS, LINE, TABLE, CARRIAGE, FILM, WRAP, CLAMP, CUT, SAFE, DEG, RPM, G } from '../js/config.js';
import * as K from '../js/kinematics.js';
import { Cell, zoneX } from '../js/sim.js';

const { abs, max, min, hypot, PI, exp, ceil } = Math;
let fails = 0;
const check = (name, ok, detail = '') => {
  if (!ok) fails++;
  console.log(`${ok ? '  ok ' : ' FAIL'}  ${name}${detail ? '  — ' + detail : ''}`);
};
const f = (x, n = 3) => x.toFixed(n);
const e = (x) => x.toExponential(1);
const deg = (x, n = 1) => (x / DEG).toFixed(n) + '°';
const HX = PALLET.sx / 2, HY = PALLET.sy / 2, W = TABLE.wrap.v;
const OFFS = [[0, 0], [LINE.stopTol, LINE.guideTol], [-LINE.stopTol, LINE.guideTol], [LINE.stopTol, -LINE.guideTol], [-LINE.stopTol, -LINE.guideTol]];

// ---------- 1. film line and moment arm ----------
console.log('\n1. Film line, moment arm and film demand');
{
  let eT = 0, side = true;
  for (let i = 0; i < 2000; i++) {
    const a = (i / 2000) * 2 * PI, C = [0.75 * Math.cos(a), 0.75 * Math.sin(a)], l = K.filmLine(C), E = CARRIAGE.roller;
    eT = max(eT, abs(hypot(l.T[0] - E[0], l.T[1] - E[1]) - CARRIAGE.rExit), abs(K.cross(l.u, [C[0] - l.T[0], C[1] - l.T[1]])), abs(hypot(C[0] - l.T[0], C[1] - l.T[1]) - l.len));
    if (K.cross(l.u, [E[0] - l.T[0], E[1] - l.T[1]]) >= 0) side = false;
  }
  check('film line is tangent to the exit roller and passes through the contact point', eT < 1e-12, e(eT) + ' m');
  check('the film passes the exit roller on the load side', side);

  let eV = 0, eInt = 0, jump = 0, aR = [9, -9], pR = [9, 0], lR = [9, 0], handovers = [];
  for (const off of OFFS) {
    const film = new K.FilmWrap(off, K.Z_BOTTOM), n = 240000, d = (6 * PI) / n;
    let sum = 0, prev = film.line(0).arm, S0 = film.pathLength(0);
    for (let i = 1; i <= n; i++) {
      const th = i * d, ho = film.advance(th - d, K.Z_BOTTOM, th, K.Z_BOTTOM), l = film.line(th), S1 = film.pathLength(th);
      if (!ho) eV = max(eV, abs((S1 - S0) / d - (l.arm + prev) / 2)); // dS/dθ = p  →  film speed = ω·p
      jump = max(jump, abs(l.arm - prev));
      if (th > 2 * PI && th <= 4 * PI) sum += ((l.arm + prev) / 2) * d;
      aR = [min(aR[0], l.alpha), max(aR[1], l.alpha)];
      if (th > 2 * PI) { pR = [min(pR[0], l.arm), max(pR[1], l.arm)]; lR = [min(lR[0], l.len), max(lR[1], l.len)]; }
      prev = l.arm; S0 = S1;
    }
    eInt = max(eInt, abs(sum - K.PERIMETER));
    if (off[0] === 0) handovers = film.anchors.slice(1, 6).map((a) => a.th);
  }
  check('film speed = ω · p: d(path length)/dθ equals the moment arm', eV < 1e-6, e(eV) + ' m');
  check('∮ p dθ over one revolution = perimeter of the load', eInt < 1e-6, `${f(K.PERIMETER)} m, error ${e(eInt)}; mean arm ${f(K.ARM_MEAN, 4)} m`);
  check('moment arm is continuous at every corner hand-over', jump < 3e-4, `largest step between samples ${e(jump)} m`);
  check('film direction at the exit roller stays inside the window', aR[0] > CARRIAGE.exit[0] && aR[1] < CARRIAGE.exit[1], `${deg(aR[0])} … ${deg(aR[1])} of ${deg(CARRIAGE.exit[0], 0)} … ${deg(CARRIAGE.exit[1], 0)}`);
  check('moment arm never vanishes', pR[0] > 0.3, `${f(pR[0])} … ${f(pR[1])} m → film speed varies ${f(pR[1] / pR[0], 2)} : 1 at constant ω`);
  console.log(`      centred load: hand-overs at q1 = ${handovers.map((a) => deg(a)).join(', ')}; free film length ${f(lR[0])} … ${f(lR[1])} m`);
  // centred load: the arm is exactly a half side when the film lies on a face, and the corner radius at most
  const film = new K.FilmWrap([0, 0], K.Z_BOTTOM);
  let pmin = 9, pmax = 0;
  for (let i = 1; i <= 400000; i++) { const th = (i * 6 * PI) / 400000; film.advance(th - 1e-5, 0, th, 0); if (th > 2 * PI) { const p = film.line(th).arm; pmin = min(pmin, p); pmax = max(pmax, p); } }
  check('centred load: arm ranges from the half width to the corner radius', abs(pmin - HY) < 1e-5 && abs(pmax - hypot(HX, HY)) < 1e-5, `${f(pmin, 4)} … ${f(pmax, 4)} m`);
}

// ---------- 2. wrap program ----------
console.log('\n2. Wrap program');
{
  let ok = true, eEnd = 0, revs = [99, 0], Ts = [1e9, 0], margin = 1e9, slack = 1e9;
  for (let H = PALLET.h + 0.2; H <= PALLET.maxH + 1e-9; H += LINE.heightRes) {
    const P = K.wrapPlan(H);
    const turn = (await import('../../asrs/js/motion.js')).scurve(2 * PI * P.revs, W, TABLE.wrap.a, TABLE.wrap.j);
    eEnd = max(eEnd, abs(turn.endError));
    ok &&= Number.isInteger(P.revs) && P.tRelease > P.tSpin && P.t4 <= P.T - P.tSpin && P.top <= CARRIAGE.lift.max && P.bottom >= CARRIAGE.lift.min;
    margin = min(margin, P.T - P.tSpin - P.t4);
    slack = min(slack, (WRAP.clampDownBy - WRAP.releaseAt) / W - CLAMP.tJaw - CLAMP.tLift);
    revs = [min(revs[0], P.revs), max(revs[1], P.revs)]; Ts = [min(Ts[0], P.T), max(Ts[1], P.T)];
  }
  check('every load height: whole revolutions, all events at full speed, carriage inside its travel', ok, `${revs[0]} … ${revs[1]} revolutions, ${f(Ts[0], 1)} … ${f(Ts[1], 1)} s; carriage is down ≥ ${f(margin, 2)} s before braking`);
  check('turntable profile ends exactly on the home angle', eEnd < 1e-9, e(eEnd) + ' rad');
  check('clamp is open and down before the film reaches it', slack > 0, `${f(slack, 3)} s to spare`);
  const up = K.liftLimits(WRAP.overlapUp), dn = K.liftLimits(WRAP.overlapDown);
  check('carriage speeds give the programmed overlap', abs(1 - (up.v * 2 * PI) / W / FILM.w - WRAP.overlapUp) < 1e-12 && abs(1 - (dn.v * 2 * PI) / W / FILM.w - WRAP.overlapDown) < 1e-12, `up ${f(up.v, 4)} m/s = ${f(K.pitchOf(WRAP.overlapUp), 4)} m/rev, down ${f(dn.v, 4)} m/s = ${f(K.pitchOf(WRAP.overlapDown), 4)} m/rev`);
}

// ---------- 3. gripper and clearances ----------
console.log('\n3. Clamp, cut slide and clearances');
{
  const h = K.homeFilmLine();
  check('clamp stands on the film line of the home pose', abs(K.clampMiss()) < 0.001 && abs(h.alpha - CLAMP.yaw) < 0.1 * DEG, `${f(1000 * K.clampMiss(), 2)} mm off; film direction ${deg(h.alpha, 2)}`);
  let miss = 0;
  for (const off of OFFS) for (const th of [-TABLE.homeTol, 0, TABLE.homeTol]) miss = max(miss, abs(K.clampMiss(off, th)));
  check('film passes between the open jaws for every pallet position and home error', miss < CLAMP.open / 2, `worst ${f(1000 * miss, 1)} mm of ±${f(500 * CLAMP.open, 0)} mm`);
  const filmLow = K.Z_BOTTOM - FILM.w / 2, rClamp = hypot(CLAMP.x, CLAMP.y);
  check('clamp is below the film when down and grips it when up', CLAMP.zDown < filmLow - 0.01 && abs(CLAMP.zDown + CLAMP.stroke - filmLow - CLAMP.gripH) < 1e-9, `${f(1000 * (filmLow - CLAMP.zDown), 0)} mm below, ${f(1000 * CLAMP.gripH, 0)} mm of web gripped`);
  check('clamp is outside the load and inside the turntable', rClamp + 0.05 < TABLE.rSwept && -CLAMP.y - 0.05 > HY + LINE.guideTol, `radius ${f(rClamp)} m`);
  const capstan = exp(FILM.mu * WRAP.anchorCorners * PI / 2);
  check('tail holds by friction round three corners when the clamp opens', capstan > 2, `capstan ratio ${f(capstan, 2)} → tail tension ${f(FILM.F / capstan, 1)} N of ${FILM.F} N`);
  check('clamp holds the film', (2 * CLAMP.mu * CLAMP.force) / FILM.Fmax > 2, `SF ${f((2 * CLAMP.mu * CLAMP.force) / FILM.Fmax, 2)} at the tension limit`);
  const B = CARRIAGE.body, rBody = hypot(max(B.x0, min(0, B.x1)), max(B.y0, min(0, B.y1))), rLoad = max(...OFFS.map(K.sweptRadius));
  check('load swept radius inside its envelope', rLoad < SAFE.rLoad, `${f(rLoad)} of ${f(SAFE.rLoad)} m`);
  check('carriage clears the turntable and the load', rBody - TABLE.rSwept >= SAFE.clear && hypot(...CARRIAGE.roller) - CARRIAGE.rExit - rLoad >= 0.2, `${f(1000 * (rBody - TABLE.rSwept), 0)} mm to the table, ${f(1000 * (hypot(...CARRIAGE.roller) - CARRIAGE.rExit - rLoad), 0)} mm exit roller to the load`);
  check('line conveyors clear the turntable', LINE.xEnd - TABLE.rSwept >= 0.03 - 1e-9, `${f(1000 * (LINE.xEnd - TABLE.rSwept), 0)} mm`);
  const rI1 = hypot(-zoneX(2) - LINE.stopTol - HX, HY);
  check('pallet waiting at I1 is outside the swept circle', rI1 - TABLE.rSwept > 0.2, `${f(rI1)} m`);
  const xb = CUT.x - CUT.brushW / 2;
  check('parked cut slide is outside the swept circle and below every film line', hypot(xb, CUT.park) - TABLE.rSwept > SAFE.clear && CARRIAGE.roller[1] - CARRIAGE.rExit - CUT.park >= SAFE.clear, `${f(1000 * (CARRIAGE.roller[1] - CARRIAGE.rExit - CUT.park), 0)} mm under the lowest film line`);
  check('cut slide passes between the clamp and the load corner, clear of the carriage', CUT.wireX - (CLAMP.x + 0.03) >= 0.03 && CUT.x + CUT.brushW / 2 < HX - LINE.stopTol && CUT.wireX - 0.01 - B.x1 >= SAFE.clear, `wire ${f(1000 * (CUT.wireX - CLAMP.x - 0.03), 0)} mm from the jaws, ${f(1000 * (CUT.wireX - 0.01 - B.x1), 0)} mm from the carriage`);
  // the wire reaches the film between the brush edge and the clamp when the brush is on the face
  const yFilm = -HY + ((xb - CUT.wireX) / (xb - CLAMP.x)) * (CLAMP.y + HY);
  check('hot wire meets the film when the brush is on the load', abs(CUT.press - CUT.wireBack - yFilm) < 0.01, `film at y = ${f(yFilm)}, wire at ${f(CUT.press - CUT.wireBack)}`);
}

// ---------- 4. load stability ----------
console.log('\n4. Load on the turntable');
{
  let sfMin = 99, hold = 99;
  const aPeak = TABLE.wrap.a; // peak α and peak ω do not coincide, taking both is conservative
  for (const S of SKUS) {
    const boxes = [];
    for (let l = 0; l < S.layers[1]; l++) for (let i = 0; i < S.nx; i++) for (let j = 0; j < S.ny; j++) boxes.push({ sx: S.sx, sy: S.sy, sz: S.sz, lx: (i - (S.nx - 1) / 2) * S.sx, ly: (j - (S.ny - 1) / 2) * S.sy, lz: PALLET.h + (l + 0.5) * S.sz });
    const st = K.loadStability(K.columns(boxes), [LINE.stopTol, LINE.guideTol], W, aPeak), h = K.topLayerHold(S.nx * S.ny * S.mass) / FILM.F;
    console.log(`      ${S.sku} × ${S.layers[1]} layers: inertial accel ${f(st.acc, 2)} m/s², SF ${f(st.sf, 2)}; top layer vs film tension SF ${f(h, 2)}`);
    sfMin = min(sfMin, st.sf); hold = min(hold, h);
  }
  check('unwrapped load does not slide or tip at wrap speed', sfMin >= SAFE.sfLoad, `SF ${f(sfMin, 2)}`);
  check('film tension does not pull the top layer off', hold >= SAFE.sfLoad, `SF ${f(hold, 2)}`);
  const I = TABLE.inertia + (PALLET.maxMass * (PALLET.sx ** 2 + PALLET.sy ** 2)) / 12;
  console.log(`      turntable torque at rated load: ${f(I * TABLE.wrap.a, 0)} N·m inertia + ${f(FILM.Fmax * hypot(HX, HY), 0)} N·m film`);
}

// ---------- 5. safety monitor ----------
console.log('\n5. Safety monitor');
{
  const load = { off: [0.005, 0.01], sf: 3 };
  const idle = { th: 0, om: 0, al: 0, thCycle: 0, deckV: 0, lift: K.Z_BOTTOM, liftV: 0, slide: CUT.park, clampUp: 1, jaw: 1, b2: false, b4: false, filmOnLoad: false, corners: 0, dancer: 0, tension: FILM.F, vFeed: 0, load: null };
  const wrap = { ...idle, th: 20, thCycle: 20, om: W, clampUp: 0, jaw: 0, filmOnLoad: true, corners: 12, vFeed: 0.8, load };
  const legal = [
    ['idle at home', idle],
    ['transfer at home', { ...idle, deckV: 0.3, b2: true }],
    ['wrapping, carriage rising', { ...wrap, lift: 1.4, liftV: 0.045 }],
    ['first revolution, tail in the clamp', { ...wrap, th: 2, thCycle: 2, clampUp: 1, jaw: 1, corners: 1 }],
    ['cut slide pressed at home', { ...idle, slide: CUT.press, filmOnLoad: true, corners: 40, load }],
  ];
  const illegal = [
    ['R1', 'carriage above its travel', { ...wrap, lift: CARRIAGE.lift.max + 0.01 }],
    ['R1', 'cut slide beyond the load face', { ...idle, slide: CUT.press + 0.01 }],
    ['R2', 'turntable overspeed', { ...wrap, om: W * 1.05 }],
    ['R2', 'turntable reversing', { ...wrap, om: -0.2 }],
    ['R2', 'carriage overspeed', { ...wrap, liftV: 0.5 }],
    ['R3', 'deck conveyor running while rotating', { ...wrap, deckV: 0.1 }],
    ['R3', 'deck conveyor running 5° off home', { ...idle, th: 5 * DEG, deckV: 0.3 }],
    ['R4', 'rotating with a pallet across the gap', { ...wrap, b2: true }],
    ['R4', 'rotating with the cut slide extended', { ...wrap, slide: -1.0 }],
    ['R4', 'rotating a load 50 mm off centre', { ...wrap, load: { off: [0.03, 0.04], sf: 3 } }],
    ['R5', 'carriage moving with the table stopped', { ...wrap, om: 0, liftV: 0.045 }],
    ['R6', 'clamp still up after 345°', { ...wrap, clampUp: 1, jaw: 0, th: 350 * DEG, thCycle: 350 * DEG, corners: 4 }],
    ['R6', 'tail released after one corner', { ...wrap, th: 2, thCycle: 2, clampUp: 1, jaw: 0.5, corners: 1 }],
    ['R7', 'cut slide out with the jaw open', { ...idle, slide: -0.8, jaw: 0, filmOnLoad: true, corners: 40 }],
    ['R7', 'cut slide out with the carriage up', { ...idle, slide: -0.8, lift: 1.5 }],
    ['R8', 'load sweeps beyond its envelope', { ...wrap, load: { off: [0.10, 0.03], sf: 3 } }],
    ['R9', 'dancer at its stop', { ...wrap, dancer: -0.07 }],
    ['R9', 'film tension over the limit', { ...wrap, tension: 130 }],
    ['R9', 'film slack while wrapping', { ...wrap, tension: 10 }],
    ['R10', 'unstable load', { ...wrap, load: { off: [0, 0], sf: 1.2 } }],
  ];
  let ok = true;
  for (const [name, s] of legal) { const b = K.monitor(s); if (b.length) { ok = false; console.log(`      legal state rejected: ${name}: ${b}`); } }
  for (const [rule, name, s] of illegal) { const b = K.monitor(s); if (!b.some((x) => x.startsWith(rule + ' '))) { ok = false; console.log(`      not caught: ${rule} ${name}: ${b}`); } }
  check(`negative tests: ${illegal.length} illegal states caught by the right rule, ${legal.length} legal states accepted`, ok);
}

// ---------- 6. production run ----------
console.log('\n6. Production run');
{
  const HOURS = 4, DT = 1 / 240, cell = new Cell({ seed: 7 }), w = cell.w, s = cell.stats;
  const raw = { deck: 0, lift: 0, clamp: 0, slide: 0, grip: 0, jerk: 0 };
  let created = 0, gone = 0, minGap = 9, eHome = 0, eBal = 0, cover = [99, 0], eTop = 9, eLock = 9, lag = 0, done = 0, prevAl = 0, prevN = 0, tWait = 0;
  const seen = new Set();
  for (let i = 0; i < HOURS * 3600 / DT; i++) {
    const phase = w.phase;
    cell.step(DT);
    // raw checks, independent of the monitor
    if (abs(w.deckV) > 0 && (w.om !== 0 || w.th !== 0)) raw.deck++;
    if (w.liftV !== 0 && w.om < WRAP.omLift) raw.lift++;
    if (w.clampUp > 0 && w.om > 0 && w.th > WRAP.clampDownBy) raw.clamp++;
    if (w.slide > CUT.park && (w.om !== 0 || w.th !== 0 || w.jaw < 1 || w.lift !== K.Z_BOTTOM)) raw.slide++;
    if (w.filmOnLoad && w.jaw < 1 && w.pallet.film.corners < WRAP.anchorCorners) raw.grip++;
    raw.jerk = max(raw.jerk, abs(w.al - prevAl) / DT); prevAl = w.al;
    if (w.liftV !== 0) lag = max(lag, abs(w.lift - w.pallet.film.active.z));
    if (phase === 'WRAP' && w.phase === 'CLAMP') eHome = max(eHome, abs(K.wrapPi(w.turn.at(cell.t).p)));
    if (w.phase === 'OUT') tWait += DT;
    const xs = [...cell.pallets].map((p) => p.x).sort((a, b) => a - b);
    for (let k = 1; k < xs.length; k++) minGap = min(minGap, xs[k] - xs[k - 1] - PALLET.sx);
    for (const p of cell.pallets) if (!seen.has(p.id)) { seen.add(p.id); created++; }
    if (cell.pallets.size < prevN) gone += prevN - cell.pallets.size;
    prevN = cell.pallets.size;
    if (s.wrapped > done) { // a pallet has just been cut loose: check what is on it
      done = s.wrapped;
      const p = w.pallet, film = p.film, c = s.cycles.at(-1), z0 = LINE.z + PALLET.h, z1 = LINE.z + p.H;
      // film paid out = film on the load + free film to the roller + what the clamp now holds − what it held at the start
      eBal = max(eBal, abs(c.film - (film.laid - (HX - CUT.tailX + p.dx) + K.filmLine([HX + p.dx, -HY + p.ey]).len - K.filmLine([CLAMP.x, CLAMP.y]).len + w.dancer)));
      let top = 0, low = 9;
      for (const a of film.anchors.slice(2)) { top = max(top, a.z + FILM.w / 2); low = min(low, a.z - FILM.w / 2); }
      eTop = min(eTop, top - z1); eLock = min(eLock, z0 - low);
      for (let sp = 0.005; sp < K.PERIMETER; sp += 0.03) for (let z = z0 - 0.03; z <= z1; z += 0.02) { const n = film.layers(sp, z); cover = [min(cover[0], n), max(cover[1], n)]; }
    }
  }
  const N = s.cycles.length, mean = (a) => a.reduce((x, y) => x + y, 0) / a.length;
  check('safety monitor violations during the run', s.violations === 0, `${s.violations}`);
  check('deck conveyor moved only at home (raw check)', raw.deck === 0, `${raw.deck} steps`);
  check('carriage moved only at wrap speed (raw check)', raw.lift === 0, `${raw.lift} steps`);
  check('clamp below the film after 345° of every cycle (raw check)', raw.clamp === 0, `${raw.clamp} steps`);
  check('cut slide left its park position only at home, jaw closed, carriage down (raw check)', raw.slide === 0, `${raw.slide} steps`);
  check('tail never released before three corners (raw check)', raw.grip === 0, `${raw.grip} steps`);
  check('turntable stops on the home angle after every wrap', eHome < 1e-9, e(eHome) + ' rad');
  check('peak turntable speed / acceleration / jerk', s.om <= W * 1.0001 && s.al <= TABLE.wrap.a * 1.0001 && raw.jerk <= TABLE.wrap.j * 1.01, `${f(s.om / RPM, 2)} rpm / ${f(s.al, 3)} rad/s² / ${f(raw.jerk, 2)} rad/s³`);
  check('peak carriage / deck / cut slide speed', s.liftV <= CARRIAGE.lift.v && s.deckV <= LINE.move.v * 1.0001 && s.slideV <= CUT.move.v * 1.0001, `${f(s.liftV, 4)} / ${f(s.deckV)} / ${f(s.slideV)} m/s`);
  check('film speed and pre-stretch acceleration inside the drive limits', s.vFeed < FILM.feed.vMax && s.aFeed < FILM.feed.aMax, `${f(s.vFeed)} of ${FILM.feed.vMax} m/s, ${f(s.aFeed, 2)} of ${FILM.feed.aMax} m/s²`);
  check('dancer stays inside its stroke', s.dancer < FILM.dancer.stroke, `±${f(1000 * s.dancer, 1)} of ±${f(1000 * FILM.dancer.stroke, 0)} mm`);
  check('film tension inside its limits while wrapping', s.tension[0] > FILM.Fmin && s.tension[1] < FILM.Fmax, `${f(s.tension[0], 1)} … ${f(s.tension[1], 1)} N`);
  check('film direction at the exit roller', s.alpha[0] > CARRIAGE.exit[0] && s.alpha[1] < CARRIAGE.exit[1], `${deg(s.alpha[0])} … ${deg(s.alpha[1])}`);
  check('moment arm range', s.arm[0] > 0.3, `${f(s.arm[0])} … ${f(s.arm[1])} m`);
  check('film paid out = film on the load + free film (every pallet)', eBal < 0.01, `worst ${f(1000 * eBal, 2)} mm of ${f(mean(s.cycles.map((c) => c.film)), 1)} m mean`);
  check('every point of every load face is under film', cover[0] >= 1, `${cover[0]} … ${cover[1]} layers`);
  check('film reaches over the top of the load and locks it to the pallet', eTop > 0 && eLock > 0.05, `≥ ${f(1000 * eTop, 0)} mm above the load, ≥ ${f(1000 * eLock, 0)} mm below the pallet deck`);
  check('least stability SF of an unwrapped load / largest swept radius', s.sf >= SAFE.sfLoad && s.swept < SAFE.rLoad, `SF ${f(s.sf, 2)} / ${f(s.swept)} m`);
  check('pallets never closer than half a pallet on the line', minGap > 0.5, `${f(minGap)} m`);
  check('pallets conserved', created === gone + cell.pallets.size && s.wrapped <= created, `${created} created = ${gone} left + ${cell.pallets.size} on the line; ${s.wrapped} wrapped`);
  console.log(`      largest height lag of the film contact point behind the carriage: ${f(1000 * lag, 0)} mm`);
  console.log(`      ${N} pallets wrapped in ${HOURS} h = ${f(N / HOURS, 1)} / h; rotation ${f(min(...s.cycles.map((c) => c.revs)), 0)} … ${f(max(...s.cycles.map((c) => c.revs)), 0)} rev; wrap+cut ${f(mean(s.cycles.map((c) => c.t)), 1)} s mean (${f(min(...s.cycles.map((c) => c.t)), 1)} … ${f(max(...s.cycles.map((c) => c.t)), 1)})`);
  console.log(`      pallet to pallet ${f(mean(s.period), 1)} s mean; film ${f(mean(s.cycles.map((c) => c.film)), 1)} m stretched = ${f(mean(s.cycles.map((c) => c.film)) / FILM.lambda, 1)} m off the roll per pallet; ${s.rolls} roll change(s); longest infeed queue ${s.queue}; waiting for the outfeed ${f(tWait, 1)} s`);
}

console.log(fails ? `\n${fails} CHECK(S) FAILED` : '\nall checks passed');
process.exit(fails ? 1 : 0);
