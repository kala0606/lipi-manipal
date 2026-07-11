/* ============================================================================
   saxi-plan.js — exact plot-time, ported from saxi (planning.ts + massager.ts)
   github.com/nornagon/saxi  ·  motion model cribbed from fogleman/axi
   Pure JS, no deps. Give it paths in millimetres; get seconds back.
   Pipeline matches saxi defaults: greedy reorder → join ≤0.5mm → ×5 steps →
   trapezoidal/triangular velocity planning with cornering-limited entry speeds.
   Tune PROFILE below to match your machine / speed settings.
============================================================================ */
(function (global) {
  const EPS = 1e-9;
  const vadd = (a, b) => ({ x: a.x + b.x, y: a.y + b.y });
  const vsub = (a, b) => ({ x: a.x - b.x, y: a.y - b.y });
  const vmul = (a, s) => ({ x: a.x * s, y: a.y * s });
  const vdot = (a, b) => a.x * b.x + a.y * b.y;
  const vlen = (a) => Math.hypot(a.x, a.y);
  const vnorm = (a) => { const l = vlen(a) || 1; return { x: a.x / l, y: a.y / l }; };
  const dist = (a, b) => Math.hypot(a.x - b.x, a.y - b.y);

  const SPMM = 5;                      // AxiDraw steps per mm
  // Match these to your saxi "more" panel. Currently set to KALA's rig:
  // down acc 200, down max vel 200, cornering 0.127, up acc 200, up max vel 200.
  const PROFILE = {
    penDownProfile: { acceleration: 200 * SPMM, maximumVelocity: 200 * SPMM, corneringFactor: 0.127 * SPMM },
    penUpProfile:   { acceleration: 200 * SPMM, maximumVelocity: 200 * SPMM, corneringFactor: 0 },
    penDropDuration: 0.12,
    penLiftDuration: 0.12,
  };

  class Block {
    constructor(accel, duration, vInitial, p1, p2) {
      this.accel = accel; this.duration = duration; this.vInitial = vInitial;
      this.p1 = p1; this.p2 = p2; this.distance = vlen(vsub(p1, p2));
    }
  }
  class XYMotion {
    constructor(blocks) { this.blocks = blocks; }
    duration() { return this.blocks.reduce((a, b) => a + b.duration, 0); }
    get p1() { return this.blocks[0].p1; }
    get p2() { return this.blocks[this.blocks.length - 1].p2; }
  }
  class PenMotion { constructor(d) { this.d = d; } duration() { return this.d; } }
  class Plan {
    constructor(motions) { this.motions = motions; }
    duration() { return this.motions.reduce((a, m) => a + m.duration(), 0); }
  }

  class Segment {
    constructor(p1, p2) { this.p1 = p1; this.p2 = p2; this.blocks = []; this.maxEntryVelocity = 0; this.entryVelocity = 0; }
    length() { return vlen(vsub(this.p2, this.p1)); }
    direction() { return vnorm(vsub(this.p2, this.p1)); }
  }
  function cornerVelocity(s1, s2, vMax, accel, cf) {
    const cosine = -vdot(s1.direction(), s2.direction());
    if (Math.abs(cosine - 1) < EPS) return 0;
    const sine = Math.sqrt((1 - cosine) / 2);
    if (Math.abs(sine - 1) < EPS) return vMax;
    return Math.min(Math.sqrt((accel * cf * sine) / (1 - sine)), vMax);
  }
  function computeTriangle(distance, vi, vf, accel, p1, p3) {
    const s1 = (2 * accel * distance + vf * vf - vi * vi) / (4 * accel);
    const s2 = distance - s1;
    const vMax = Math.sqrt(vi * vi + 2 * accel * s1);
    const t1 = (vMax - vi) / accel, t2 = (vf - vMax) / -accel;
    const p2 = vadd(p1, vmul(vnorm(vsub(p3, p1)), s1));
    return { s1, s2, t1, t2, vMax, p1, p2, p3 };
  }
  function computeTrapezoid(distance, vi, vmax, vf, accel, p1, p4) {
    const t1 = (vmax - vi) / accel, s1 = ((vmax + vi) / 2) * t1;
    const t3 = (vf - vmax) / -accel, s3 = ((vf + vmax) / 2) * t3;
    const s2 = distance - s1 - s3, t2 = s2 / vmax;
    const dir = vnorm(vsub(p4, p1));
    const p2 = vadd(p1, vmul(dir, s1)), p3 = vadd(p1, vmul(dir, distance - s3));
    return { s1, s2, s3, t1, t2, t3, p1, p2, p3, p4 };
  }
  function dedup(points) {
    const out = [points[0]];
    for (let i = 1; i < points.length; i++)
      if (vlen(vsub(points[i], out[out.length - 1])) > EPS) out.push(points[i]);
    return out;
  }
  function constantAccelerationPlan(points, profile) {
    const pts = dedup(points);
    if (pts.length === 1) return new XYMotion([new Block(0, 0, 0, pts[0], pts[0])]);
    const segments = pts.slice(1).map((a, i) => new Segment(pts[i], a));
    const accel = profile.acceleration, vMax = profile.maximumVelocity, cf = profile.corneringFactor;
    segments.slice(1).forEach((seg2, i) => { seg2.maxEntryVelocity = cornerVelocity(segments[i], seg2, vMax, accel, cf); });
    const last = pts[pts.length - 1];
    segments.push(new Segment(last, last));
    let i = 0;
    while (i < segments.length - 1) {
      const seg = segments[i], next = segments[i + 1];
      const distance = seg.length(), vi = seg.entryVelocity, vExit = next.maxEntryVelocity;
      const p1 = seg.p1, p2 = seg.p2;
      const m = computeTriangle(distance, vi, vExit, accel, p1, p2);
      if (m.s1 < -EPS) {
        seg.maxEntryVelocity = Math.sqrt(vExit * vExit + 2 * accel * distance); i -= 1;
      } else if (m.s2 <= 0) {
        const vF = Math.sqrt(vi * vi + 2 * accel * distance), t = (vF - vi) / accel;
        seg.blocks = [new Block(accel, t, vi, p1, p2)]; next.entryVelocity = vF; i += 1;
      } else if (m.vMax > vMax) {
        const z = computeTrapezoid(distance, vi, vMax, vExit, accel, p1, p2);
        seg.blocks = [new Block(accel, z.t1, vi, z.p1, z.p2), new Block(0, z.t2, vMax, z.p2, z.p3), new Block(-accel, z.t3, vMax, z.p3, z.p4)];
        next.entryVelocity = vExit; i += 1;
      } else {
        seg.blocks = [new Block(accel, m.t1, vi, m.p1, m.p2), new Block(-accel, m.t2, m.vMax, m.p2, m.p3)];
        next.entryVelocity = vExit; i += 1;
      }
    }
    const blocks = [];
    for (const seg of segments) for (const b of seg.blocks) if (b.duration > EPS) blocks.push(b);
    return new XYMotion(blocks.length ? blocks : [new Block(0, 0, 0, pts[0], pts[pts.length - 1])]);
  }
  function plan(paths, profile, penHome) {
    const home = penHome || { x: 0, y: 0 };
    const motions = []; let cur = home;
    for (const path of paths) {
      const motion = constantAccelerationPlan(path, profile.penDownProfile);
      const position = constantAccelerationPlan([cur, motion.p1], profile.penUpProfile);
      motions.push(position, new PenMotion(profile.penDropDuration), motion, new PenMotion(profile.penLiftDuration));
      cur = motion.p2;
    }
    motions.push(constantAccelerationPlan([cur, home], profile.penUpProfile));
    return new Plan(motions);
  }

  // greedy nearest-neighbour reorder with endpoint flipping (≈ saxi sortPaths)
  function reorder(paths) {
    const n = paths.length; if (n < 2) return paths.slice();
    const used = new Array(n).fill(false);
    const out = [paths[0]]; used[0] = true; let end = paths[0][paths[0].length - 1];
    for (let k = 1; k < n; k++) {
      let best = -1, bestD = Infinity, flip = false;
      for (let i = 0; i < n; i++) {
        if (used[i]) continue;
        const pl = paths[i], s = pl[0], e = pl[pl.length - 1];
        const ds = dist(end, s); if (ds < bestD) { bestD = ds; best = i; flip = false; }
        const de = dist(end, e); if (de < bestD) { bestD = de; best = i; flip = true; }
      }
      let pl = paths[best]; if (flip) pl = pl.slice().reverse();
      used[best] = true; out.push(pl); end = pl[pl.length - 1];
    }
    return out;
  }
  function joinNearby(paths, r) {
    if (paths.length < 2 || r <= 0) return paths.map((p) => p.slice());
    const out = [paths[0].slice()];
    for (let i = 1; i < paths.length; i++) {
      const last = out[out.length - 1], nx = paths[i];
      if (dist(last[last.length - 1], nx[0]) <= r) {
        const st = dist(last[last.length - 1], nx[0]) < EPS ? 1 : 0;
        for (let j = st; j < nx.length; j++) last.push(nx[j]);
      } else out.push(nx.slice());
    }
    return out;
  }

  /** Exact plot seconds for paths given in millimetres. Excludes the pen-up
   *  approach from / return to home, so it reads as "time to draw these marks". */
  function duration(pathsMm, opts) {
    opts = opts || {};
    const sort = opts.sort !== false;
    const joinR = opts.joinR == null ? 0.5 : opts.joinR;
    let p = pathsMm.filter((pl) => pl && pl.length >= 1);
    if (!p.length) return 0;
    if (sort) p = reorder(p);
    if (joinR > 0) p = joinNearby(p, joinR);
    p = p.map((pl) => pl.map((pt) => ({ x: pt.x * SPMM, y: pt.y * SPMM })));
    const home = p[0][0];
    const thePlan = plan(p, PROFILE, home);
    const ret = thePlan.motions[thePlan.motions.length - 1].duration();
    return thePlan.duration() - ret;
  }

  global.SAXI = { duration, plan, reorder, joinNearby, PROFILE, SPMM };
})(window);
