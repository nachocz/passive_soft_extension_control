// The controller of Sec. IV, the analysis quantities of Sec. V, and the closed loop of Sec. VI:
// a port of softext/constraints.py, barriers.py, controller.py, scenarios.py, simulation.py, and
// analysis.py.
import {
    add, col, cross, dot, eye, matmul, matvec, mscale, norm, Random, rank, scale, singularValues, solve, solveQP, sub, symEig,
    transpose, vec, vecmat, zeros,
} from './linalg.js';
import { Arm, differential, softResponse, spanJacobian } from './model.js';

const DEG = Math.PI / 180;

// ---------------------------------------------------------------- EE constraints (Table II)
export class EEConstraint {
    constructor(kind = 'free', { pStar = null, c = null } = {}) {
        this.kind = kind;
        this.pStar = pStar && pStar.slice();
        this.c = c && c.slice();
    }

    get rank() { return { free: 0, position: 3, fulcrum: 2 }[this.kind]; }

    error(frames) {
        const B = frames[6];
        if (this.kind === 'position') return sub(this.pStar, B.p);
        if (this.kind === 'fulcrum') { const w = sub(this.c, B.p); return [dot(col(B.R, 0), w), dot(col(B.R, 1), w)]; }
        return [];
    }

    jacobian(frames, JB) {                       // J_c, with d e_c / dt = -J_c qdot (12)
        const B = frames[6], Jv = JB.slice(0, 3), Jw = JB.slice(3);
        if (this.kind === 'position') return Jv.map((row) => row.slice());
        if (this.kind === 'fulcrum') {
            const w = sub(this.c, B.p);
            return [0, 1].map((i) => {
                const u = col(B.R, i);
                return sub(vecmat(u, Jv), vecmat(cross(u, w), Jw));
            });
        }
        return [];
    }

    deviation(frames) { return this.kind === 'free' ? 0 : norm(this.error(frames)); }
}

export function pinv(J) {
    if (!J.length) return zeros(6, 0);
    return matmul(transpose(J), solve(matmul(J, transpose(J)), eye(J.length)));
}
export function projector(J) {                   // N_c = I - J_c^+ J_c (13)
    if (!J.length) return eye(6);
    const P = matmul(pinv(J), J);
    return eye(6).map((row, i) => row.map((v, j) => v - P[i][j]));
}

// ---------------------------------------------------------------- barriers (Table III)
export class Obstacle {
    constructor(name, kind, center, size) {
        this.name = name; this.kind = kind; this.center = center.slice();
        this.size = kind === 'box' ? (Array.isArray(size) ? size.slice() : [size, size, size]) : [Array.isArray(size) ? size[0] : size];
    }

    distance(p) {
        const offset = sub(p, this.center);
        if (this.kind === 'sphere') {
            const d = norm(offset);
            if (d < 1e-12) return [-this.size[0], [1, 0, 0]];
            return [d - this.size[0], scale(offset, 1 / d)];
        }
        const sign = offset.map((v) => (v >= 0 ? 1 : -1));
        const delta = offset.map((v, i) => Math.abs(v) - 0.5 * this.size[i]);
        const outside = delta.map((v) => Math.max(v, 0)), length = norm(outside);
        if (length > 1e-12) return [length, outside.map((v, i) => sign[i] * v / length)];
        let axis = 0;
        for (let i = 1; i < 3; i++) if (delta[i] > delta[axis]) axis = i;
        const grad = [0, 0, 0]; grad[axis] = sign[axis];
        return [delta[axis], grad];
    }

    closestFraction(a, b) {
        const edge = sub(b, a), length = dot(edge, edge);
        if (length < 1e-18) return 0;
        if (this.kind === 'sphere') return Math.min(Math.max(dot(sub(this.center, a), edge) / length, 0), 1);
        const at = (x) => this.distance(add(a, scale(edge, x)))[0];
        const golden = 0.5 * (Math.sqrt(5) - 1);
        let lo = 0, hi = 1, x1 = hi - golden * (hi - lo), x2 = lo + golden * (hi - lo), f1 = at(x1), f2 = at(x2);
        for (let k = 0; k < 60; k++) {
            if (f1 <= f2) { hi = x2; x2 = x1; f2 = f1; x1 = hi - golden * (hi - lo); f1 = at(x1); }
            else { lo = x1; x1 = x2; f1 = f2; x2 = lo + golden * (hi - lo); f2 = at(x2); }
        }
        let best = 0.5 * (lo + hi);
        for (const end of [0, 1]) if (at(end) < at(best)) best = end;
        return best;
    }

    clearance(points) {
        let best = Infinity;
        for (let k = 0; k + 1 < points.length; k++) {
            const a = points[k], b = points[k + 1], f = this.closestFraction(a, b);
            best = Math.min(best, this.distance(add(a, scale(sub(b, a), f)))[0]);
        }
        return best;
    }
}

export function obstacleRow(obstacle, points, jacobians, cfg) {
    const dist = [], dirs = [], fracs = [];
    for (let k = 0; k + 1 < points.length; k++) {
        const a = points[k], b = points[k + 1], f = obstacle.closestFraction(a, b);
        const [gap, n] = obstacle.distance(add(a, scale(sub(b, a), f)));
        dist.push(gap - cfg.offset); dirs.push(n); fracs.push(f);
    }
    const floor = Math.min(...dist);
    const w = dist.map((d) => Math.exp(-(d - floor) / cfg.smoothing)), total = w.reduce((x, y) => x + y, 0);
    const h = floor - cfg.smoothing * Math.log(total);
    const grad = vec(6);
    dirs.forEach((n, k) => {
        const share = w[k] / total, f = fracs[k];
        const moved = jacobians[k].map((row, i) => row.map((v, j) => (1 - f) * v + f * jacobians[k + 1][i][j]));
        const g = vecmat(n, moved);
        for (let j = 0; j < 6; j++) grad[j] += share * g[j];
    });
    return { name: obstacle.name, kind: 'obstacle', h, grad, alpha: cfg.alpha, eps: cfg.eps };
}

export function spanRows(spans, Jlq, lower, upper, cfg) {
    const rows = [];
    spans.forEach((s, i) => {
        rows.push({ name: `span ${i + 1} lower`, kind: 'span', h: s - lower[i], grad: Jlq[i].slice(), alpha: cfg.alpha, eps: cfg.eps });
        rows.push({ name: `span ${i + 1} upper`, kind: 'span', h: upper[i] - s, grad: Jlq[i].map((v) => -v), alpha: cfg.alpha, eps: cfg.eps });
    });
    return rows;
}

export class Fulcrum {
    constructor(c, { axis = [0, 0, -1], apertureRadius = 0, rodRadius = 0.009 } = {}) {
        this.c = c.slice(); this.axis = axis.slice(); this.apertureRadius = apertureRadius; this.rodRadius = rodRadius;
    }
}

// Tilt theta of the tool axis u from the aperture axis, insertion depth delta = u^T (c - p_B), distance d_c.
export function fulcrumGeometry(fulcrum, frames) {
    const B = frames[6], u = col(B.R, 2), w = sub(fulcrum.c, B.p);
    const cos = Math.min(Math.max(dot(u, fulcrum.axis), -1), 1), delta = dot(u, w);
    return { u, w, theta: Math.acos(cos), delta, dc: norm(sub(w, scale(u, delta))) };
}

export function fulcrumRows(fulcrum, frames, JB, cfg) {
    const { u, w, theta, delta } = fulcrumGeometry(fulcrum, frames);
    const Jv = JB.slice(0, 3), Jw = JB.slice(3);
    const du = transpose(transpose(Jw).map((omega) => cross(omega, u)));      // 3 x 6
    const cos = Math.cos(theta), dcos = vecmat(fulcrum.axis, du);
    const sin = Math.sqrt(Math.max(1 - cos * cos, 0));
    const dtheta = sin > 1e-9 ? dcos.map((v) => -v / sin) : vec(6);
    const ddelta = sub(vecmat(cross(u, w), Jw), vecmat(u, Jv));
    const cone = cfg.cone, ins = cfg.insertion;
    const rows = [
        { name: 'cone', kind: 'cone', h: cone.half_angle_deg * DEG - theta, grad: dtheta.map((v) => -v), alpha: cone.alpha, eps: cone.eps_deg * DEG },
        { name: 'insertion min', kind: 'insertion', h: delta - ins.min, grad: ddelta, alpha: ins.alpha, eps: ins.eps },
        { name: 'insertion max', kind: 'insertion', h: ins.max - delta, grad: ddelta.map((v) => -v), alpha: ins.alpha, eps: ins.eps },
    ];
    if (fulcrum.apertureRadius > 0) {
        const perp = sub(w, scale(u, delta)), dc = norm(perp);
        const dperp = Jv.map((row, i) => row.map((v, j) => -v - u[i] * ddelta[j] - delta * du[i][j]));
        const ddc = dc > 1e-12 ? vecmat(perp, dperp).map((v) => v / dc) : vec(6);
        const r = fulcrum.apertureRadius - fulcrum.rodRadius;
        rows.push({ name: 'aperture', kind: 'aperture', h: r * cos - dc, grad: dcos.map((v, j) => r * v - ddc[j]),
            alpha: cfg.aperture.alpha, eps: cfg.aperture.eps });
    }
    return rows;
}

// ---------------------------------------------------------------- controller (controller.py)
export function spectralFilter(lam, xi, kind = 'dls') { return lam.map((l) => (kind === 'dls' ? 1 / (l + xi * xi) : 1)); }

// A^T f(A A^T) r (24)
export function spectralVelocity(A, r, xi, kind = 'dls') {
    const { values, vectors } = symEig(matmul(A, transpose(A)));
    const f = spectralFilter(values, xi, kind);
    const coeff = vecmat(r, vectors).map((v, i) => v * f[i]);
    return vecmat(matvec(vectors, coeff), A);
}

export class Controller {
    constructor(plant, constraint, spanMap, { obstacles = [], fulcrum = null, barriers = true, obstacleBarrier = true,
        spanBarrier = true, exact = false, filter = 'dls', W = null } = {}) {
        Object.assign(this, { plant, config: plant.config, constraint, spanMap, obstacles, fulcrum, barriers, obstacleBarrier,
            spanBarrier, exact, filter, W });
        this.gains(plant.config.control);
    }

    gains(c) {
        Object.assign(this, { k_t: c.k_t, k_c: c.k_c, xi: c.xi, mu: c.mu, w_c: c.w_c, qdotMax: c.qdot_max });
    }

    rows(frames, JB, spans, diff) {
        const cfg = this.config.barriers;
        const rows = this.spanBarrier ? spanRows(spans, diff.Jlq, this.spanMap.lower, this.spanMap.upper, cfg.span) : [];
        if (this.obstacleBarrier && this.obstacles.length) {
            const B = frames[6];
            const points = this.spanMap.predictShape(spans).map((lp) => add(B.p, matvec(B.R, lp)));
            for (const o of this.obstacles) rows.push(obstacleRow(o, points, diff.Jp, cfg.obstacle));
        }
        if (this.fulcrum) rows.push(...fulcrumRows(this.fulcrum, frames, JB, cfg));
        return rows;
    }

    command(q, tip, reference, z = null) {
        const frames = this.plant.arm.frames(q);
        const { anchors, entries } = this.plant.endpoints(frames);
        const spans = anchors.map((a, i) => norm(sub(a, entries[i])));
        const diff = differential(this.plant, frames, tip, spans, this.exact ? null : this.spanMap, z);
        if (this.exact && this.spanMap.shape) diff.Jp = differential(this.plant, frames, tip, spans, this.spanMap).Jp;
        const { JB, Jt } = diff;
        const ec = this.constraint.error(frames), Jc = this.constraint.jacobian(frames, JB);
        const vStar = scale(sub(reference, tip), this.k_t);
        const qdotC = Jc.length ? matvec(pinv(Jc), scale(ec, this.k_c)) : vec(6);
        const Nc = projector(Jc);
        const A = matmul(Jt, Nc);
        const vRes = sub(vStar, matvec(Jt, qdotC));
        const nominal = add(qdotC, matvec(Nc, spectralVelocity(A, this.W ? matvec(this.W, vRes) : vRes, this.xi, this.filter)));
        const rows = this.barriers ? this.rows(frames, JB, spans, diff) : [];
        const base = { nominal, rows, ec, A, Nc, diff, spans, frames, vStar };
        if (!rows.length) {
            const s = Math.max(1, Math.max(...nominal.map(Math.abs)) / this.qdotMax);
            return { ...base, qdot: nominal.map((v) => v / s), qp: false };
        }
        const JtT = transpose(Jt), JcT = transpose(Jc);
        let H = matmul(JtT, Jt);
        if (Jc.length) H = H.map((row, i) => row.map((v, j) => v + this.w_c * dot(JcT[i], JcT[j])));
        H = H.map((row, i) => row.map((v, j) => v + (i === j ? this.mu * this.mu : 0)));
        let g = matvec(JtT, vStar);
        if (Jc.length) g = add(g, scale(matvec(JcT, ec), this.w_c * this.k_c));
        g = scale(g, -1);
        const I6 = eye(6);
        const A_in = [...rows.map((r) => r.grad), ...I6, ...mscale(I6, -1)];
        const rhs = rows.map((r) => -r.alpha * (r.h - r.eps)), bounds = vec(12, -this.qdotMax);
        let qdot = solveQP(H, g, A_in, [...rhs, ...bounds]);
        if (!qdot) qdot = solveQP(H, g, A_in, [...rhs.map((v) => Math.min(v, 0)), ...bounds]);
        return { ...base, qdot: qdot.map((v) => Math.min(Math.max(v, -this.qdotMax), this.qdotMax)), qp: true };
    }
}

// ---------------------------------------------------------------- scenarios (scenarios.py, Table V)
export const FULCRUM = [-0.8437, -0.1639, 0.2283];
const sphere = (name, x, y, z, radius) => ({ name, kind: 'sphere', center: [x, y, z], size: radius });
const box = (name, x, y, z, edge) => ({ name, kind: 'box', center: [x, y, z], size: edge });
export const SCENARIOS = {
    S1: { key: 'S1', title: 'S1: position-constrained EE', constraint: 'position', fulcrum: null, infeasible: [],
        targets: [[-0.907362, -0.061136, 0.072054], [-0.695992, -0.113576, 0.120694], [-0.883322, -0.100471, 0.057675],
            [-0.753462, -0.277474, 0.10188], [-0.728124, -0.111937, 0.09271], [-0.825047, -0.070726, 0.063259],
            [-0.708042, -0.10993, 0.1074], [-0.786902, -0.083479, 0.066897], [-0.732699, -0.129059, 0.092891]],
        obstacles: [sphere('upper sphere', -0.805, -0.035, 0.155, 0.055), box('lower box', -0.7, -0.25, 0.25, 0.12),
            sphere('outer sphere', -1.02, -0.25, 0.25, 0.065), box('high box', -0.65, -0.04, 0.27, 0.1)] },
    S2: { key: 'S2', title: 'S2: laparoscopic fulcrum', constraint: 'fulcrum', fulcrum: FULCRUM, infeasible: [],
        targets: [[-0.753197, -0.099656, 0.044469], [-0.713412, -0.123692, 0.080384], [-0.771788, -0.092032, 0.070998],
            [-0.727814, -0.12106, 0.050137], [-0.695012, -0.095911, 0.088171]],
        obstacles: [sphere('polyp', -0.7392, -0.0963, 0.0824, 0.016)] },
    S3: { key: 'S3', title: 'S3: infeasible targets', constraint: 'fulcrum', fulcrum: FULCRUM, infeasible: [3, 5],
        targets: [[-0.713289, -0.123741, 0.080453], [-0.772029, -0.092165, 0.070856], [-0.745157, -0.070506, 0.005626],
            [-0.743763, -0.117458, 0.049452], [-0.736013, -0.06184, -0.015036], [-0.713289, -0.123741, 0.080453]],
        obstacles: [sphere('blocked target', -0.745157, -0.070506, 0.005626, 0.03), sphere('rigid-tool gate', -0.89003, -0.20384, 0.31706, 0.03),
            sphere('slalom near', -0.74269, -0.03197, 0.00632, 0.02), sphere('slalom far', -0.70724, -0.06508, 0.01266, 0.02)] },
};

// ---------------------------------------------------------------- closed loop (simulation.py)
export function advanceReference(reference, goal, speed, dt) {
    const delta = sub(goal, reference), d = norm(delta);
    return d <= speed * dt ? goal.slice() : add(reference, scale(delta, speed * dt / d));
}

export class Simulation {
    // options: exact, adapt, barriers, obstacleBarrier, spanBarrier, filter, noise, seed, constraint (override)
    constructor(plant, scenario, spanMap, options = {}) {
        const config = this.config = plant.config;
        this.plant = plant;
        plant.reset();
        this.scenario = scenario;
        this.spanMap = spanMap.copy();
        this.spanMap.reset();
        this.adapt = options.adapt ?? true;
        const sensing = config.sensing, rate = sensing.plant_rate;
        this.noise = options.noise ?? sensing.noise;
        this.rng = new Random(options.seed ?? sensing.seed);
        this.dt = 1 / rate;
        this.senseEvery = Math.round(rate / sensing.rate);
        this.controlEvery = Math.round(rate / config.control.rate);
        this.q = plant.q0.slice();
        this.qdot = vec(6);
        this.state = plant.evaluate(this.q);
        const kind = options.constraint ?? scenario.constraint;
        const fulcrumPoint = scenario.fulcrum ?? FULCRUM;
        const constraint = kind === 'position' ? new EEConstraint('position', { pStar: this.state.p_B })
            : kind === 'fulcrum' ? new EEConstraint('fulcrum', { c: fulcrumPoint }) : new EEConstraint('free');
        const fulcrum = kind === 'fulcrum' || kind === 'aperture'
            ? new Fulcrum(fulcrumPoint, { apertureRadius: kind === 'aperture' ? (options.apertureRadius ?? 0.05) : 0 }) : null;
        if (kind === 'aperture') constraint.kind = 'free';
        this.constraintKind = kind;
        this.obstacles = scenario.obstacles.map((o) => new Obstacle(o.name, o.kind, o.center, o.size));
        this.controller = new Controller(plant, constraint, this.spanMap, {
            obstacles: this.obstacles, fulcrum, barriers: options.barriers ?? true, obstacleBarrier: options.obstacleBarrier ?? true,
            spanBarrier: options.spanBarrier ?? true, exact: options.exact ?? false, filter: options.filter ?? 'dls' });
        this.targets = scenario.targets.map((t) => t.slice());
        const t = config.targets, controlDt = this.dt * this.controlEvery;
        this.tolerance = t.tolerance; this.release = t.tolerance + t.hysteresis;
        this.holdTicks = Math.round(t.hold / controlDt);
        this.stallTicks = Math.round(t.stall / controlDt);
        this.budgetTicks = Math.round(t.budget / controlDt);
        this.stallProgress = t.stall_progress;
        this.tick = 0; this.index = 0; this.finished = false;
        this.resetTarget();
        this.events = [];
        this.log = { t: [], q: [], qdot: [], tip: [], tipMeasured: [], reference: [], goal: [], target: [], error: [], spans: [],
            tensions: [], eeDeviation: [], qp: [] };
        this.barrierLog = {};
        this.margins = {};
        this.measured = null; this.previous = null;
        this.measure();
        this.reference = this.measured.tip.slice();
        this.last = null;
    }

    get time() { return this.tick * this.dt; }

    resetTarget() { this.held = 0; this.stalled = 0; this.targetTicks = 0; this.best = Infinity; }

    measure() {
        const s = this.state, n = this.noise;
        const tip = add(s.p_D, scale(this.rng.normals(3), n));
        const points = s.bodyPoints.map((p) => add(p, scale(this.rng.normals(3), n)));
        this.measured = { tip, points };
    }

    control() {
        if (this.finished) { this.qdot = vec(6); return; }
        const goal = this.targets[this.index], tip = this.measured.tip;
        if (this.adapt) {
            const s = this.state, local = vecmat(sub(tip, s.p_B), s.R_B);              // R_B^T (tip - p_B)
            if (this.previous) this.spanMap.update(s.spans, this.previous[0], sub(local, this.previous[1]));
            this.previous = [s.spans.slice(), local];
        }
        this.reference = advanceReference(this.reference, goal, this.config.control.reference_speed, this.dt * this.controlEvery);
        const cmd = this.controller.command(this.q, tip, this.reference, this.state.z);
        this.qdot = cmd.qdot;
        this.last = cmd;
        this.record(cmd, goal);
        const error = norm(sub(goal, tip));
        if (error < this.tolerance) this.held += 1;
        else if (error >= this.release) this.held = 0;
        const dwelling = this.held > 0 && error < this.release;
        const improved = error <= this.best - this.stallProgress;
        if (improved) this.best = error;
        this.stalled = dwelling || improved ? 0 : this.stalled + 1;
        this.targetTicks += 1;
        const outcome = this.held >= this.holdTicks ? 'reached' : this.stalled >= this.stallTicks ? 'stalled'
            : this.targetTicks >= this.budgetTicks ? 'budget' : null;
        if (outcome) {
            this.events.push({ target: this.index + 1, t: this.time, outcome, error: norm(sub(goal, this.state.p_D)) });
            if (this.index + 1 < this.targets.length) { this.index += 1; this.resetTarget(); }
            else this.finished = true;
        }
    }

    record(cmd, goal) {
        const s = this.state, log = this.log;
        log.t.push(this.time); log.q.push(this.q.slice()); log.qdot.push(cmd.qdot.slice()); log.tip.push(s.p_D.slice());
        log.tipMeasured.push(this.measured.tip.slice()); log.reference.push(this.reference.slice()); log.goal.push(goal.slice());
        log.target.push(this.index + 1); log.error.push(norm(sub(goal, s.p_D))); log.spans.push(s.spans.slice());
        log.tensions.push(s.tensions.slice()); log.eeDeviation.push(this.controller.constraint.deviation(s.frames)); log.qp.push(cmd.qp);
        const n = log.t.length;
        for (const row of cmd.rows) {
            const values = this.barrierLog[row.name] ?? (this.barrierLog[row.name] = []);
            while (values.length < n - 1) values.push(NaN);
            values.push(row.h);
            this.margins[row.name] = row.eps;
        }
    }

    step() {
        const sense = this.tick % this.senseEvery === 0, control = this.tick % this.controlEvery === 0;
        if ((sense || control) && this.tick > 0) this.state = this.plant.evaluate(this.q);
        if (sense && this.tick > 0) this.measure();
        if (control) this.control();
        const limits = this.plant.arm.limits;
        this.q = this.q.map((v, j) => Math.min(Math.max(v + this.dt * this.qdot[j], limits[j][0]), limits[j][1]));
        this.tick += 1;
    }

    run(duration = Infinity) {
        const end = this.time + duration;
        while (!this.finished && this.time < end - 1e-9) this.step();
        return this.log;
    }
}

// ---------------------------------------------------------------- analysis (analysis.py)
export function rankReport(plant, state, constraint) {
    const frames = state.frames, JB = Arm.geometricJacobian(frames);
    const Jlq = spanJacobian(plant, frames);
    const Nc = projector(constraint.jacobian(frames, JB));
    const soft = softResponse(plant.rod, state.z, state.spans);
    const JlqNc = matmul(Jlq, Nc);
    const Jdef = matmul(matmul(soft.JDtau, soft.Ctau), JlqNc);
    const mA = soft.Ka.filter((k) => k !== 0).length;
    const out = { rankJDtau: rank(soft.JDtau), mA, rankJlq: rank(Jlq), nMinusRc: 6 - constraint.rank, rankJlqNc: rank(JlqNc),
        rankJdef: rank(Jdef), svJlq: singularValues(Jlq), svJlqNc: singularValues(JlqNc), svJdef: singularValues(Jdef),
        svJDtau: singularValues(soft.JDtau), Jlq, JlqNc, Jdef, soft };
    out.boundFirst = Math.min(out.rankJDtau, mA, out.rankJlqNc);
    out.boundSecond = Math.min(out.rankJDtau, mA, out.rankJlq, out.nMinusRc);
    return out;
}

// V, Vdot (28), beta, and the rates rho_i = k_t lambda_i f(lambda_i) (32), W = I.
export function descent(A, e, k_t, xi, kind = 'dls') {
    const { values, vectors } = symEig(matmul(A, transpose(A)));
    const lam = values.map((v) => Math.max(v, 0)), f = spectralFilter(lam, xi, kind);
    const proj = vecmat(e, vectors);
    let Vdot = 0;
    proj.forEach((p, i) => { Vdot -= k_t * p * p * lam[i] * f[i]; });
    return { V: 0.5 * dot(e, e), Vdot, beta: Math.min(...lam.map((l, i) => l * f[i])), rates: lam.map((l, i) => k_t * l * f[i]), lambda: lam,
        grad: norm(vecmat(e, A)) };
}

// The nominal law without barriers or rate limits, to fixed targets in turn (Fig. 7). One step per call.
export class NominalRun {
    constructor(plant, targets, constraint, { spanMap = null, k_t = 1.4, k_c = 3.0, xi = 0.04, dt = 0.02, hold = 6.0, filter = 'dls', q0 = null } = {}) {
        Object.assign(this, { plant, targets, constraint, spanMap, k_t, k_c, xi, dt, hold, filter });
        plant.reset();
        this.q = (q0 ?? plant.q0).slice();
        this.state = plant.evaluate(this.q);
        this.holdSteps = Math.round(hold / dt);
        this.k = 0; this.rows = []; this.shapes = [this.state.centerline];
    }

    get done() { return this.k >= this.holdSteps * this.targets.length; }

    step() {
        if (this.done) return null;
        const index = Math.floor(this.k / this.holdSteps), goal = this.targets[index];
        const s = this.state = this.plant.evaluate(this.q);
        if (this.k > 0 && this.k % this.holdSteps === 0) this.shapes.push(s.centerline);
        const exact = differential(this.plant, s.frames, s.p_D, s.spans, null, s.z);
        const model = this.spanMap ? differential(this.plant, s.frames, s.p_D, s.spans, this.spanMap) : exact;
        const e = sub(goal, s.p_D);
        const ec = this.constraint.error(s.frames), Jc = this.constraint.jacobian(s.frames, exact.JB);
        const Nc = projector(Jc);
        const qdotC = Jc.length ? matvec(pinv(Jc), scale(ec, this.k_c)) : vec(6);
        const A = matmul(model.Jt, Nc);
        const nu = spectralVelocity(A, sub(scale(e, this.k_t), matvec(model.Jt, qdotC)), this.xi, this.filter);
        const qdot = add(qdotC, matvec(Nc, nu));
        const d = descent(matmul(exact.Jt, Nc), e, this.k_t, this.xi, this.filter);
        const row = { t: this.k * this.dt, target: index, V: d.V, beta: d.beta, rates: d.rates, grad: d.grad,
            ec: this.constraint.deviation(s.frames), tip: s.p_D.slice(), qdot: Math.max(...qdot.map(Math.abs)) };
        this.rows.push(row);
        this.q = add(this.q, scale(qdot, this.dt));
        this.k += 1;
        return row;
    }
}
