// The simulated plant and the differential model of Sec. III: a port of softext/config.py,
// arm.py, rod.py, plant.py, model.py, and identification.py. tests/test_web_parity.py runs
// it with Node.js and compares it with the Python package.
import {
    add, clone, col, cross, dot, eye, matmul, matvec, norm, outer, Random, scale, skew, solve, sub, transpose, vec, vecmat, zeros,
} from './linalg.js';

const DEG = Math.PI / 180;

// ---------------------------------------------------------------- configuration (config.yaml)
export const DEFAULTS = {
    arm: {
        a: [0.0, -0.612, -0.5723, 0.0, 0.0, 0.0],
        alpha_deg: [90.0, 0.0, 0.0, 90.0, -90.0, 0.0],
        d: [0.1273, 0.0, 0.0, 0.163941, 0.1157, 0.0922],
        joint_limits_deg: [[-180, 180], [-180, 10], [-160, 160], [-180, 180], [-180, 180], [-180, 180]],
        q0_deg: [0.0, -70.0, 95.0, -115.0, -90.0, 0.0],
    },
    rod: { length: 0.320, radius: 0.009, youngs_modulus: 1.1e6, shear_modulus: 4.0e5, sections: 5, steps_per_section: 5 },
    tendons: [
        { name: 'tendon 1', angle_deg: 0.0, routing_radius: 0.0055, stiffness: 260.0, max_tension: 14.0, anchor_link: 4, anchor_fraction: 0.75 },
        { name: 'tendon 2', angle_deg: 90.0, routing_radius: 0.0055, stiffness: 260.0, max_tension: 14.0, anchor_link: 3, anchor_fraction: 0.60 },
    ],
    home_tension: 1.5,
    sensing: { plant_rate: 100.0, rate: 20.0, noise: 0.0005, body_points: 5, seed: 0 },
    identification: { grid: 11, range: 0.010, degree: 4, broyden_gain: 0.4, broyden_decay: 0.995, broyden_min_step: 0.0002 },
    control: { rate: 50.0, k_t: 1.4, k_c: 3.0, xi: 0.04, mu: 0.04, w_c: 200.0, qdot_max: 0.30, reference_speed: 0.10 },
    targets: { tolerance: 0.004, hold: 8.0, stall: 10.0, stall_progress: 0.0001, hysteresis: 0.0015, budget: 90.0 },
    barriers: {
        obstacle: { alpha: 3.0, eps: 0.008, offset: 0.012, smoothing: 0.004 },
        span: { alpha: 4.0, eps: 0.0005 },
        cone: { alpha: 3.0, eps_deg: 2.0, half_angle_deg: 35.0 },
        insertion: { alpha: 3.0, eps: 0.005, min: 0.05, max: 0.26 },
        aperture: { alpha: 3.0, eps: 0.002 },
    },
};

export const defaultConfig = () => JSON.parse(JSON.stringify(DEFAULTS));
export const q0Of = (config) => config.arm.q0_deg.map((v) => v * DEG);
export const limitsOf = (config) => config.arm.joint_limits_deg.map(([lo, hi]) => [lo * DEG, hi * DEG]);
export function tendonsOf(config) {
    return config.tendons.map((t) => {
        const angle = t.angle_deg * DEG;
        return { name: t.name, angle, radius: t.routing_radius, k: t.stiffness, tauMax: t.max_tension, link: t.anchor_link,
            fraction: t.anchor_fraction, entry: [t.routing_radius * Math.cos(angle), t.routing_radius * Math.sin(angle), 0] };
    });
}

// ---------------------------------------------------------------- arm (arm.py)
// A frame is {R, p}: rotation (rows) and position in the world.
function dhTransform(a, alpha, d, theta) {
    const ct = Math.cos(theta), st = Math.sin(theta), ca = Math.cos(alpha), sa = Math.sin(alpha);
    return { R: [[ct, -st * ca, st * sa], [st, ct * ca, -ct * sa], [0, sa, ca]], p: [a * ct, a * st, d] };
}
const compose = (A, B) => ({ R: matmul(A.R, B.R), p: add(A.p, matvec(A.R, B.p)) });

export class Arm {
    constructor(config) {
        this.a = config.arm.a.slice();
        this.alpha = config.arm.alpha_deg.map((v) => v * DEG);
        this.d = config.arm.d.slice();
        this.limits = limitsOf(config);
    }

    frames(q) {
        const out = [{ R: eye(3), p: [0, 0, 0] }];
        for (let i = 0; i < 6; i++) out.push(compose(out[i], dhTransform(this.a[i], this.alpha[i], this.d[i], q[i])));
        return out;
    }

    static originJacobian(frames, k) {
        const J = zeros(3, 6), p = frames[k].p;
        for (let j = 1; j <= k; j++) {
            const c = cross(col(frames[j - 1].R, 2), sub(p, frames[j - 1].p));
            for (let i = 0; i < 3; i++) J[i][j - 1] = c[i];
        }
        return J;
    }

    static pointOnLink(frames, link, fraction) {
        return add(scale(frames[link - 1].p, 1 - fraction), scale(frames[link].p, fraction));
    }

    static pointJacobian(frames, link, fraction) {
        const A = Arm.originJacobian(frames, link - 1), B = Arm.originJacobian(frames, link);
        return A.map((row, i) => row.map((v, j) => (1 - fraction) * v + fraction * B[i][j]));
    }

    static geometricJacobian(frames) {
        const Jv = Arm.originJacobian(frames, 6);
        const Jw = [0, 1, 2].map((i) => [0, 1, 2, 3, 4, 5].map((j) => frames[j].R[i][2]));
        return [...Jv, ...Jw];
    }
}

// ---------------------------------------------------------------- rod (rod.py)
const E_Z = [0, 0, 1];

export function rotationLog(R) {
    const w = [0.5 * (R[2][1] - R[1][2]), 0.5 * (R[0][2] - R[2][0]), 0.5 * (R[1][0] - R[0][1])];
    const s = norm(w), c = 0.5 * (R[0][0] + R[1][1] + R[2][2] - 1);
    const angle = Math.atan2(s, c);
    return s < 1e-12 ? w : scale(w, angle / s);
}

function sectionIncrement(u, ds) {
    const k = norm(u), U = skew(u);
    if (k < 1e-10) return { R: eye(3).map((row, i) => row.map((v, j) => v + U[i][j] * ds)), p: [0, 0, ds] };
    const U2 = matmul(U, U), angle = k * ds;
    const c1 = Math.sin(angle) / k, c2 = (1 - Math.cos(angle)) / (k * k), c3 = (angle - Math.sin(angle)) / (k * k * k);
    const R = eye(3).map((row, i) => row.map((v, j) => v + c1 * U[i][j] + c2 * U2[i][j]));
    const V = eye(3).map((row, i) => row.map((v, j) => v * ds + c2 * U[i][j] + c3 * U2[i][j]));
    return { R, p: col(V, 2) };
}

export class Rod {
    constructor(config) {
        const rod = config.rod;
        this.L = rod.length;
        this.S = rod.sections;
        this.steps = rod.steps_per_section;
        this.ls = this.L / this.S;
        this.ds = this.ls / this.steps;
        const I = Math.PI * rod.radius ** 4 / 4;
        this.K = [rod.youngs_modulus * I, rod.youngs_modulus * I, rod.shear_modulus * 2 * I];
        const tendons = tendonsOf(config);
        this.m = tendons.length;
        this.r = tendons.map((t) => t.entry);
        this.k = tendons.map((t) => t.k);
        this.tauMax = tendons.map((t) => t.tauMax);
        this.rest = vec(this.m, this.L);
    }

    section(z, s) { return [z[3 * s], z[3 * s + 1], z[3 * s + 2]]; }

    interiorLengths(z) {
        return this.r.map((r) => {
            let l = 0;
            for (let s = 0; s < this.S; s++) l += this.ls * norm(add(E_Z, cross(this.section(z, s), r)));
            return l;
        });
    }

    interiorGradient(z) {                       // J_lz, m x 3S
        return this.r.map((r) => {
            const row = [];
            for (let s = 0; s < this.S; s++) {
                const t = add(E_Z, cross(this.section(z, s), r));
                row.push(...scale(cross(r, scale(t, 1 / norm(t))), this.ls));
            }
            return row;
        });
    }

    stretch(z, spans) {
        const lin = this.interiorLengths(z);
        const e = spans.map((s, i) => s + lin[i] - this.rest[i]);
        const tau = e.map((v, i) => Math.min(Math.max(this.k[i] * v, 0), this.tauMax[i]));
        const active = e.map((v, i) => v > 0 && this.k[i] * v < this.tauMax[i]);
        return { e, tau, active };
    }

    energy(z, spans) {
        let elastic = 0;
        for (let i = 0; i < z.length; i++) elastic += z[i] * z[i] * this.K[i % 3];
        elastic *= 0.5 * this.ls;
        const { e } = this.stretch(z, spans);
        let cable = 0;
        e.forEach((ei, i) => {
            if (ei <= 0) return;
            const k = this.k[i], t = this.tauMax[i];
            cable += k * ei <= t ? 0.5 * k * ei * ei : t * ei - 0.5 * t * t / k;
        });
        return elastic + cable;
    }

    gradient(z, spans) {
        const { tau } = this.stretch(z, spans);
        const g = z.map((v, i) => this.ls * v * this.K[i % 3]);
        const J = this.interiorGradient(z);
        J.forEach((row, i) => { for (let j = 0; j < g.length; j++) g[j] += tau[i] * row[j]; });
        return g;
    }

    loadHessian(z, tau) {                       // H_0
        const n = 3 * this.S, H = zeros(n, n);
        for (let i = 0; i < n; i++) H[i][i] = this.ls * this.K[i % 3];
        this.r.forEach((r, i) => {
            if (tau[i] === 0) return;
            const Rx = skew(r);
            for (let s = 0; s < this.S; s++) {
                const t = add(E_Z, cross(this.section(z, s), r)), nt = norm(t), th = scale(t, 1 / nt);
                const P = eye(3).map((row, a) => row.map((v, b) => v - th[a] * th[b]));
                const B = matmul(matmul(transpose(Rx), P), Rx);
                for (let a = 0; a < 3; a++) for (let b = 0; b < 3; b++) H[3 * s + a][3 * s + b] += tau[i] * this.ls * B[a][b] / nt;
            }
        });
        return H;
    }

    hessian(z, spans) {
        const { tau, active } = this.stretch(z, spans);
        const H = this.loadHessian(z, tau), J = this.interiorGradient(z);
        J.forEach((row, i) => {
            if (!active[i]) return;
            for (let a = 0; a < row.length; a++) for (let b = 0; b < row.length; b++) H[a][b] += this.k[i] * row[a] * row[b];
        });
        return H;
    }

    solve(spans, z0 = null, tol = 1e-13, maxIter = 60) {
        let z = z0 ? z0.slice() : vec(3 * this.S);
        for (let it = 0; it < maxIter; it++) {
            const g = this.gradient(z, spans);
            if (norm(g) < tol) return { z, iterations: it };
            const step = solve(this.hessian(z, spans), g);
            const slope = dot(g, step);
            let t = 1;
            if (slope > 1e-14) {
                const u0 = this.energy(z, spans);
                while (t > 1e-6 && this.energy(sub(z, scale(step, t)), spans) > u0 - 1e-4 * t * slope) t *= 0.5;
            }
            z = sub(z, scale(step, t));
        }
        return { z, iterations: maxIter };
    }

    integrate(z, base = null) {
        let R = base ? clone(base.R) : eye(3), p = base ? base.p.slice() : [0, 0, 0];
        const points = [p.slice()], rotations = [clone(R)];
        for (let s = 0; s < this.S; s++) {
            const inc = sectionIncrement(this.section(z, s), this.ds);
            for (let k = 0; k < this.steps; k++) {
                p = add(p, matvec(R, inc.p));
                R = matmul(R, inc.R);
                points.push(p.slice());
                rotations.push(clone(R));
            }
        }
        return { points, rotations };
    }

    tipShapeJacobian(z, h = 1e-6) {             // J_Dz, 6 x 3S, in {B}
        const J = zeros(6, z.length);
        for (let j = 0; j < z.length; j++) {
            const zp = z.slice(), zm = z.slice();
            zp[j] += h; zm[j] -= h;
            const a = this.integrate(zp), b = this.integrate(zm);
            const dp = sub(a.points[a.points.length - 1], b.points[b.points.length - 1]);
            const w = rotationLog(matmul(a.rotations[a.rotations.length - 1], transpose(b.rotations[b.rotations.length - 1])));
            for (let i = 0; i < 3; i++) { J[i][j] = dp[i] / (2 * h); J[3 + i][j] = w[i] / (2 * h); }
        }
        return J;
    }
}

// ---------------------------------------------------------------- plant (plant.py)
export class Plant {
    constructor(config, calibrationIterations = 8, initialExcess = 0.055) {
        this.config = config;
        this.arm = new Arm(config);
        this.tendons = tendonsOf(config);
        this.rod = new Rod(config);
        const samples = this.rod.S * this.rod.steps + 1, k = config.sensing.body_points;
        this.bodyIndex = [...new Set(Array.from({ length: k }, (_, i) => Math.trunc(i * (samples - 1) / (k - 1))))];
        this.q0 = q0Of(config);
        this.rod.rest = vec(this.rod.m, this.rod.L + initialExcess);
        for (let it = 0; it < calibrationIterations; it++) {
            const spans = this.spans(this.q0);
            const { z } = this.rod.solve(spans);
            const lin = this.rod.interiorLengths(z);
            this.rod.rest = spans.map((s, i) => this.rod.L + Math.max(s + lin[i] - this.rod.L - config.home_tension / this.rod.k[i], 0));
        }
        this.z = null;
    }

    endpoints(frames) {
        const B = frames[6];
        return {
            anchors: this.tendons.map((t) => Arm.pointOnLink(frames, t.link, t.fraction)),
            entries: this.tendons.map((t) => add(B.p, matvec(B.R, t.entry))),
        };
    }

    spans(q) {
        const { anchors, entries } = this.endpoints(this.arm.frames(q));
        return anchors.map((a, i) => norm(sub(a, entries[i])));
    }

    evaluate(q, warm = true) {
        q = q.slice();
        const frames = this.arm.frames(q);
        const { anchors, entries } = this.endpoints(frames);
        const spans = anchors.map((a, i) => norm(sub(a, entries[i])));
        const { z } = this.rod.solve(spans, warm ? this.z : null);
        this.z = z;
        const { tau, active } = this.rod.stretch(z, spans);
        const { points, rotations } = this.rod.integrate(z);
        const B = frames[6];
        const centerline = points.map((p) => add(B.p, matvec(B.R, p)));
        return {
            q, frames, anchors, entries, spans, tensions: tau, active, z, local: points, rotations, centerline,
            R_B: B.R, p_B: B.p, p_D: centerline[centerline.length - 1],
            R_D: matmul(B.R, rotations[rotations.length - 1]),
            bodyPoints: this.bodyIndex.map((i) => centerline[i]),
        };
    }

    reset() { this.z = null; }
}

// ---------------------------------------------------------------- differential model (model.py)
export function spanJacobian(plant, frames) {
    const JB = Arm.geometricJacobian(frames), R_B = frames[6].R;
    const { anchors, entries } = plant.endpoints(frames);
    return plant.tendons.map((t, i) => {
        const diff = sub(anchors[i], entries[i]), d = scale(diff, 1 / norm(diff));
        const Ja = Arm.pointJacobian(frames, t.link, t.fraction);
        const S = matmul(skew(matvec(R_B, t.entry)), JB.slice(3));
        return vecmat(d, Ja.map((row, a) => row.map((v, j) => v - JB[a][j] + S[a][j])));
    });
}

export function transportJacobian(JB, pB, pD) {
    const S = matmul(skew(sub(pD, pB)), JB.slice(3));
    return [...JB.slice(0, 3).map((row, i) => row.map((v, j) => v - S[i][j])), ...JB.slice(3).map((row) => row.slice())];
}

export function distalJacobian(Jtr, RB, Jspan, Jlq) {
    const D = matmul(Jspan, Jlq);                // 6 x 6
    const top = matmul(RB, D.slice(0, 3)), bottom = matmul(RB, D.slice(3));
    return Jtr.map((row, i) => row.map((v, j) => v + (i < 3 ? top[i][j] : bottom[i - 3][j])));
}

export function softResponse(rod, z, spans) {
    const { tau, active } = rod.stretch(z, spans);
    const Jlz = rod.interiorGradient(z);
    const Jztau = solve(rod.loadHessian(z, tau), transpose(Jlz)).map((row) => row.map((v) => -v));
    const JDtau = matmul(rod.tipShapeJacobian(z), Jztau);
    const Ka = active.map((a, i) => (a ? rod.k[i] : 0));
    const KJ = matmul(Jlz, Jztau).map((row, i) => row.map((v) => Ka[i] * v));
    const M = eye(rod.m).map((row, i) => row.map((v, j) => v - KJ[i][j]));
    const Ctau = solve(M, eye(rod.m).map((row, i) => row.map((v) => v * Ka[i])));
    return { Jlz, Jztau, JDtau, Ka, Ctau, Jspan: matmul(JDtau, Ctau) };
}

// The model of the controller at the measured tip: identified (spanMap) or exact (z, simulation only).
export function differential(plant, frames, tip, spans, spanMap = null, z = null) {
    const JB = Arm.geometricJacobian(frames), R_B = frames[6].R, p_B = frames[6].p;
    const Jlq = spanJacobian(plant, frames);
    const Jtr = transportJacobian(JB, p_B, tip);
    let Jspan, Jp = null;
    if (!spanMap) {
        Jspan = softResponse(plant.rod, z, spans).Jspan;
    } else {
        Jspan = [...spanMap.tipJacobian(spans), ...zeros(3, spans.length)];
        if (spanMap.shape) {
            const local = spanMap.predictShape(spans), sens = spanMap.shapeJacobian(spans);
            const Jw = JB.slice(3), RJ = (s) => matmul(matmul(R_B, s), Jlq);
            Jp = local.map((lp, k) => {
                const S = matmul(skew(matvec(R_B, lp)), Jw), D = RJ(sens[k]);
                return JB.slice(0, 3).map((row, i) => row.map((v, j) => v - S[i][j] + D[i][j]));
            });
        }
    }
    const JD = distalJacobian(Jtr, R_B, Jspan, Jlq);
    return { JB, Jlq, Jtr, Jspan, JD, Jt: JD.slice(0, 3), Jp };
}

// ---------------------------------------------------------------- identified model (identification.py)
export class SpanMap {
    constructor(degree = 4) {
        this.degree = degree;
        this.centre = this.scale = this.coef = this.shape = null;
        this.shapePoints = 0;
        this.lower = this.upper = null;
        this.residual = null;
        this.gain = 0.4; this.decay = 0.995; this.minStep = 2e-4;
    }

    exponents(m) {
        const out = [];
        const rec = (prefix) => {
            if (prefix.length === m) { if (prefix.reduce((a, b) => a + b, 0) <= this.degree) out.push(prefix); return; }
            for (let p = 0; p <= this.degree; p++) rec([...prefix, p]);
        };
        rec([]);
        return out.sort((a, b) => {
            const sa = a.reduce((x, y) => x + y, 0), sb = b.reduce((x, y) => x + y, 0);
            if (sa !== sb) return sa - sb;
            for (let i = 0; i < m; i++) if (a[i] !== b[i]) return a[i] - b[i];
            return 0;
        });
    }

    features(spans) {
        const x = spans.map((s, i) => (s - this.centre[i]) / this.scale[i]);
        return this.terms(x.length).map((p) => p.reduce((acc, power, i) => acc * x[i] ** power, 1));
    }

    terms(m) { if (!this._terms || this._terms.m !== m) this._terms = Object.assign(this.exponents(m), { m }); return this._terms; }

    featureGradient(spans) {
        const x = spans.map((s, i) => (s - this.centre[i]) / this.scale[i]);
        return this.terms(x.length).map((p) => p.map((power, i) => {
            if (!power) return 0;
            let others = 1;
            for (let j = 0; j < x.length; j++) if (j !== i) others *= x[j] ** p[j];
            return power * x[i] ** (power - 1) * others / this.scale[i];
        }));
    }

    predictTip(spans) { return vecmat(this.features(spans), this.coef); }

    tipJacobian(spans) {                         // 3 x m, with the online correction
        const base = matmul(transpose(this.coef), this.featureGradient(spans));
        return base.map((row, i) => row.map((v, j) => v + this.residual[i][j]));
    }

    predictShape(spans) {
        const flat = vecmat(this.features(spans), this.shape);
        return Array.from({ length: this.shapePoints }, (_, k) => flat.slice(3 * k, 3 * k + 3));
    }

    shapeJacobian(spans) {                       // (k, 3, m)
        const flat = matmul(transpose(this.shape), this.featureGradient(spans));
        return Array.from({ length: this.shapePoints }, (_, k) => flat.slice(3 * k, 3 * k + 3));
    }

    update(spans, previous, stepTipLocal) {
        const step = sub(spans, previous);
        if (norm(step) < this.minStep) { this.residual = this.residual.map((row) => row.map((v) => this.decay * v)); return; }
        const predicted = matvec(this.tipJacobian(spans), step);
        const c = outer(sub(stepTipLocal, predicted), step), ss = dot(step, step);
        this.residual = this.residual.map((row, i) => row.map((v, j) => v + this.gain * c[i][j] / ss));
    }

    reset() { this.residual = zeros(3, this.centre.length); }

    fit(spans, tips, shapes = null, regularization = 1e-6) {
        const n = spans.length, m = spans[0].length;
        this.centre = Array.from({ length: m }, (_, i) => spans.reduce((a, s) => a + s[i], 0) / n);
        this.scale = Array.from({ length: m }, (_, i) => Math.max(Math.sqrt(spans.reduce((a, s) => a + (s[i] - this.centre[i]) ** 2, 0) / n), 1e-6));
        this.lower = Array.from({ length: m }, (_, i) => Math.min(...spans.map((s) => s[i])));
        this.upper = Array.from({ length: m }, (_, i) => Math.max(...spans.map((s) => s[i])));
        const F = spans.map((s) => this.features(s)), FT = transpose(F);
        const gram = matmul(FT, F).map((row, i) => row.map((v, j) => v + (i === j ? regularization : 0)));
        this.coef = solve(gram, matmul(FT, tips));
        if (shapes) {
            this.shapePoints = shapes[0].length;
            this.shape = solve(gram, matmul(FT, shapes.map((pts) => pts.flat())));
        }
        this.reset();
        const err = F.map((f, k) => sub(vecmat(f, this.coef), tips[k]));
        return { samples: n, rmsTip: Math.sqrt(err.reduce((a, e) => a + dot(e, e), 0) / n) };
    }

    static fromData(data, config = null) {
        const out = new SpanMap(data.degree);
        out.centre = data.centre.slice(); out.scale = data.scale.slice();
        out.coef = data.coefficients.map((row) => row.slice(0, 3));
        if (data.shape_coefficients) { out.shape = data.shape_coefficients.map((row) => row.slice()); out.shapePoints = data.shape_points; }
        out.lower = data.span_min_m.slice(); out.upper = data.span_max_m.slice();
        if (config) { const c = config.identification; out.gain = c.broyden_gain; out.decay = c.broyden_decay; out.minStep = c.broyden_min_step; }
        out.reset();
        return out;
    }

    copy() {
        const out = Object.assign(new SpanMap(this.degree), this);
        out.residual = this.residual.map((row) => row.slice());
        out._terms = null;
        return out;
    }
}

export function jointsForSpans(plant, wanted, seed, iterations = 60, tolerance = 5e-5) {
    const limits = plant.arm.limits;
    let q = seed.slice();
    for (let it = 0; it < iterations; it++) {
        const err = sub(wanted, plant.spans(q));
        if (norm(err) < 1e-9) break;
        const J = spanJacobian(plant, plant.arm.frames(q));
        const step = vecmat(solve(matmul(J, transpose(J)), err), J);
        q = q.map((v, j) => Math.min(Math.max(v + step[j], limits[j][0]), limits[j][1]));
    }
    return norm(sub(wanted, plant.spans(q))) < tolerance ? q : null;
}

// The probing motions: a grid x grid box of spans around q0, measured with camera noise.
export function probe(plant, { grid, range, noise, seed } = {}) {
    const ident = plant.config.identification, sensing = plant.config.sensing;
    grid = grid ?? ident.grid; range = range ?? ident.range; noise = noise ?? sensing.noise;
    const rng = new Random(seed ?? sensing.seed);
    const centre = plant.spans(plant.q0);
    const axis = (c) => Array.from({ length: grid }, (_, i) => c - range + 2 * range * i / (grid - 1));
    const a0 = axis(centre[0]), a1 = axis(centre[1]);
    const out = { spans: [], tips: [], shapes: [], joints: [], wanted: [], reached: [], skipped: 0 };
    for (let i = 0; i < grid; i++) for (let j = 0; j < grid; j++) {         // meshgrid order: a1 rows, a0 columns
        const wanted = [a0[j], a1[i]];
        out.wanted.push(wanted);
        const q = jointsForSpans(plant, wanted, plant.q0);
        out.reached.push(!!q);
        if (!q) { out.skipped++; continue; }
        const s = plant.evaluate(q, false);
        const points = s.bodyPoints.map((p) => add(p, scale(rng.normals(3), noise)));
        const tip = add(s.p_D, scale(rng.normals(3), noise));
        const RT = transpose(s.R_B);
        out.spans.push(s.spans);
        out.tips.push(matvec(RT, sub(tip, s.p_B)));
        out.shapes.push(points.map((p) => matvec(RT, sub(p, s.p_B))));
        out.joints.push(q);
    }
    return out;
}

export function identify(plant, options = {}) {
    const data = probe(plant, options);
    const map = new SpanMap(plant.config.identification.degree);
    const report = map.fit(data.spans, data.tips, data.shapes);
    const c = plant.config.identification;
    map.gain = c.broyden_gain; map.decay = c.broyden_decay; map.minStep = c.broyden_min_step;
    return { map, data, report };
}

export function validationError(map, plant, samples = 160, seed = 99) {
    const rng = new Random(seed);
    let sum = 0, count = 0;
    for (let k = 0; k < samples; k++) {
        const spans = map.lower.map((lo, i) => lo + (map.upper[i] - lo) * rng.uniform());
        if (!jointsForSpans(plant, spans, plant.q0)) continue;
        const { z } = plant.rod.solve(spans);
        const { points } = plant.rod.integrate(z);
        const e = sub(map.predictTip(spans), points[points.length - 1]);
        sum += dot(e, e); count++;
    }
    return { rms: Math.sqrt(sum / Math.max(count, 1)), count };
}
