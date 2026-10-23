// Small dense linear algebra for the simulation (matrices are arrays of rows), the
// QP solver of softext/qp.py, and the seeded generator of softext/rng.py.

export const vec = (n, v = 0) => new Array(n).fill(v);
export const zeros = (m, n) => Array.from({ length: m }, () => new Array(n).fill(0));
export const eye = (n) => Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, j) => (i === j ? 1 : 0)));
export const clone = (A) => A.map((row) => row.slice());

export const add = (a, b) => a.map((v, i) => v + b[i]);
export const sub = (a, b) => a.map((v, i) => v - b[i]);
export const scale = (a, s) => a.map((v) => v * s);
export const dot = (a, b) => { let s = 0; for (let i = 0; i < a.length; i++) s += a[i] * b[i]; return s; };
export const norm = (a) => Math.sqrt(dot(a, a));
export const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
export const skew = (x) => [[0, -x[2], x[1]], [x[2], 0, -x[0]], [-x[1], x[0], 0]];
export const outer = (a, b) => a.map((x) => b.map((y) => x * y));
export const col = (A, j) => A.map((row) => row[j]);

export function transpose(A) {
    if (!A.length) return [];
    return A[0].map((_, j) => A.map((row) => row[j]));
}
export function matmul(A, B) {
    const m = A.length, n = B.length ? B[0].length : 0, k = B.length;
    const C = zeros(m, n);
    for (let i = 0; i < m; i++) {
        const Ai = A[i], Ci = C[i];
        for (let p = 0; p < k; p++) {
            const a = Ai[p];
            if (a === 0) continue;
            const Bp = B[p];
            for (let j = 0; j < n; j++) Ci[j] += a * Bp[j];
        }
    }
    return C;
}
export const matvec = (A, x) => A.map((row) => dot(row, x));
export function vecmat(x, A) {                       // x^T A
    const n = A.length ? A[0].length : 0, out = vec(n);
    for (let i = 0; i < A.length; i++) { const xi = x[i]; if (xi) for (let j = 0; j < n; j++) out[j] += xi * A[i][j]; }
    return out;
}
export const madd = (A, B) => A.map((row, i) => row.map((v, j) => v + B[i][j]));
export const msub = (A, B) => A.map((row, i) => row.map((v, j) => v - B[i][j]));
export const mscale = (A, s) => A.map((row) => row.map((v) => v * s));

// Solve A X = B by Gaussian elimination with partial pivoting (B a vector or a matrix).
export function solve(A, B) {
    const n = A.length, isVec = !Array.isArray(B[0]);
    const M = clone(A), X = isVec ? B.map((v) => [v]) : clone(B), m = X[0] ? X[0].length : 0;
    for (let k = 0; k < n; k++) {
        let p = k, best = Math.abs(M[k][k]);
        for (let i = k + 1; i < n; i++) if (Math.abs(M[i][k]) > best) { best = Math.abs(M[i][k]); p = i; }
        if (p !== k) { [M[k], M[p]] = [M[p], M[k]]; [X[k], X[p]] = [X[p], X[k]]; }
        const piv = M[k][k];
        for (let i = k + 1; i < n; i++) {
            const f = M[i][k] / piv;
            if (f === 0) continue;
            for (let j = k; j < n; j++) M[i][j] -= f * M[k][j];
            for (let j = 0; j < m; j++) X[i][j] -= f * X[k][j];
        }
    }
    for (let k = n - 1; k >= 0; k--) {
        for (let j = 0; j < m; j++) {
            let s = X[k][j];
            for (let i = k + 1; i < n; i++) s -= M[k][i] * X[i][j];
            X[k][j] = s / M[k][k];
        }
    }
    return isVec ? X.map((r) => r[0]) : X;
}
export const inv = (A) => solve(A, eye(A.length));

export function cholesky(A) {
    const n = A.length, L = zeros(n, n);
    for (let j = 0; j < n; j++) {
        let s = A[j][j];
        for (let k = 0; k < j; k++) s -= L[j][k] * L[j][k];
        L[j][j] = Math.sqrt(s);
        for (let i = j + 1; i < n; i++) {
            let t = A[i][j];
            for (let k = 0; k < j; k++) t -= L[i][k] * L[j][k];
            L[i][j] = t / L[j][j];
        }
    }
    return L;
}
export function solveLower(L, b) {
    const x = vec(b.length);
    for (let i = 0; i < b.length; i++) { let s = b[i]; for (let k = 0; k < i; k++) s -= L[i][k] * x[k]; x[i] = s / L[i][i]; }
    return x;
}
export function solveUpperT(L, b) {                   // solve L^T x = b
    const n = b.length, x = vec(n);
    for (let i = n - 1; i >= 0; i--) { let s = b[i]; for (let k = i + 1; k < n; k++) s -= L[k][i] * x[k]; x[i] = s / L[i][i]; }
    return x;
}

// Symmetric eigendecomposition (cyclic Jacobi): ascending eigenvalues, eigenvectors as columns of U.
export function symEig(S) {
    const n = S.length, A = clone(S), U = eye(n);
    for (let sweep = 0; sweep < 60; sweep++) {
        let off = 0;
        for (let i = 0; i < n; i++) for (let j = i + 1; j < n; j++) off += A[i][j] * A[i][j];
        if (off < 1e-30) break;
        for (let p = 0; p < n; p++) for (let q = p + 1; q < n; q++) {
            if (Math.abs(A[p][q]) < 1e-300) continue;
            const theta = (A[q][q] - A[p][p]) / (2 * A[p][q]);
            const t = Math.sign(theta || 1) / (Math.abs(theta) + Math.sqrt(theta * theta + 1));
            const c = 1 / Math.sqrt(t * t + 1), s = t * c;
            for (let k = 0; k < n; k++) {
                const akp = A[k][p], akq = A[k][q];
                A[k][p] = c * akp - s * akq; A[k][q] = s * akp + c * akq;
            }
            for (let k = 0; k < n; k++) {
                const apk = A[p][k], aqk = A[q][k];
                A[p][k] = c * apk - s * aqk; A[q][k] = s * apk + c * aqk;
            }
            for (let k = 0; k < n; k++) {
                const ukp = U[k][p], ukq = U[k][q];
                U[k][p] = c * ukp - s * ukq; U[k][q] = s * ukp + c * ukq;
            }
        }
    }
    const order = A.map((_, i) => i).sort((a, b) => A[a][a] - A[b][b]);
    return { values: order.map((i) => A[i][i]), vectors: U.map((row) => order.map((i) => row[i])) };
}

// Singular values, descending.
export function singularValues(A) {
    if (!A.length || !A[0].length) return [];
    const G = A.length <= A[0].length ? matmul(A, transpose(A)) : matmul(transpose(A), A);
    return symEig(G).values.map((v) => Math.sqrt(Math.max(v, 0))).reverse();
}
export function rank(A, tol = 1e-6) {
    const s = singularValues(A);
    return s.length ? s.filter((v) => v > tol * Math.max(1, s[0])).length : 0;
}

// ---------------------------------------------------------------- QP (softext/qp.py)
export function householderLstsq(A, b) {
    const R = clone(A), y = b.slice(), m = R.length, n = m ? R[0].length : 0;
    for (let k = 0; k < n; k++) {
        const v = [];
        for (let i = k; i < m; i++) v.push(R[i][k]);
        const alpha = -Math.sign(v[0] !== 0 ? v[0] : 1) * norm(v);
        v[0] -= alpha;
        const vv = dot(v, v);
        if (vv < 1e-300) continue;
        for (let j = k; j < n; j++) {
            let s = 0;
            for (let i = k; i < m; i++) s += v[i - k] * R[i][j];
            const f = 2 * s / vv;
            for (let i = k; i < m; i++) R[i][j] -= f * v[i - k];
        }
        let s = 0;
        for (let i = k; i < m; i++) s += v[i - k] * y[i];
        const f = 2 * s / vv;
        for (let i = k; i < m; i++) y[i] -= f * v[i - k];
    }
    const x = vec(n);
    for (let k = n - 1; k >= 0; k--) {
        let s = y[k];
        for (let j = k + 1; j < n; j++) s -= R[k][j] * x[j];
        x[k] = s / R[k][k];
    }
    return x;
}

// min |E u - f| subject to u >= 0 (Lawson-Hanson).
export function nnls(E, f, tol = 1e-12) {
    const n = E[0].length;
    let u = vec(n);
    const passive = vec(n, false);
    const ET = transpose(E);
    for (let outer = 0; outer < 3 * n + 10; outer++) {
        const r = sub(f, matvec(E, u));
        const w = matvec(ET, r);
        let t = -1, best = tol;
        for (let j = 0; j < n; j++) if (!passive[j] && w[j] > best) { best = w[j]; t = j; }
        if (t < 0) break;
        passive[t] = true;
        for (;;) {
            const idx = [];
            for (let j = 0; j < n; j++) if (passive[j]) idx.push(j);
            const z = vec(n);
            if (idx.length) {
                const sol = householderLstsq(E.map((row) => idx.map((j) => row[j])), f);
                idx.forEach((j, k) => { z[j] = sol[k]; });
            }
            if (idx.every((j) => z[j] > tol)) { u = z; break; }
            let alpha = Infinity;
            for (const j of idx) if (z[j] <= tol) alpha = Math.min(alpha, u[j] / (u[j] - z[j]));
            u = u.map((v, j) => v + alpha * (z[j] - v));
            for (let j = 0; j < n; j++) { if (passive[j] && !(u[j] > tol)) passive[j] = false; if (!passive[j]) u[j] = 0; }
        }
    }
    return u;
}

// min |y| subject to G y >= h; null if infeasible.
export function ldp(G, h) {
    const n = G[0].length;
    const E = [...transpose(G), h.slice()];
    const f = vec(n + 1); f[n] = 1;
    const u = nnls(E, f);
    const r = sub(matvec(E, u), f);
    if (norm(r) < 1e-10 || Math.abs(r[n]) < 1e-12) return null;
    return r.slice(0, n).map((v) => -v / r[n]);
}

// min 1/2 x^T H x + g^T x subject to A x >= b; null if infeasible.
export function solveQP(H, g, A, b) {
    const L = cholesky(H);
    const xFree = scale(solve(H, g), -1);
    if (!b.length) return xFree;
    const G = A.map((row) => solveLower(L, row));            // rows of A L^-T
    const y = ldp(G, sub(b, matvec(A, xFree)));
    if (!y) return null;
    return add(xFree, solveUpperT(L, y));
}

// ---------------------------------------------------------------- random numbers (softext/rng.py)
export class Random {
    constructor(seed = 0) { this.state = seed >>> 0; }
    uniform() {
        const a = this.state = (this.state + 0x6D2B79F5) >>> 0;
        let t = Math.imul(a ^ (a >>> 15), 1 | a) >>> 0;
        t = ((t + (Math.imul(t ^ (t >>> 7), 61 | t) >>> 0)) >>> 0) ^ t;
        return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    }
    normal() {
        const u1 = 1 - this.uniform(), u2 = this.uniform();
        return Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u2);
    }
    normals(n) { return Array.from({ length: n }, () => this.normal()); }
}
