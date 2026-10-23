// Overview page: previews of the three interactive pages.
import { initPage, loadData } from '../lib/shell.js';
import { alpha, onThemeChange, palette } from '../lib/theme.js';
import { EEConstraint, NominalRun } from '../sim/control.js';
import { defaultConfig, Plant } from '../sim/model.js';

await initPage();
const data = await loadData();

// A canvas that redraws on resize and on a theme change.
function preview(canvas, draw) {
    const paint = () => {
        const dpr = Math.min(window.devicePixelRatio || 1, 2), w = canvas.clientWidth, h = canvas.clientHeight;
        if (!w || !h) return;
        canvas.width = Math.round(w * dpr); canvas.height = Math.round(h * dpr);
        const ctx = canvas.getContext('2d');
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
        ctx.fillStyle = palette().canvas; ctx.fillRect(0, 0, w, h);
        ctx.lineJoin = 'round'; ctx.lineCap = 'round';
        draw(ctx, w, h, palette());
    };
    new ResizeObserver(paint).observe(canvas);
    onThemeChange(paint);
}

// Fit a set of 2D points into the canvas with a margin; returns the mapping.
function fit(points, w, h, margin = 14) {
    const xs = points.map((p) => p[0]), ys = points.map((p) => p[1]);
    const x0 = Math.min(...xs), x1 = Math.max(...xs), y0 = Math.min(...ys), y1 = Math.max(...ys);
    const s = Math.min((w - 2 * margin) / (x1 - x0 || 1), (h - 2 * margin) / (y1 - y0 || 1));
    const ox = (w - s * (x1 - x0)) / 2, oy = (h - s * (y1 - y0)) / 2;
    return ([x, y]) => [ox + s * (x - x0), h - (oy + s * (y - y0))];
}
const line = (ctx, pts, color, width, dash = []) => {
    ctx.strokeStyle = color; ctx.lineWidth = width; ctx.setLineDash(dash);
    ctx.beginPath(); pts.forEach(([x, y], i) => (i ? ctx.lineTo(x, y) : ctx.moveTo(x, y))); ctx.stroke(); ctx.setLineDash([]);
};
const dot = (ctx, [x, y], r, color) => { ctx.fillStyle = color; ctx.beginPath(); ctx.arc(x, y, r, 0, 2 * Math.PI); ctx.fill(); };
const font = () => getComputedStyle(document.body).fontFamily;

// 1: the arm at q0 and after null-space motion of the position-constrained EE (side view)
const config = defaultConfig();
const plant = new Plant(config);
const home = plant.evaluate(plant.q0, false);
const run = new NominalRun(new Plant(config), [data.scenarios.S1.targets[0]], new EEConstraint('position', { pStar: home.p_B }), { hold: 4 });
while (!run.done) run.step();
const moved = run.plant.evaluate(run.q);
const side = (p) => [-p[0] + 0.25 * p[1], p[2] - 0.12 * p[1]];
preview(document.getElementById('preview1'), (ctx, w, h, p) => {
    const states = [home, moved];
    const all = states.flatMap((s) => [...s.frames.map((f) => side(f.p)), ...s.centerline.map(side), [side([0, 0, -0.05])[0], -0.05]]);
    const X = fit(all, w, h, 12);
    states.forEach((s, k) => {
        const faded = k === 0;
        ctx.globalAlpha = faded ? 0.35 : 1;
        line(ctx, s.frames.map((f) => X(side(f.p))), p.arm, 7);
        s.frames.slice(1).forEach((f) => dot(ctx, X(side(f.p)), 4.2, p.joint));
        [p.tendon1, p.tendon2].forEach((c, i) => line(ctx, [X(side(s.anchors[i])), X(side(s.entries[i]))], c, 1.6));
        line(ctx, s.centerline.map((q) => X(side(q))), p.rod, 4);
        dot(ctx, X(side(s.p_D)), 3.4, p.path);
    });
    ctx.globalAlpha = 1;
    line(ctx, [X(side([0.1, 0, -0.03])), X(side([-0.2, 0, -0.03]))], p.joint, 10);
    ctx.fillStyle = p.muted; ctx.font = `600 10px ${font()}`;
    ctx.fillText('q0 (faded) and after null-space motion, EE position held', 12, 16);
});

// 2: S2 under the fulcrum, from a run of the Python package
const s2 = data.scenarios.S2, pre = data.preview;
const axo = (q) => [q[0] * 0.82 - q[1] * 0.57, q[2] + 0.35 * (q[0] * 0.57 + q[1] * 0.82)];
preview(document.getElementById('preview2'), (ctx, w, h, p) => {
    const X = fit([...pre.s2_path.map(axo), ...pre.s2_shapes.flat().map(axo), ...s2.targets.map(axo), axo(s2.fulcrum)], w, h, 16);
    pre.s2_shapes.forEach((shape, k) => {
        ctx.globalAlpha = k === pre.s2_shapes.length - 1 ? 1 : 0.4;
        line(ctx, shape.map((q) => X(axo(q))), p.rod, k === pre.s2_shapes.length - 1 ? 4 : 2);
    });
    ctx.globalAlpha = 1;
    const o = s2.obstacles[0], c = X(axo(o.center)), edge = X(axo([o.center[0] + o.size, o.center[1], o.center[2]]));
    ctx.fillStyle = alpha(p.obstacle, 0.25); ctx.beginPath(); ctx.arc(c[0], c[1], Math.hypot(edge[0] - c[0], edge[1] - c[1]), 0, 2 * Math.PI); ctx.fill();
    line(ctx, pre.s2_path.map((q) => X(axo(q))), p.path, 1.6);
    ctx.font = `700 10px ${font()}`;
    s2.targets.forEach((t, k) => { const q = X(axo(t)); dot(ctx, q, 3.2, p.target); ctx.fillStyle = p.inkSoft; ctx.fillText(`T${k + 1}`, q[0] + 5, q[1] - 4); });
    const f = X(axo(s2.fulcrum));
    line(ctx, [[f[0] - 4, f[1] - 4], [f[0] + 4, f[1] + 4]], p.fulcrum, 2); line(ctx, [[f[0] - 4, f[1] + 4], [f[0] + 4, f[1] - 4]], p.fulcrum, 2);
    ctx.fillStyle = p.muted; ctx.font = `600 10px ${font()}`;
    ctx.fillText('S2: tip path, soft extension at each target, fulcrum c', 12, 16);
});

// 3: V on a log axis for the nominal control law (Fig. 7)
preview(document.getElementById('preview3'), (ctx, w, h, p) => {
    const t = pre.nominal_t, V = pre.nominal_V, T = t[t.length - 1];
    const L = 30, R = w - 12, top = 24, B = h - 16;
    const X = (x) => L + (R - L) * x / T, Y = (v) => B - (B - top) * (Math.log10(v) + 11) / 9;
    ctx.strokeStyle = p.grid; ctx.lineWidth = 1;
    for (const e of [-10, -8, -6, -4, -2]) { ctx.beginPath(); ctx.moveTo(L, Y(10 ** e)); ctx.lineTo(R, Y(10 ** e)); ctx.stroke(); }
    ctx.fillStyle = p.muted; ctx.font = `10px ${font()}`; ctx.textAlign = 'right';
    for (const e of [-10, -6, -2]) ctx.fillText(`1e${e}`, L - 4, Y(10 ** e) + 3);
    ctx.textAlign = 'left';
    const hold = 6, beta = Math.min(...pre.nominal_beta);
    for (let k = 0; k < 5; k++) {
        const idx = t.map((x, i) => i).filter((i) => t[i] >= k * hold && t[i] < (k + 1) * hold);
        const v0 = V[idx[0]];
        line(ctx, idx.map((i) => [X(t[i]), Y(v0 * Math.exp(-2 * pre.k_t * beta * (t[i] - t[idx[0]])))]), p.ink, 1, [1, 3]);
        line(ctx, idx.map((i) => [X(t[i]), Y(Math.max(V[i], 1e-11))]), p.exact, 2);
    }
    ctx.fillStyle = p.muted; ctx.font = `600 10px ${font()}`;
    ctx.fillText('V(e) under the nominal control law, five targets, bound (31)', 12, 14);
});

