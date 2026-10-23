// Line plots on a canvas: series, filled bands between two curves, shaded time
// intervals, reference lines, and markers.
import { alpha, onThemeChange, palette } from './theme.js';

const PAD = { l: 50, r: 12, t: 10, b: 30 };

export class LinePlot {
    // opts: {xLabel, yLabel, x: [min, max] | null (auto), y: [min, max] | null, logY, yIncludeZero}
    constructor(canvas, opts = {}) {
        this.canvas = canvas;
        this.opts = { x: null, y: null, logY: false, yIncludeZero: true, ...opts };
        this.series = []; this.fills = []; this.spans = []; this.hlines = []; this.vlines = []; this.markers = [];
        new ResizeObserver(() => this.draw()).observe(canvas);
        onThemeChange(() => this.draw());
    }

    set(parts = {}) {
        for (const key of ['series', 'fills', 'spans', 'hlines', 'vlines', 'markers']) if (parts[key]) this[key] = parts[key];
        this.draw();
    }

    range() {
        const { opts } = this;
        let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
        const add = (xs, ys) => {
            const n = Math.min(xs.length, ys.length);
            for (let i = 0; i < n; i++) {
                const x = xs[i], y = ys[i];
                if (!Number.isFinite(x) || !Number.isFinite(y) || (opts.logY && y <= 0)) continue;
                if (opts.x && (x < opts.x[0] || x > opts.x[1])) continue;
                if (x < x0) x0 = x; if (x > x1) x1 = x;
                if (y < y0) y0 = y; if (y > y1) y1 = y;
            }
        };
        for (const s of this.series) if (!s.noScale) add(s.x, s.y);
        for (const f of this.fills) { add(f.x, f.lo); add(f.x, f.hi); }
        for (const l of this.hlines) if (l.scale) { if (l.y < y0) y0 = l.y; if (l.y > y1) y1 = l.y; }
        if (!Number.isFinite(x0)) { x0 = 0; x1 = 1; }
        if (!Number.isFinite(y0)) { y0 = 0; y1 = 1; }
        if (opts.x) [x0, x1] = opts.x;
        if (opts.y) [y0, y1] = opts.y;
        else if (opts.logY) { y0 = 10 ** Math.floor(Math.log10(y0)); y1 = 10 ** Math.ceil(Math.log10(y1)); if (opts.yMin) y0 = Math.max(y0, opts.yMin); }
        else {
            if (opts.yIncludeZero) { y0 = Math.min(y0, 0); y1 = Math.max(y1, 0); }
            [y0, y1] = niceRange(y0, y1);
        }
        if (x1 <= x0) x1 = x0 + 1;
        if (y1 <= y0) y1 = y0 + 1;
        return { x0, x1, y0, y1 };
    }

    draw() {
        const canvas = this.canvas, dpr = Math.min(window.devicePixelRatio || 1, 2);
        const w = canvas.clientWidth, hgt = canvas.clientHeight;
        if (!w || !hgt) return;
        if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(hgt * dpr)) {
            canvas.width = Math.round(w * dpr); canvas.height = Math.round(hgt * dpr);
        }
        const ctx = canvas.getContext('2d');
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
        ctx.clearRect(0, 0, w, hgt);
        const p = palette(), font = getComputedStyle(document.body).fontFamily;
        const { x0, x1, y0, y1 } = this.range();
        const { logY } = this.opts;
        const L = PAD.l, R = w - PAD.r, T = PAD.t, B = hgt - PAD.b;
        const fy = (v) => (logY ? Math.log10(Math.max(v, 1e-300)) : v);
        const X = (v) => L + (v - x0) / (x1 - x0) * (R - L);
        const Y = (v) => B - (fy(v) - fy(y0)) / (fy(y1) - fy(y0)) * (B - T);

        for (const s of this.spans) {               // shaded intervals (disturbances)
            const a = Math.max(s.x0, x0), b = Math.min(s.x1 ?? x1, x1);
            if (b <= a) continue;
            ctx.fillStyle = s.color;
            ctx.fillRect(X(a), T, X(b) - X(a), B - T);
        }
        const yTicks = logY ? logTicks(y0, y1) : niceTicks(y0, y1, 4), xTicks = niceTicks(x0, x1, 5);
        ctx.strokeStyle = p.grid; ctx.lineWidth = 1;
        for (const t of yTicks) { ctx.beginPath(); ctx.moveTo(L, Y(t)); ctx.lineTo(R, Y(t)); ctx.stroke(); }
        if (!logY && y0 < 0 && y1 > 0) { ctx.strokeStyle = p.gridStrong; ctx.beginPath(); ctx.moveTo(L, Y(0)); ctx.lineTo(R, Y(0)); ctx.stroke(); }

        ctx.save();
        ctx.beginPath(); ctx.rect(L, T - 1, R - L, B - T + 2); ctx.clip();
        for (const f of this.fills) {               // band between lo and hi
            ctx.fillStyle = f.color;
            ctx.beginPath();
            let started = false;
            for (let i = 0; i < f.x.length; i++) { const px = X(f.x[i]), py = Y(f.hi[i]); if (started) ctx.lineTo(px, py); else { ctx.moveTo(px, py); started = true; } }
            for (let i = f.x.length - 1; i >= 0; i--) ctx.lineTo(X(f.x[i]), Y(f.lo[i]));
            ctx.closePath(); ctx.fill();
        }
        for (const l of this.hlines) {
            ctx.strokeStyle = l.color || alpha(p.ink, 0.3); ctx.lineWidth = l.width || 1; ctx.setLineDash(l.dash || [4, 4]);
            ctx.beginPath(); ctx.moveTo(X(l.x0 ?? x0), Y(l.y)); ctx.lineTo(X(l.x1 ?? x1), Y(l.y)); ctx.stroke();
            if (l.label) { ctx.setLineDash([]); ctx.fillStyle = l.color; ctx.font = `600 9.5px ${font}`; ctx.textAlign = 'left'; ctx.textBaseline = 'bottom'; ctx.fillText(l.label, X(l.x0 ?? x0) + 4, Y(l.y) - 2); }
        }
        for (const l of this.vlines) {
            ctx.strokeStyle = l.color || alpha(p.ink, 0.3); ctx.lineWidth = l.width || 1; ctx.setLineDash(l.dash || [3, 4]);
            ctx.beginPath(); ctx.moveTo(X(l.x), T); ctx.lineTo(X(l.x), B); ctx.stroke();
        }
        ctx.setLineDash([]); ctx.lineJoin = 'round'; ctx.lineCap = 'round';
        for (const s of this.series) {
            ctx.strokeStyle = s.color; ctx.lineWidth = s.width || 1.6; ctx.globalAlpha = s.alpha ?? 1; ctx.setLineDash(s.dash || []);
            ctx.beginPath();
            let pen = false;
            const n = Math.min(s.x.length, s.y.length);
            for (let i = 0; i < n; i++) {
                const yv = s.y[i];
                if (!Number.isFinite(yv) || (logY && yv <= 0)) { pen = false; continue; }
                const px = X(s.x[i]), py = Y(yv);
                if (pen) ctx.lineTo(px, py); else { ctx.moveTo(px, py); pen = true; }
            }
            ctx.stroke();
        }
        ctx.globalAlpha = 1; ctx.setLineDash([]);
        for (const m of this.markers) {
            if (!Number.isFinite(m.y)) continue;
            ctx.fillStyle = m.color; ctx.strokeStyle = p.surface; ctx.lineWidth = 1.5;
            ctx.beginPath(); ctx.arc(X(m.x), Y(m.y), m.r || 4, 0, Math.PI * 2); ctx.fill(); ctx.stroke();
        }
        ctx.restore();

        ctx.fillStyle = p.muted; ctx.font = `10px ${font}`;
        ctx.textAlign = 'right'; ctx.textBaseline = 'middle';
        for (const t of yTicks) ctx.fillText(formatTick(t, logY), L - 6, Y(t));
        ctx.textAlign = 'center'; ctx.textBaseline = 'top';
        for (const t of xTicks) ctx.fillText(this.opts.xFormat ? this.opts.xFormat(t) : formatTick(t), X(t), B + 5);
        ctx.font = `600 10px ${font}`;
        if (this.opts.xLabel) { ctx.textBaseline = 'alphabetic'; ctx.fillText(this.opts.xLabel, (L + R) / 2, hgt - 2); }
        if (this.opts.yLabel) {
            ctx.save(); ctx.translate(11, (T + B) / 2); ctx.rotate(-Math.PI / 2);
            ctx.textBaseline = 'middle'; ctx.fillStyle = this.opts.yColor || p.muted; ctx.fillText(this.opts.yLabel, 0, 0); ctx.restore();
        }
        ctx.strokeStyle = p.lineStrong; ctx.lineWidth = 1;
        ctx.beginPath(); ctx.moveTo(L, T); ctx.lineTo(L, B); ctx.lineTo(R, B); ctx.stroke();
        this.frame = { X, Y, L, R, T, B, x0, x1, y0, y1 };
    }
}

function niceStep(span, count) {
    const raw = span / Math.max(count, 1), mag = 10 ** Math.floor(Math.log10(raw)), norm = raw / mag;
    return (norm < 1.5 ? 1 : norm < 3 ? 2 : norm < 7 ? 5 : 10) * mag;
}
export function niceRange(lo, hi) {
    if (!Number.isFinite(lo) || !Number.isFinite(hi)) return [0, 1];
    if (hi - lo < 1e-12) hi = lo + 1;
    const step = niceStep(hi - lo, 4);
    return [Math.floor(lo / step + 1e-9) * step, Math.ceil(hi / step - 1e-9) * step];
}
function niceTicks(lo, hi, count) {
    const step = niceStep(hi - lo, count), out = [];
    for (let v = Math.ceil(lo / step - 1e-9) * step; v <= hi + 1e-9 * step; v += step) out.push(Math.abs(v) < 1e-12 ? 0 : v);
    return out;
}
function logTicks(lo, hi) {
    const out = [];
    for (let e = Math.ceil(Math.log10(lo) - 1e-9); e <= Math.log10(hi) + 1e-9; e++) out.push(10 ** e);
    return out;
}
function formatTick(v, log = false) {
    if (log) { const e = Math.round(Math.log10(v)); return e >= -2 && e <= 3 ? String(Number((10 ** e).toPrecision(1))) : `1e${e}`; }
    const a = Math.abs(v);
    if (a === 0) return '0';
    if (a >= 10000 || a < 0.01) return v.toExponential(0);
    return String(Number(v.toPrecision(3)));
}
