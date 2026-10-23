// Modeling page (Sec. III): the simulated UR10 with its soft extension, moved joint by joint or
// along the null space of an EE constraint; spans, tensions, the transport and deformation terms
// of the tip velocity, J_lq, and the identification of the span map from probing motions.
import { RobotScene } from '../lib/scene3d.js';
import { LinePlot } from '../lib/plot.js';
import { initPage, loadData } from '../lib/shell.js';
import { alpha, onThemeChange, palette } from '../lib/theme.js';
import { button, card, fmt, h, label, legend, readouts, segmented, seriesLegend, slider, toast } from '../lib/ui.js';
import { add, col, dot, matmul, matvec, norm, scale, sub, symEig, transpose } from '../sim/linalg.js';
import { Arm, defaultConfig, differential, identify, Plant, SpanMap, spanJacobian, validationError } from '../sim/model.js';
import { EEConstraint, pinv, projector } from '../sim/control.js';

await initPage();
const data = await loadData();
const DEG = Math.PI / 180;
const config = defaultConfig();
const plant = new Plant(config);
const paperMap = SpanMap.fromData(data.span_map, config);
let map = paperMap, probing = null;
let q = plant.q0.slice();
let state = plant.evaluate(q);
const home = state;
const homeSpans = home.spans.slice();
let fixed = null;                       // reference shape for the transport at fixed shape
let showLabels = new Set(['anchors', 'spans', 'frames']);

// ---------------------------------------------------------------- layout
const workspace = document.getElementById('workspace');
const viewCard = card('Simulated UR10 and soft extension', 'drag to orbit, scroll to zoom');
const view = h('div', { class: 'view3d' });
const toolbar = h('div', { class: 'view-toolbar' });
view.append(toolbar);
viewCard.body.append(view, legend([
    { label: 'tendon 1 ($a_1$ on the wrist)', color: palette().tendon1 },
    { label: 'tendon 2 ($a_2$ on the forearm)', color: palette().tendon2 },
    { label: 'soft extension', color: palette().rod },
    { label: 'tip $p_D$', color: palette().path, kind: 'dot' },
    { label: 'identified $\\widehat{p}_D$', color: palette().reference, kind: 'dot' },
    { label: 'frames: $x$ red, $y$ green, $z$ blue', color: palette().muted, kind: 'dotted' },
]));
const scene = new RobotScene(view, { view: 'ee' });

const plotsRow = h('div', { class: 'plots three' });
const spanCard = card('Exterior span changes $\\Delta\\ell^o_i$ [mm]', '');
const spanCanvas = h('canvas', { class: 'plot-canvas' });
spanCard.body.append(spanCanvas, seriesLegend([{ label: '$\\Delta\\ell^o_1$', color: palette().tendon1 }, { label: '$\\Delta\\ell^o_2$', color: palette().tendon2 }]));
const tensionCard = card('Tensions $\\tau_i$ [N]', '');
const tensionCanvas = h('canvas', { class: 'plot-canvas' });
tensionCard.body.append(tensionCanvas, seriesLegend([{ label: '$\\tau_1$', color: palette().tendon1 }, { label: '$\\tau_2$', color: palette().tendon2 }]));
const splitCard = card('Tip velocity per joint rate [mm/°]', 'columns of $J_t$');
const splitCanvas = h('canvas', { class: 'plot-canvas' });
splitCard.body.append(splitCanvas, seriesLegend([{ label: 'transport $\\|J_{\\mathrm{transport}}\\|$', color: palette().accent },
    { label: 'deformation $\\|R_B J_{\\mathrm{span}} J_{\\ell q}\\|$', color: palette().rod }]));
plotsRow.append(spanCard.el, tensionCard.el, splitCard.el);

const identCard = card('Identified model (Sec. III-F)', 'probing motions and the span map');
const plane = h('canvas', { class: 'span-plane' });
const identReadouts = readouts([
    { key: 'source', label: 'span map' }, { key: 'probes', label: 'probes reached' }, { key: 'skipped', label: 'skipped' },
    { key: 'valid', label: 'validation RMS, tip [mm]' },
]);
const rangeSlider = slider({ label: 'probing range $\\pm$ [mm]', min: 4, max: 25, step: 1, value: 10, format: (v) => v.toFixed(0) });
const gridSelect = segmented({ options: [{ value: 7, label: '7 × 7' }, { value: 9, label: '9 × 9' }, { value: 11, label: '11 × 11' }], value: 11 });
const noiseSlider = slider({ label: 'camera noise [mm]', min: 0, max: 2, step: 0.1, value: 0.5, format: (v) => v.toFixed(1) });
const probeButton = button('Probe and fit', () => runProbing(), { primary: true });
const paperButton = button('Use the paper’s map', () => { map = paperMap; probing = null; updateIdent(); refresh(); });
identCard.body.append(h('div', { class: 'two-col' },
    h('div', {}, plane),
    h('div', {}, h('p', { class: 'interaction-guide' }, label('The grid of span targets around the spans at $q_0$, the identified span domain $\\mathcal L$ (box), and the current spans (dot). The tip predicted by the map is the orange point in the 3D view.')),
        rangeSlider.el, h('div', { class: 'control' }, h('div', { class: 'control-head' }, h('label', {}, 'probe grid')), gridSelect.el),
        noiseSlider.el, h('div', { class: 'toolbar' }, probeButton, paperButton), h('div', { style: 'margin-top:12px' }, identReadouts.el))));

const left = h('div', { class: 'stack' }, viewCard.el, plotsRow, identCard.el);

// rail: joint motion, readouts, J_lq
const motionCard = card('Joint motion', '');
const mode = segmented({ options: [{ value: 'joint', label: 'By joint' }, { value: 'null', label: 'Null-space motion' }], value: 'joint',
    onChange: () => setMode() });
const jointSliders = Array.from({ length: 6 }, (_, j) => slider({
    label: `$q_${j + 1}$ [°]`, min: plant.arm.limits[j][0] / DEG, max: plant.arm.limits[j][1] / DEG, step: 0.1, value: q[j] / DEG,
    format: (v) => v.toFixed(1), onInput: (v) => { q[j] = v * DEG; refresh(); },
}));
const jointPanel = h('div', { class: 'jog-grid' }, jointSliders.map((s) => s.el));
const constraintSelect = segmented({ options: [{ value: 'position', label: 'position' }, { value: 'fulcrum', label: 'fulcrum' }, { value: 'free', label: 'free EE' }],
    value: 'position', onChange: () => setConstraint() });
const directionsBox = h('div', { class: 'stack' });
const nullPanel = h('div', { class: 'hidden' },
    h('div', { class: 'control' }, h('div', { class: 'control-head' }, h('label', {}, 'EE constraint')), constraintSelect.el),
    h('p', { class: 'control-help' }, label('Hold a button to move along a direction of the null space of $J_c$; the restoring term $\\dot{q}_c$ corrects the drift.')),
    directionsBox);
const homeButton = button('Home $q_0$', () => { q = plant.q0.slice(); plant.reset(); fixed = null; scene.setGhost(null); setConstraint(); refresh(); syncSliders(); });
const fixButton = button('Fix shape here', () => { fixed = { local: state.local, rotations: state.rotations }; refresh(); });
motionCard.body.append(mode.el, h('div', { style: 'height:10px' }), jointPanel, nullPanel, h('div', { class: 'toolbar', style: 'margin-top:8px' }, homeButton, fixButton));

const readCard = card('At this configuration', '');
const values = readouts([
    { key: 'l1', label: '$\\ell^o_1$ [mm]' }, { key: 'l2', label: '$\\ell^o_2$ [mm]' },
    { key: 't1', label: '$\\tau_1$ [N]' }, { key: 't2', label: '$\\tau_2$ [N]' },
    { key: 'tip', label: 'tip moved [mm]' }, { key: 'ee', label: '$p_B$ moved [mm]' },
    { key: 'def', label: 'deformation [mm]', title: 'distance from the tip transported at fixed shape' },
    { key: 'active', label: 'tendons' },
]);
readCard.body.append(values.el);
const jlqCard = card('$J_{\\ell q}$ [mm/°]', '(4)');
const jlqTable = h('table', { class: 'matrix' });
jlqCard.body.append(h('div', { class: 'matrix-wrap' }, jlqTable), h('p', { class: 'control-help', style: 'margin-top:6px' }, label('Row $i$: change of the span $\\ell^o_i$ per degree of each joint.')));
const rail = h('div', { class: 'rail' }, motionCard.el, readCard.el, jlqCard.el);
workspace.append(h('div', { class: 'stage wide-rail' }, left, rail));

toolbar.append(
    button('EE view', () => scene.frameView('ee'), { small: true }),
    button('Arm view', () => scene.frameView('arm'), { small: true }),
);

// ---------------------------------------------------------------- plots
const spanPlot = new LinePlot(spanCanvas, { xLabel: 'time [s]' });
const tensionPlot = new LinePlot(tensionCanvas, { xLabel: 'time [s]' });
const history = { t: [], l1: [], l2: [], t1: [], t2: [] };
const t0 = performance.now();
function record() {
    const t = (performance.now() - t0) / 1000;
    history.t.push(t);
    history.l1.push(1e3 * (state.spans[0] - homeSpans[0])); history.l2.push(1e3 * (state.spans[1] - homeSpans[1]));
    history.t1.push(state.tensions[0]); history.t2.push(state.tensions[1]);
    if (history.t.length > 900) for (const k of Object.keys(history)) history[k].shift();
}
function drawPlots() {
    const p = palette(), x = history.t;
    spanPlot.set({ series: [{ x, y: history.l1, color: p.tendon1 }, { x, y: history.l2, color: p.tendon2 }] });
    tensionPlot.set({ series: [{ x, y: history.t1, color: p.tendon1 }, { x, y: history.t2, color: p.tendon2 }] });
}

function drawSplit(diff) {
    const c = splitCanvas, dpr = Math.min(window.devicePixelRatio || 1, 2), w = c.clientWidth, hh = c.clientHeight;
    if (!w || !hh || !diff) return;
    c.width = Math.round(w * dpr); c.height = Math.round(hh * dpr);
    const ctx = c.getContext('2d'), p = palette(), f = getComputedStyle(document.body).fontFamily;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, hh);
    const transport = [0, 1, 2, 3, 4, 5].map((j) => norm(col(diff.Jtr.slice(0, 3), j)) * 1e3 * DEG);
    const deform = [0, 1, 2, 3, 4, 5].map((j) => norm(sub(col(diff.Jt, j), col(diff.Jtr.slice(0, 3), j))) * 1e3 * DEG);
    const max = Math.max(16, ...transport, ...deform);
    const L = 34, R = w - 8, T = 8, B = hh - 24, gw = (R - L) / 6;
    ctx.strokeStyle = p.grid; ctx.fillStyle = p.muted; ctx.font = `10px ${f}`; ctx.textAlign = 'right';
    for (let v = 0; v <= max; v += 5) { const y = B - (B - T) * v / max; ctx.beginPath(); ctx.moveTo(L, y); ctx.lineTo(R, y); ctx.stroke(); ctx.fillText(String(v), L - 5, y + 3); }
    ctx.textAlign = 'center';
    for (let j = 0; j < 6; j++) {
        const x = L + gw * j, bw = gw * 0.32;
        ctx.fillStyle = p.accent; ctx.fillRect(x + gw * 0.16, B - (B - T) * transport[j] / max, bw, (B - T) * transport[j] / max);
        ctx.fillStyle = p.rod; ctx.fillRect(x + gw * 0.52, B - (B - T) * deform[j] / max, bw, (B - T) * deform[j] / max);
        ctx.fillStyle = p.muted; ctx.fillText(`q${j + 1}`, x + gw / 2, B + 14);
    }
}

function drawMatrix(Jlq) {
    const p = palette();
    const rows = Jlq.map((row) => row.map((v) => v * 1e3 * DEG));
    const max = Math.max(0.2, ...rows.flat().map(Math.abs));
    jlqTable.replaceChildren(
        h('tr', {}, h('th', {}), [1, 2, 3, 4, 5, 6].map((j) => h('th', {}, `q${j}`))),
        ...rows.map((row, i) => h('tr', {}, h('th', {}, `ℓ${i + 1}`), row.map((v) => {
            const td = h('td', {}, Math.abs(v) < 5e-4 ? '0' : v.toFixed(3));
            td.style.background = alpha(v >= 0 ? p.accent : p.reference, 0.08 + 0.5 * Math.abs(v) / max);
            return td;
        }))));
}

// ---------------------------------------------------------------- span plane
function drawPlane() {
    const c = plane, dpr = Math.min(window.devicePixelRatio || 1, 2), w = c.clientWidth, hh = c.clientHeight;
    if (!w || !hh) return;
    c.width = Math.round(w * dpr); c.height = Math.round(hh * dpr);
    const ctx = c.getContext('2d'), p = palette(), f = getComputedStyle(document.body).fontFamily;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, hh);
    const R = 27, M = 30;                                    // +-27 mm
    const X = (d) => M + (w - M - 10) * (d + R) / (2 * R), Y = (d) => hh - M - (hh - M - 10) * (d + R) / (2 * R);
    ctx.strokeStyle = p.grid; ctx.fillStyle = p.muted; ctx.font = `10px ${f}`;
    for (let v = -20; v <= 20; v += 10) {
        ctx.beginPath(); ctx.moveTo(X(v), Y(-R)); ctx.lineTo(X(v), Y(R)); ctx.stroke();
        ctx.beginPath(); ctx.moveTo(X(-R), Y(v)); ctx.lineTo(X(R), Y(v)); ctx.stroke();
        ctx.textAlign = 'center'; ctx.fillText(String(v), X(v), hh - M + 14);
        ctx.textAlign = 'right'; ctx.fillText(String(v), M - 5, Y(v) + 3);
    }
    ctx.textAlign = 'center'; ctx.font = `600 10px ${f}`;
    ctx.fillText('Δℓ°₁ [mm]', (M + w) / 2, hh - 3);
    ctx.save(); ctx.translate(10, (hh - M) / 2); ctx.rotate(-Math.PI / 2); ctx.fillText('Δℓ°₂ [mm]', 0, 0); ctx.restore();
    const d = (s, i) => 1e3 * (s - homeSpans[i]);
    // identified span domain
    ctx.fillStyle = alpha(p.accent, 0.12); ctx.strokeStyle = p.accent; ctx.lineWidth = 1.5;
    const x0 = X(d(map.lower[0], 0)), x1 = X(d(map.upper[0], 0)), y0 = Y(d(map.lower[1], 1)), y1 = Y(d(map.upper[1], 1));
    ctx.fillRect(x0, y1, x1 - x0, y0 - y1); ctx.strokeRect(x0, y1, x1 - x0, y0 - y1);
    ctx.fillStyle = p.accentStrong; ctx.textAlign = 'left'; ctx.fillText('𝓛', x0 + 4, y1 + 12);
    if (probing) {
        probing.wanted.forEach((s, k) => {
            const px = X(d(s[0], 0)), py = Y(d(s[1], 1));
            if (probing.reached[k]) { ctx.fillStyle = p.ink; ctx.beginPath(); ctx.arc(px, py, 2.2, 0, 2 * Math.PI); ctx.fill(); }
            else { ctx.strokeStyle = p.bad; ctx.lineWidth = 1.2; ctx.beginPath(); ctx.moveTo(px - 3, py - 3); ctx.lineTo(px + 3, py + 3); ctx.moveTo(px - 3, py + 3); ctx.lineTo(px + 3, py - 3); ctx.stroke(); }
        });
    }
    ctx.strokeStyle = p.path; ctx.lineWidth = 1.4; ctx.beginPath();
    history.l1.forEach((v, i) => (i ? ctx.lineTo(X(v), Y(history.l2[i])) : ctx.moveTo(X(v), Y(history.l2[i])))); ctx.stroke();
    ctx.fillStyle = p.path; ctx.beginPath(); ctx.arc(X(d(state.spans[0], 0)), Y(d(state.spans[1], 1)), 4.5, 0, 2 * Math.PI); ctx.fill();
}

function updateIdent() {
    identReadouts.set('source', map === paperMap ? 'paper' : 'yours');
    identReadouts.set('probes', probing ? String(probing.spans.length) : '104');
    identReadouts.set('skipped', probing ? String(probing.skipped) : '17');
    identReadouts.set('valid', map.validation !== undefined ? fmt(1e3 * map.validation, 2) : '–');
    drawPlane();
}

function runProbing() {
    probeButton.disabled = true;
    probeButton.textContent = 'Probing…';
    setTimeout(() => {
        try {
            const probePlant = new Plant(config);
            const result = identify(probePlant, { grid: gridSelect.get(), range: rangeSlider.get() / 1e3, noise: noiseSlider.get() / 1e3, seed: 0 });
            map = result.map;
            probing = result.data;
            map.validation = validationError(map, probePlant, 120).rms;
        } catch (error) {
            toast(`The fit failed: ${error.message}`);
        }
        probeButton.disabled = false;
        probeButton.textContent = 'Probe and fit';
        updateIdent();
        refresh(false);
    }, 30);
}
paperMap.validation = 0.00152;          // validation RMS of the paper's map, Sec. VI-A (examples/identify.py)

// ---------------------------------------------------------------- null-space motion
let constraint = null, directions = [], held = null, lastFrame = 0;
function setConstraint() {
    const kind = constraintSelect.get();
    const B = state.frames[6];
    constraint = kind === 'position' ? new EEConstraint('position', { pStar: B.p })
        : kind === 'fulcrum' ? new EEConstraint('fulcrum', { c: add(B.p, scale(col(B.R, 2), 0.14)) }) : new EEConstraint('free');
    scene.setFulcrum(kind === 'fulcrum' ? constraint.c : null);
    directions = [];
    buildDirections();
}

// Orthonormal directions of range(N_c): first those that move the tip (right singular vectors of
// A = J_t N_c, by decreasing singular value), then those that leave it in place.
function nullDirections(diff) {
    const JB = diff.JB, Jc = constraint.jacobian(state.frames, JB), Nc = projector(Jc);
    const A = matmul(diff.Jt, Nc);
    const { values, vectors } = symEig(matmul(transpose(A), A));
    const out = [];
    for (let k = 5; k >= 0; k--) if (values[k] > 1e-10) out.push({ v: col(vectors, k), sigma: Math.sqrt(values[k]) });
    const M = Nc.map((row, i) => row.map((x, j) => x - out.reduce((s, d) => s + d.v[i] * d.v[j], 0)));
    const rest = symEig(M);
    for (let k = 5; k >= 0; k--) if (rest.values[k] > 0.5) out.push({ v: col(rest.vectors, k), sigma: 0 });
    return { list: out, Jc, Nc };
}

function buildDirections() {
    const diff = differential(plant, state.frames, state.p_D, state.spans, null, state.z);
    const { list } = nullDirections(diff);
    directions = list.map((d, k) => ({ ...d, previous: d.v }));
    directionsBox.replaceChildren(...directions.map((d, k) => {
        const minus = h('button', { class: 'button small', type: 'button' }, '◀');
        const plus = h('button', { class: 'button small', type: 'button' }, '▶');
        for (const [b, s] of [[minus, -1], [plus, 1]]) {
            b.addEventListener('pointerdown', (e) => { e.preventDefault(); held = { k, s }; lastFrame = performance.now(); requestAnimationFrame(move); });
            for (const ev of ['pointerup', 'pointerleave', 'pointercancel']) b.addEventListener(ev, () => { held = null; });
        }
        const text = d.sigma > 0 ? `moves the tip, ${(d.sigma * 1e3 * DEG).toFixed(2)} mm/°` : 'leaves the tip in place';
        d.note = h('span', { class: 'hint' }, text);
        return h('div', { class: 'toolbar' }, minus, plus, h('b', { style: 'font-size:0.78rem' }, `direction ${k + 1}`), d.note);
    }));
}

function move(now) {
    if (!held) return;
    const dt = Math.min((now - lastFrame) / 1000, 0.05);
    lastFrame = now;
    const diff = differential(plant, state.frames, state.p_D, state.spans, null, state.z);
    const { list, Jc } = nullDirections(diff);
    // keep the directions continuous along the motion (same order, same sign)
    const d = directions[held.k];
    let best = list[Math.min(held.k, list.length - 1)];
    for (const c of list) if (Math.abs(dot(c.v, d.previous)) > Math.abs(dot(best.v, d.previous))) best = c;
    const v = dot(best.v, d.previous) < 0 ? scale(best.v, -1) : best.v;
    d.previous = v;
    const ec = constraint.error(state.frames);
    const qdotC = Jc.length ? matvec(pinv(Jc), scale(ec, 3.0)) : [0, 0, 0, 0, 0, 0];
    const qdot = add(qdotC, scale(v, held.s * 6 * DEG));
    q = q.map((x, j) => Math.min(Math.max(x + dt * qdot[j], plant.arm.limits[j][0]), plant.arm.limits[j][1]));
    refresh();
    requestAnimationFrame(move);
}

function setMode() {
    const nullMode = mode.get() === 'null';
    jointPanel.classList.toggle('hidden', nullMode);
    nullPanel.classList.toggle('hidden', !nullMode);
    if (nullMode) setConstraint(); else { scene.setFulcrum(null); syncSliders(); }
}
function syncSliders() { jointSliders.forEach((s, j) => s.set(q[j] / DEG)); }

// ---------------------------------------------------------------- update
function labels() {
    const items = [], p = palette();
    if (showLabels.has('anchors')) state.anchors.forEach((a, i) => items.push({ p: a, text: `a${'₁₂'[i]}`, cls: 'symbol', color: [p.tendon1, p.tendon2][i] }));
    if (showLabels.has('entries')) state.entries.forEach((b, i) => items.push({ p: b, text: `b${'₁₂'[i]}`, cls: 'symbol', color: [p.tendon1, p.tendon2][i], dx: 8, dy: 6 + 10 * i }));
    if (showLabels.has('spans')) state.anchors.forEach((a, i) => items.push({ p: scale(add(a, state.entries[i]), 0.5), text: `ℓ°${'₁₂'[i]}`, cls: 'symbol', color: [p.tendon1, p.tendon2][i] }));
    if (showLabels.has('frames')) { items.push({ p: state.frames[6].p, text: '{B}', cls: 'symbol', dx: -26 }); items.push({ p: state.p_D, text: '{D}', cls: 'symbol', dx: -28, dy: 10 }); }
    if (showLabels.has('body')) state.bodyPoints.slice(1, -1).forEach((b) => items.push({ p: b, text: 'pⱼ', cls: 'symbol' }));
    if (showLabels.has('ghost') && fixed) items.push({ p: ghostTip, text: 'fixed shape', dy: 14 });
    scene.setLabels(items);
}

let ghostTip = null;
function refresh(recordSample = true) {
    state = plant.evaluate(q);
    scene.setState(state, { tendonOffsets: plant.tendons.map((t) => t.entry) });
    const B = state.frames[6];
    if (fixed) {
        const points = fixed.local.map((p) => add(B.p, matvec(B.R, p)));
        scene.setGhost(points, fixed.rotations.map((R) => matmul(B.R, R)));
        ghostTip = points[points.length - 1];
    } else { scene.setGhost(null); ghostTip = null; }
    const inside = state.spans.every((s, i) => s >= map.lower[i] - 1e-9 && s <= map.upper[i] + 1e-9);
    scene.setReference(add(B.p, matvec(B.R, map.predictTip(state.spans))));
    const diff = differential(plant, state.frames, state.p_D, state.spans, null, state.z);
    values.set('l1', fmt(1e3 * state.spans[0], 1)); values.set('l2', fmt(1e3 * state.spans[1], 1));
    values.set('t1', fmt(state.tensions[0], 2)); values.set('t2', fmt(state.tensions[1], 2));
    values.set('tip', fmt(1e3 * norm(sub(state.p_D, home.p_D)), 1));
    values.set('ee', fmt(1e3 * norm(sub(B.p, home.frames[6].p)), 1));
    values.set('def', ghostTip ? fmt(1e3 * norm(sub(state.p_D, ghostTip)), 1) : '–');
    values.set('active', state.active.map((a, i) => (a ? 'taut' : state.tensions[i] > 0 ? 'saturated' : 'slack')).join(', '), state.active.every(Boolean) ? '' : 'bad');
    viewCard.note.textContent = inside ? 'spans inside 𝓛' : 'spans outside 𝓛: the identified prediction extrapolates';
    viewCard.note.style.color = inside ? '' : palette().bad;
    drawMatrix(spanJacobian(plant, state.frames));
    drawSplit(diff);
    if (recordSample) record();
    drawPlots();
    drawPlane();
    labels();
}

// notation rows: show the symbols in the 3D view
for (const row of document.querySelectorAll('#notation tr[data-show]')) {
    row.addEventListener('click', () => {
        const key = row.dataset.show;
        if (showLabels.has(key)) showLabels.delete(key); else showLabels.add(key);
        if (key === 'ghost' && !fixed) fixed = { local: state.local, rotations: state.rotations };
        for (const r of document.querySelectorAll('#notation tr[data-show]')) r.classList.toggle('active', showLabels.has(r.dataset.show));
        refresh(false);
    });
}
for (const r of document.querySelectorAll('#notation tr[data-show]')) r.classList.toggle('active', showLabels.has(r.dataset.show));

new ResizeObserver(() => { drawSplit(differential(plant, state.frames, state.p_D, state.spans, null, state.z)); drawPlane(); }).observe(splitCanvas);
onThemeChange(() => { scene.refreshTheme(); if (constraint && constraintSelect.get() === 'fulcrum' && mode.get() === 'null') scene.setFulcrum(constraint.c); refresh(false); });
updateIdent();
refresh();
// ?mode=null&constraint=fulcrum opens the null-space motion
const params = new URLSearchParams(location.search);
if (params.get('constraint')) constraintSelect.set(params.get('constraint'));
if (params.get('mode') === 'null') { mode.set('null'); setMode(); }
