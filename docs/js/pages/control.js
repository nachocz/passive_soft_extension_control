// Control demo (Secs. IV and VI): the closed loop of the simulations, S1-S3, with the plant,
// the identified model, the null-space control law, and the safety QP of docs/js/sim.
import { RobotScene } from '../lib/scene3d.js';
import { LinePlot } from '../lib/plot.js';
import { initPage, loadData } from '../lib/shell.js';
import { onThemeChange, palette } from '../lib/theme.js';
import { button, card, clock, control, fmt, h, label, legend, readouts, segmented, select, seriesLegend, slider, toggle } from '../lib/ui.js';
import { add, dot, norm, sub } from '../sim/linalg.js';
import { defaultConfig, Plant, SpanMap } from '../sim/model.js';
import { fulcrumGeometry, FULCRUM, SCENARIOS, Simulation } from '../sim/control.js';

await initPage();
const data = await loadData();
const DEG = Math.PI / 180;
const paperMap = SpanMap.fromData(data.span_map, defaultConfig());

const settings = {
    scenario: new URLSearchParams(location.search).get('scenario') || 'S2', constraint: 'scenario', exact: false, adapt: true,
    barriers: true, obstacleBarrier: true, spanBarrier: true, filter: 'dls', k_t: 1.4, xi: 0.04, k_c: 3.0, noise: 0.5,
};
if (!SCENARIOS[settings.scenario]) settings.scenario = 'S2';
let sim = null, running = false, speed = 1, extraTargets = [], lastWall = 0, lastLog = 0, lastIndex = -1, frame = 0;

// ---------------------------------------------------------------- layout
const workspace = document.getElementById('workspace');
const runCard = card('', '');
const scenarioPick = segmented({ options: [{ value: 'S1', label: 'S1' }, { value: 'S2', label: 'S2' }, { value: 'S3', label: 'S3' }],
    value: settings.scenario, onChange: (v) => { settings.scenario = v; extraTargets = []; restart(); } });
const runButton = button('▶ Run', () => setRunning(!running), { primary: true });
const restartButton = button('↺ Restart', () => restart());
const speedPick = segmented({ options: [1, 2, 4, 8].map((v) => ({ value: v, label: `${v}×` })), value: 1, onChange: (v) => { speed = v; } });
const clockEl = h('span', { class: 'clock' }, '0:00');
const status = h('div', { class: 'status-row' });
runCard.body.append(h('div', { class: 'toolbar' }, h('b', { style: 'font-size:0.8rem' }, 'Scenario'), scenarioPick.el, runButton, restartButton,
    h('span', { class: 'toolbar-divider' }), speedPick.el, clockEl), h('div', { style: 'margin-top:8px' }, status));

const viewCard = card('Simulated UR10', 'drag to orbit, scroll to zoom');
const view = h('div', { class: 'view3d' });
const viewTools = h('div', { class: 'view-toolbar' });
view.append(viewTools);
viewCard.body.append(view, legend([
    { label: 'tendons 1 and 2', color: palette().tendon1 }, { label: 'soft extension', color: palette().rod },
    { label: 'tip path', color: palette().path }, { label: 'tip reference', color: palette().reference, kind: 'dot' },
    { label: 'targets', color: palette().target, kind: 'dot' }, { label: 'obstacles', color: palette().obstacle, kind: 'band' },
    { label: 'fulcrum $c$ and cone', color: palette().fulcrum, kind: 'dashed' },
]));
const scene = new RobotScene(view, { view: 'ee' });
viewTools.append(button('EE view', () => scene.frameView('ee'), { small: true }), button('Arm view', () => scene.frameView('arm'), { small: true }));

const plotDefs = [
    { key: 'error', title: 'Tip error $\\|e\\|$ [mm]', opts: { logY: true, yMin: 0.01 } },
    { key: 'rates', title: 'Joint rates $\\dot q_j$ [°/s]', opts: {} },
    { key: 'spans', title: 'Span changes $\\Delta\\ell^o_i$ [mm]', opts: {} },
    { key: 'barriers', title: 'Barrier margins $h-\\varepsilon_h$ [mm]', opts: {} },
];
const plots = {};
const plotsGrid = h('div', { class: 'plots' });
for (const d of plotDefs) {
    const c = card(d.title, '');
    const canvas = h('canvas', { class: 'plot-canvas' });
    c.body.append(canvas);
    plots[d.key] = { plot: new LinePlot(canvas, { xLabel: 'time [s]', ...d.opts }), card: c };
    plotsGrid.append(c.el);
}
plots.error.card.body.append(seriesLegend([{ label: 'tip error', color: palette().path }, { label: '4 mm arrival tolerance', color: palette().muted, dash: 'dashed' }]));
plots.rates.card.body.append(seriesLegend([1, 2, 3, 4, 5, 6].map((j) => ({ label: `$\\dot q_${j}$`, color: palette()[`j${j}`] }))));
plots.spans.card.body.append(seriesLegend([{ label: '$\\Delta\\ell^o_1$', color: palette().tendon1 }, { label: '$\\Delta\\ell^o_2$', color: palette().tendon2 },
    { label: 'bounds of $\\mathcal L$', color: palette().muted, dash: 'dashed' }]));
const barrierLegend = h('div');
plots.barriers.card.body.append(barrierLegend);
const left = h('div', { class: 'stack' }, runCard.el, viewCard.el, plotsGrid);

// rail
const setCard = card('Controller and plant', 'changes apply at once');
const constraintSelect = select({
    options: [{ value: 'scenario', label: 'of the scenario' }, { value: 'free', label: 'free EE' }, { value: 'position', label: 'position-constrained EE' },
        { value: 'fulcrum', label: 'laparoscopic fulcrum (ideal)' }, { value: 'aperture', label: 'fulcrum at a 50 mm aperture (barriers only)' }],
    value: 'scenario', onChange: (v) => { settings.constraint = v; restart(); },
});
const modelPick = segmented({ options: [{ value: 'identified', label: 'identified $\\widehat J_{\\mathrm{span}}$' }, { value: 'exact', label: 'exact $J_{\\mathrm{span}}$' }],
    value: 'identified', onChange: (v) => { settings.exact = v === 'exact'; apply(); } });
const adaptToggle = toggle({ label: 'online (Broyden) correction', value: true, onChange: (v) => { settings.adapt = v; apply(); } });
const barrierToggle = toggle({ label: 'barriers and QP (19)', value: true, onChange: (v) => { settings.barriers = v; apply(); } });
const obstacleToggle = toggle({ label: 'obstacle barrier $h_{\\mathrm{obs}}$', value: true, onChange: (v) => { settings.obstacleBarrier = v; apply(); } });
const spanToggle = toggle({ label: 'span-domain barrier $h_{\\mathrm{span}}$', value: true, onChange: (v) => { settings.spanBarrier = v; apply(); } });
const filterPick = segmented({ options: [{ value: 'dls', label: 'damped inverse' }, { value: 'transpose', label: 'transpose' }], value: 'dls',
    onChange: (v) => { settings.filter = v; apply(); } });
const ktSlider = slider({ label: 'tip gain $k_t$ [1/s]', min: 0.2, max: 3, step: 0.05, value: 1.4, onInput: (v) => { settings.k_t = v; apply(); } });
const xiSlider = slider({ label: 'damping $\\xi$ [m]', min: 0.005, max: 0.12, step: 0.005, value: 0.04, format: (v) => v.toFixed(3), onInput: (v) => { settings.xi = v; apply(); } });
const kcSlider = slider({ label: 'EE gain $k_c$ [1/s]', min: 0.5, max: 6, step: 0.1, value: 3.0, onInput: (v) => { settings.k_c = v; apply(); } });
const noiseSlider = slider({ label: 'camera noise [mm]', min: 0, max: 2, step: 0.1, value: 0.5, format: (v) => v.toFixed(1), onInput: (v) => { settings.noise = v; apply(); } });
setCard.body.append(control('EE constraint (restarts)', constraintSelect.el), control('model of the controller', modelPick.el), adaptToggle.el,
    h('p', { class: 'group-label' }, 'Barriers'), barrierToggle.el, obstacleToggle.el, spanToggle.el,
    control('spectral filter (nominal control law, without barriers)', filterPick.el),
    h('p', { class: 'group-label' }, 'Gains and sensing'), ktSlider.el, xiSlider.el, kcSlider.el, noiseSlider.el);

const targetCard = card('New target', 'relative to the tip');
const offsets = ['x', 'y', 'z'].map((axis) => slider({ label: `$\\Delta ${axis}$ [mm]`, min: -60, max: 60, step: 1, value: 0, format: (v) => v.toFixed(0),
    onInput: () => showCandidate() }));
const addButton = button('Add target', () => addTarget(), { primary: true });
targetCard.body.append(...offsets.map((s) => s.el), h('div', { class: 'toolbar' }, addButton, h('span', { class: 'hint' }, 'appended to the queue')));

const readCard = card('Now', '');
const values = readouts([
    { key: 'target', label: 'target' }, { key: 'error', label: 'tip error [mm]' }, { key: 'ee', label: 'EE deviation [mm]' },
    { key: 'clear', label: 'min. clearance [mm]', title: 'distance of the polyline through the body points of the plant to the closest obstacle' },
    { key: 'depth', label: 'insertion $\\delta$ [mm]' }, { key: 'tilt', label: 'tilt $\\theta$ [°]' },
]);
const activeRow = h('div', { class: 'status-row', style: 'margin-top:10px' });
const events = h('ul', { class: 'events', style: 'margin-top:10px' });
readCard.body.append(values.el, activeRow, events);
const rail = h('div', { class: 'rail' }, readCard.el, setCard.el, targetCard.el);
workspace.append(h('div', { class: 'stage wide-rail' }, left, rail));

// ---------------------------------------------------------------- simulation
function makeSim() {
    const config = defaultConfig();
    config.control.k_t = settings.k_t; config.control.xi = settings.xi; config.control.k_c = settings.k_c;
    config.sensing.noise = settings.noise / 1e3;
    const base = SCENARIOS[settings.scenario];
    const scenario = { ...base, targets: [...base.targets, ...extraTargets] };
    return new Simulation(new Plant(config), scenario, paperMap, {
        exact: settings.exact, adapt: settings.adapt, barriers: settings.barriers, obstacleBarrier: settings.obstacleBarrier,
        spanBarrier: settings.spanBarrier, filter: settings.filter, constraint: settings.constraint === 'scenario' ? undefined : settings.constraint,
        apertureRadius: 0.05,
    });
}

// live changes of the controller and the sensing (no restart)
function apply() {
    if (!sim) return;
    const c = sim.controller;
    c.gains({ ...sim.config.control, k_t: settings.k_t, xi: settings.xi, k_c: settings.k_c });
    Object.assign(c, { exact: settings.exact, barriers: settings.barriers, obstacleBarrier: settings.obstacleBarrier, spanBarrier: settings.spanBarrier, filter: settings.filter });
    sim.adapt = settings.adapt;
    sim.noise = settings.noise / 1e3;
    filterPick.el.style.opacity = settings.barriers ? 0.5 : 1;
    [obstacleToggle, spanToggle].forEach((t) => { t.el.style.opacity = settings.barriers ? 1 : 0.5; });
}

function restart() {
    sim = makeSim();
    lastLog = 0; lastIndex = -1;
    scene.clearTrail();
    const kind = sim.constraintKind;
    scene.setObstacles(sim.scenario.obstacles);
    scene.setFulcrum(kind === 'fulcrum' || kind === 'aperture' ? (sim.scenario.fulcrum ?? FULCRUM) : null,
        { halfAngle: sim.config.barriers.cone.half_angle_deg * DEG, apertureRadius: kind === 'aperture' ? 0.05 : 0 });
    updateScene(true);
    apply();
    drawPlots();
    events.replaceChildren();
}

function setRunning(flag) {
    running = flag && !(sim && sim.finished);
    runButton.replaceChildren(label(running ? '❚❚ Pause' : '▶ Run'));
    if (running) { lastWall = performance.now(); requestAnimationFrame(loop); }
}

function loop(now) {
    if (!running) return;
    const dt = Math.min((now - lastWall) / 1000, 0.1);
    lastWall = now;
    const target = sim.time + dt * speed, start = performance.now();
    while (sim.time < target - 1e-9 && !sim.finished && performance.now() - start < 30) {
        sim.step();
        if (sim.log.t.length > lastLog) { for (let k = lastLog; k < sim.log.t.length; k++) scene.pushTrail(sim.log.tip[k]); lastLog = sim.log.t.length; }
    }
    updateScene(false);
    if (++frame % 6 === 0 || sim.finished) drawPlots();
    if (sim.finished) { setRunning(false); drawPlots(); }
    else requestAnimationFrame(loop);
}

// ---------------------------------------------------------------- views
function candidate() {
    const tip = sim.measured.tip;
    return add(tip, offsets.map((s) => s.get() / 1e3));
}
function showCandidate() { updateTargets(true); }

function updateTargets(force = false) {
    if (!force && sim.index === lastIndex) return;
    lastIndex = sim.index;
    const showNew = offsets.some((s) => s.get() !== 0);
    scene.setTargets(sim.targets, sim.finished ? -1 : sim.index, sim.scenario.infeasible || [], showNew ? candidate() : null);
}

function addTarget() {
    const t = candidate();
    extraTargets.push(t);
    sim.targets.push(t);
    if (sim.finished) { sim.finished = false; sim.index = sim.targets.length - 1; sim.resetTarget(); }
    offsets.forEach((s) => s.set(0));
    updateTargets(true);
}

function activeRows(cmd) {
    if (!cmd || !cmd.qp) return [];
    return cmd.rows.filter((r) => Math.abs(dot(r.grad, cmd.qdot) + r.alpha * (r.h - r.eps)) < 1e-7 || r.h - r.eps < 0);
}

function updateScene(force) {
    const s = sim.state;
    scene.setState(s, { tendonOffsets: sim.plant.tendons.map((t) => t.entry) });
    scene.setReference(sim.finished ? null : sim.reference);
    updateTargets(force);
    clockEl.textContent = `t = ${clock(sim.time)}`;
    const goal = sim.targets[Math.min(sim.index, sim.targets.length - 1)];
    values.set('target', `${Math.min(sim.index + 1, sim.targets.length)} / ${sim.targets.length}`);
    values.set('error', fmt(1e3 * norm(sub(goal, s.p_D)), 1));
    values.set('ee', sim.constraintKind === 'free' || sim.constraintKind === 'aperture' ? '–' : fmt(1e3 * sim.controller.constraint.deviation(s.frames), 3));
    values.set('clear', sim.obstacles.length ? fmt(1e3 * Math.min(...sim.obstacles.map((o) => o.clearance(s.bodyPoints))), 1) : '–');
    if (sim.controller.fulcrum) {
        const g = fulcrumGeometry(sim.controller.fulcrum, s.frames);
        values.set('depth', fmt(1e3 * g.delta, 1)); values.set('tilt', fmt(g.theta / DEG, 1));
    } else { values.set('depth', '–'); values.set('tilt', '–'); }
    const cmd = sim.last, active = activeRows(cmd);
    const kinds = [...new Set(active.map((r) => (r.kind === 'obstacle' ? r.name : r.kind === 'span' ? 'span domain' : r.kind)))];
    activeRow.replaceChildren(h('span', { class: `status-chip ${cmd?.qp ? 'good' : ''}` }, !cmd ? 'not started' : cmd.qp ? 'QP (19)' : 'nominal control law (15)'),
        ...(kinds.length ? kinds.map((k) => h('span', { class: 'status-chip on' }, `active: ${k}`)) : [h('span', { class: 'status-chip' }, 'no active barrier')]));
    status.replaceChildren(h('span', { class: 'status-chip' }, sim.scenario.title), h('span', { class: 'status-chip' }, `EE: ${{ free: 'free', position: 'position-constrained', fulcrum: 'fulcrum', aperture: 'finite aperture' }[sim.constraintKind]}`),
        sim.finished ? h('span', { class: 'status-chip good' }, 'run complete') : running ? h('span', { class: 'status-chip on' }, 'running') : h('span', { class: 'status-chip' }, 'paused'));
    if (events.children.length !== sim.events.length) {
        events.replaceChildren(...sim.events.map((e) => h('li', {}, h('b', {}, `T${e.target}`),
            h('span', {}, `${e.outcome === 'reached' ? 'reached' : e.outcome === 'stalled' ? 'no further decrease' : 'time budget'} at ${e.t.toFixed(1)} s`),
            h('span', {}, `${(1e3 * e.error).toFixed(1)} mm`))));
    }
}

function changes(log) {
    const t = log.t, idx = [];
    for (let k = 1; k < log.target.length; k++) if (log.target[k] !== log.target[k - 1]) idx.push(t[k]);
    return idx.map((x) => ({ x }));
}

function drawPlots() {
    const p = palette(), log = sim.log, t = log.t, v = changes(log);
    plots.error.plot.set({ series: [{ x: t, y: log.error.map((e) => 1e3 * e), color: p.path }], hlines: [{ y: 4, color: p.muted }], vlines: v });
    plots.rates.plot.set({ series: [0, 1, 2, 3, 4, 5].map((j) => ({ x: t, y: log.qdot.map((q) => q[j] / DEG), color: p[`j${j + 1}`], width: 1.2 })), vlines: v });
    const s0 = sim.plant.spans(sim.plant.q0), map = sim.spanMap;
    plots.spans.plot.set({
        series: [0, 1].map((i) => ({ x: t, y: log.spans.map((s) => 1e3 * (s[i] - s0[i])), color: [p.tendon1, p.tendon2][i] })),
        hlines: [0, 1].flatMap((i) => [map.lower[i], map.upper[i]].map((b) => ({ y: 1e3 * (b - s0[i]), color: [p.tendon1, p.tendon2][i], dash: [4, 4], scale: true }))),
        vlines: v,
    });
    const groups = { obstacle: [], span: [], insertion: [], aperture: [] };
    for (const [name, values] of Object.entries(sim.barrierLog)) {
        const kind = name.startsWith('span') ? 'span' : name.startsWith('insertion') ? 'insertion' : name === 'aperture' ? 'aperture' : name === 'cone' ? null : 'obstacle';
        if (kind) groups[kind].push(values.map((h) => 1e3 * (h - sim.margins[name])));
    }
    const colors = { obstacle: p.obstacle, span: p.accent, insertion: p.fulcrum, aperture: p.reference };
    const series = Object.entries(groups).filter(([, rows]) => rows.length).map(([kind, rows]) => ({
        kind, x: t, y: t.map((_, k) => Math.min(...rows.map((r) => (Number.isFinite(r[k]) ? r[k] : Infinity)))), color: colors[kind],
    }));
    plots.barriers.plot.set({ series, hlines: [{ y: 0, color: p.ink, dash: [] }], vlines: v });
    const names = { obstacle: 'obstacle', span: 'span domain', insertion: 'insertion', aperture: 'aperture' };
    barrierLegend.replaceChildren(seriesLegend(series.map((s) => ({ label: `${names[s.kind]} (min.)`, color: s.color }))));
}

onThemeChange(() => { scene.refreshTheme(); scene.setObstacles(sim.scenario.obstacles); restartViews(); drawPlots(); });
function restartViews() {
    const kind = sim.constraintKind;
    scene.setFulcrum(kind === 'fulcrum' || kind === 'aperture' ? (sim.scenario.fulcrum ?? FULCRUM) : null,
        { halfAngle: sim.config.barriers.cone.half_angle_deg * DEG, apertureRadius: kind === 'aperture' ? 0.05 : 0 });
    updateTargets(true);
}
restart();
// ?scenario=S3&run&speed=8 starts a run directly
const params = new URLSearchParams(location.search);
if (params.has('speed')) { speed = Number(params.get('speed')) || 1; speedPick.set(speed); }
if (params.has('run')) setRunning(true);
