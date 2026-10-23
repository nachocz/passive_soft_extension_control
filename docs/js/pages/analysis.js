// Analysis page (Sec. V): Step 1, the bound (22) on the rank of the constrained deformation
// Jacobian, with movable anchors and slack tendons; Step 2, the descent of V under the nominal
// control law (Fig. 7); Step 3, a closed-loop run (S3) in which active barriers stop the descent.
import { RobotScene } from '../lib/scene3d.js';
import { LinePlot } from '../lib/plot.js';
import { initPage, loadData } from '../lib/shell.js';
import { alpha, onThemeChange, palette } from '../lib/theme.js';
import { button, card, clock, control, fmt, h, label, legend, readouts, segmented, select, seriesLegend, slider, toggle } from '../lib/ui.js';
import { add, col, dot, matmul, norm, scale, singularValues, sub } from '../sim/linalg.js';
import { defaultConfig, Plant, SpanMap } from '../sim/model.js';
import { EEConstraint, FULCRUM, NominalRun, rankReport, SCENARIOS, Simulation } from '../sim/control.js';

await initPage();
const data = await loadData();
const DEG = Math.PI / 180;
const paperMap = SpanMap.fromData(data.span_map, defaultConfig());

// ---------------------------------------------------------------- layout
const workspace = document.getElementById('workspace');
const stepDefs = [
    { key: 'rank', num: 'Step 1', title: 'Deformation rank (V-A)' },
    { key: 'descent', num: 'Step 2', title: 'Descent of V (V-B)' },
    { key: 'scope', num: 'Step 3', title: 'Active barriers (V-C)' },
];
const tabs = h('div', { class: 'steps', role: 'tablist' });
const tabButtons = stepDefs.map((d) => {
    const b = h('button', { class: 'step-tab', role: 'tab', type: 'button', 'aria-selected': 'false' }, h('b', {}, d.num), h('span', {}, d.title));
    b.addEventListener('click', () => activate(d.key));
    tabs.append(b);
    return b;
});
const viewCard = card('Simulated UR10', 'drag to orbit, scroll to zoom');
const view = h('div', { class: 'view3d' });
const viewTools = h('div', { class: 'view-toolbar' });
view.append(viewTools);
viewCard.body.append(view, legend([
    { label: 'tendon 1', color: palette().tendon1 }, { label: 'tendon 2', color: palette().tendon2 },
    { label: 'tip path', color: palette().path }, { label: 'targets', color: palette().target, kind: 'dot' },
    { label: 'fulcrum $c$', color: palette().fulcrum, kind: 'dashed' },
]));
const scene = new RobotScene(view, { view: 'ee' });
viewTools.append(button('EE view', () => scene.frameView('ee'), { small: true }), button('Arm view', () => scene.frameView('arm'), { small: true }));
const leftPanels = {}, railPanels = {};
for (const d of stepDefs) { leftPanels[d.key] = h('div', { class: 'stack hidden' }); railPanels[d.key] = h('div', { class: 'rail hidden' }); }
const left = h('div', { class: 'stack' }, tabs, viewCard.el, ...Object.values(leftPanels));
const rail = h('div', { class: 'rail' }, ...Object.values(railPanels));
workspace.append(h('div', { class: 'stage wide-rail' }, left, rail));
const offsets = (plant) => plant.tendons.map((t) => t.entry);

// ================================================================ Step 1: rank bound (22)
const rank = { kind: 'position', links: [4, 3], fractions: [0.75, 0.6], slack: [false, false], q: defaultConfig().arm.q0_deg.map((v) => v * DEG) };
let rankPlant = null;
const factorCard = card('Factors of the bound (22)', 'at this configuration');
const factorBox = h('div', { class: 'factors' });
factorCard.body.append(factorBox, h('p', { class: 'control-help', style: 'margin-top:10px' },
    label('Each factor bounds $\\operatorname{rank} J_{\\mathrm{def}}$; the smallest of the four in the second line of (22) is highlighted.')));
const svCard = card('Singular values', '');
const svBox = h('div');
svCard.body.append(svBox);
leftPanels.rank.append(h('div', { class: 'two-col' }, factorCard.el, svCard.el));

const rankSet = card('EE constraint and tendons', '');
const kindPick = segmented({ options: [{ value: 'free', label: 'free' }, { value: 'position', label: 'position' }, { value: 'fulcrum', label: 'fulcrum' }],
    value: 'position', onChange: (v) => { rank.kind = v; updateRank(); } });
const linkSelects = [0, 1].map((i) => select({
    options: [1, 2, 3, 4, 5, 6].map((l) => ({ value: String(l), label: `link ${l}${l === 3 ? ' (forearm)' : l === 4 ? ' (wrist 1)' : l === 6 ? ' (EE)' : ''}` })),
    value: String(rank.links[i]), onChange: (v) => { rank.links[i] = Number(v); rankPlant = null; updateRank(); },
}));
const fractionSliders = [0, 1].map((i) => slider({ label: `position of $a_${i + 1}$ along the link`, min: 0.05, max: 0.95, step: 0.05, value: rank.fractions[i],
    onChange: (v) => { rank.fractions[i] = v; rankPlant = null; updateRank(); } }));
const slackToggles = [0, 1].map((i) => toggle({ label: `tendon ${i + 1} slack (10 mm longer)`, value: false, onChange: (v) => { rank.slack[i] = v; rankPlant = null; updateRank(); } }));
const wristSliders = [3, 4, 5].map((j) => slider({ label: `$q_${j + 1}$ [°]`, min: -180, max: 180, step: 1, value: rank.q[j] / DEG, format: (v) => v.toFixed(0),
    onInput: (v) => { rank.q[j] = v * DEG; updateRank(); } }));
const resetRank = button('Reset', () => {
    Object.assign(rank, { links: [4, 3], fractions: [0.75, 0.6], slack: [false, false], q: defaultConfig().arm.q0_deg.map((v) => v * DEG) });
    linkSelects.forEach((s, i) => s.set(String(rank.links[i]))); fractionSliders.forEach((s, i) => s.set(rank.fractions[i]));
    slackToggles.forEach((t) => t.set(false)); wristSliders.forEach((s, k) => s.set(rank.q[k + 3] / DEG));
    rankPlant = null; updateRank();
});
rankSet.body.append(control('EE constraint', kindPick.el),
    ...[0, 1].flatMap((i) => [h('p', { class: 'group-label' }, `Tendon ${i + 1}`), control(`anchor $a_${i + 1}$ on`, linkSelects[i].el), fractionSliders[i].el, slackToggles[i].el]),
    h('p', { class: 'group-label' }, 'Wrist joints'), ...wristSliders.map((s) => s.el), h('div', { class: 'toolbar' }, resetRank));
railPanels.rank.append(rankSet.el);

function makeRankPlant() {
    const config = defaultConfig();
    config.tendons.forEach((t, i) => { t.anchor_link = rank.links[i]; t.anchor_fraction = rank.fractions[i]; });
    const plant = new Plant(config);
    rank.slack.forEach((s, i) => { if (s) plant.rod.rest[i] += 0.01; });
    return plant;
}

function factor(text, value, max, cls = '') {
    const bar = h('div', { class: 'bar' }, h('i'));
    bar.firstChild.style.width = `${100 * value / max}%`;
    return h('div', { class: `factor ${cls}` }, h('span', {}, label(text)), bar, h('b', {}, String(value)));
}

function updateRank() {
    if (!rankPlant) rankPlant = makeRankPlant();
    rankPlant.reset();
    const s = rankPlant.evaluate(rank.q, false);
    const B = s.frames[6];
    const c = add(B.p, scale(col(B.R, 2), 0.14));
    const constraint = rank.kind === 'position' ? new EEConstraint('position', { pStar: B.p })
        : rank.kind === 'fulcrum' ? new EEConstraint('fulcrum', { c }) : new EEConstraint('free');
    const r = rankReport(rankPlant, s, constraint);
    const four = [r.rankJDtau, r.mA, r.rankJlq, r.nMinusRc], least = Math.min(...four);
    factorBox.replaceChildren(
        factor('$\\operatorname{rank} J_{D\\tau}$ (soft body)', r.rankJDtau, 6, r.rankJDtau === least ? 'limiting' : ''),
        factor('$m_a$ (taut, unsaturated tendons)', r.mA, 6, r.mA === least ? 'limiting' : ''),
        factor('$\\operatorname{rank} J_{\\ell q}$ (anchor placement)', r.rankJlq, 6, r.rankJlq === least ? 'limiting' : ''),
        factor('$n-r_c$ (null-space motion)', r.nMinusRc, 6, r.nMinusRc === least ? 'limiting' : ''),
        factor('$\\operatorname{rank}(J_{\\ell q}N_c)$', r.rankJlqNc, 6),
        factor('bound (22)', r.boundFirst, 6, 'result'),
        factor('$\\operatorname{rank} J_{\\mathrm{def}}$', r.rankJdef, 6, 'result'));
    const deg = 1e3 * DEG;
    const tipDef = matmul(B.R, r.Jdef.slice(0, 3));
    const list = (values, unit) => h('span', { class: 'mono' }, values.length
        ? values.map((v) => (v < 1e-6 * Math.max(...values) ? '0' : v.toPrecision(3))).join(', ') + unit : '–');
    svBox.replaceChildren(
        control('$\\sigma(J_{\\ell q})$ [mm/°]', list(r.svJlq.map((v) => v * deg), '')),
        control('$\\sigma(J_{\\ell q}N_c)$ [mm/°]', list(r.svJlqNc.map((v) => v * deg), '')),
        control('$\\sigma$ of the tip rows of $\\bar R_B J_{\\mathrm{def}}$ [mm/°]', list(singularValues(tipDef).map((v) => v * deg), '')),
        h('p', { class: 'control-help' }, label('The tip rows give the tip displacement per degree of null-space motion produced by the tendons; the other rows of $J_{\\mathrm{def}}$ rotate $\\{D\\}$.')),
        control('tensions $\\tau$ [N]', h('span', { class: 'mono' }, s.tensions.map((t) => t.toFixed(2)).join(', '))));
    scene.setState(s, { tendonOffsets: offsets(rankPlant) });
    scene.setFulcrum(rank.kind === 'fulcrum' ? c : null);
    scene.setLabels(s.anchors.map((a, i) => ({ p: a, text: `a${'₁₂'[i]}`, cls: 'symbol', color: [palette().tendon1, palette().tendon2][i] })));
}

// ================================================================ Step 2: descent (Fig. 7)
const desc = { model: 'exact', filter: 'dls', xi: 0.04, k_t: 1.4, speed: 4 };
let run = null, descRunning = false, descLast = 0;
const vCard = card('$V=\\tfrac12\\|e\\|^2$ [m²]', 'log axis');
const vCanvas = h('canvas', { class: 'plot-canvas tall' });
vCard.body.append(vCanvas, seriesLegend([{ label: '$V$', color: palette().exact }, { label: 'bound (31), restarted at each target', color: palette().ink, dash: 'dotted' }]));
const rCard = card('Descent rate $-\\dot V/V$ [1/s]', '');
const rCanvas = h('canvas', { class: 'plot-canvas tall' });
rCard.body.append(rCanvas, seriesLegend([{ label: '$-\\dot V/V$', color: palette().exact }, { label: 'band $[2k_t\\beta,\\,2k_t]$ of the damped inverse', color: palette().exact, dash: 'band' }]));
const vPlot = new LinePlot(vCanvas, { xLabel: 'time [s]', logY: true, yMin: 1e-11 });
const rPlot = new LinePlot(rCanvas, { xLabel: 'time [s]', yIncludeZero: false });
const descRead = readouts([{ key: 'beta', label: '$\\beta$ along the run' }, { key: 'rate', label: '$-\\dot V/V$ [1/s]' },
    { key: 'inc', label: 'steps with $V$ increasing' }, { key: 'err', label: 'tip error [mm]' }]);
const rhoBox = h('div', { class: 'factors', style: 'margin-top:10px' });
const descRows = h('div', { class: 'plots' }, vCard.el, rCard.el);
const descReadCard = card('Quantities of Proposition 2', 'exact $A$');
descReadCard.body.append(descRead.el, h('p', { class: 'group-label', style: 'margin-top:12px' }, 'Rates along the eigenvectors of AAᵀ (32)'), rhoBox);
leftPanels.descent.append(descRows, descReadCard.el);

const descSet = card('Nominal control law (25)', 'five targets of S2, fulcrum, W = I');
const modelPick = segmented({ options: [{ value: 'exact', label: 'exact $J_D$' }, { value: 'identified', label: 'identified $\\widehat J_D$' }],
    value: 'exact', onChange: (v) => { desc.model = v; restartDescent(); } });
const filterPick = segmented({ options: [{ value: 'dls', label: 'damped inverse' }, { value: 'transpose', label: 'transpose' }], value: 'dls',
    onChange: (v) => { desc.filter = v; restartDescent(); } });
const xiSlider = slider({ label: 'damping $\\xi$ [m]', min: 0.005, max: 0.15, step: 0.005, value: 0.04, format: (v) => v.toFixed(3), onChange: (v) => { desc.xi = v; restartDescent(); } });
const ktSlider = slider({ label: 'tip gain $k_t$ [1/s]', min: 0.2, max: 3, step: 0.1, value: 1.4, onChange: (v) => { desc.k_t = v; restartDescent(); } });
const descRun = button('▶ Run', () => setDescRunning(!descRunning), { primary: true });
const descSpeed = segmented({ options: [1, 4, 16].map((v) => ({ value: v, label: `${v}×` })), value: 4, onChange: (v) => { desc.speed = v; } });
const descClock = h('span', { class: 'clock' }, '0:00');
descSet.body.append(control('model of the controller', modelPick.el), control('spectral filter (24)', filterPick.el), xiSlider.el, ktSlider.el,
    h('div', { class: 'toolbar' }, descRun, button('↺ Restart', () => restartDescent()), descSpeed.el, descClock),
    h('p', { class: 'control-help', style: 'margin-top:8px' }, label('As in Fig. 7: no barriers, no rate limits, each target held for 6 s, and the quantities of Proposition 2 computed from the exact $A$.')));
railPanels.descent.append(descSet.el);

function restartDescent() {
    const config = defaultConfig();
    const s2 = SCENARIOS.S2;
    run = new NominalRun(new Plant(config), s2.targets, new EEConstraint('fulcrum', { c: s2.fulcrum }),
        { spanMap: desc.model === 'identified' ? paperMap : null, k_t: desc.k_t, xi: desc.xi, filter: desc.filter, hold: 6 });
    if (current === 'descent') {
        scene.clearTrail(); scene.setTargets(s2.targets, 0); scene.setObstacles([]); scene.setFulcrum(s2.fulcrum); scene.setReference(null);
        scene.setLabels([]);
        scene.setState(run.state, { tendonOffsets: offsets(run.plant) });
    }
    drawDescent();
}

function setDescRunning(flag) {
    descRunning = flag && run && !run.done;
    descRun.replaceChildren(label(descRunning ? '❚❚ Pause' : '▶ Run'));
    if (descRunning) { descLast = performance.now(); requestAnimationFrame(descLoop); }
}

function descLoop(now) {
    if (!descRunning || current !== 'descent') return;
    const dt = Math.min((now - descLast) / 1000, 0.1);
    descLast = now;
    const goal = run.k * run.dt + dt * desc.speed, start = performance.now();
    let target = -1;
    while (!run.done && run.k * run.dt < goal && performance.now() - start < 30) {
        const row = run.step();
        scene.pushTrail(row.tip);
        target = row.target;
    }
    scene.setState(run.state, { tendonOffsets: offsets(run.plant) });
    if (target >= 0) scene.setTargets(SCENARIOS.S2.targets, target);
    drawDescent();
    if (run.done) setDescRunning(false); else requestAnimationFrame(descLoop);
}

function drawDescent() {
    const p = palette(), rows = run.rows, hold = run.holdSteps, dt = run.dt;
    const t = rows.map((r) => r.t), V = rows.map((r) => r.V), beta = rows.map((r) => r.beta);
    const rate = rows.map((r, k) => (k + 1 < rows.length && (k + 1) % hold !== 0 && V[k] > 1e-9 && V[k + 1] > 1e-9 ? -Math.log(V[k + 1] / V[k]) / dt : NaN));
    const betaMin = beta.length ? Math.min(...beta) : NaN;
    const bound = rows.map((r, k) => { const k0 = Math.floor(k / hold) * hold; return V[k0] * Math.exp(-2 * run.k_t * betaMin * (t[k] - t[k0])); });
    const switches = [1, 2, 3, 4].map((k) => ({ x: k * run.hold }));
    vPlot.opts.x = [0, run.hold * run.targets.length];
    rPlot.opts.x = vPlot.opts.x;
    const breakAt = (y) => y.map((v, k) => ((k + 1) % hold === 0 ? NaN : v));
    vPlot.set({ series: [{ x: t, y: breakAt(bound), color: p.ink, width: 1, dash: [1, 3] }, { x: t, y: breakAt(V), color: p.exact }], vlines: switches });
    const fills = desc.filter === 'dls' && rows.length ? [{ x: t, lo: beta.map((b) => 2 * run.k_t * b), hi: t.map(() => 2 * run.k_t), color: p.band }] : [];
    rPlot.set({ series: [{ x: t, y: rate, color: p.exact }], fills, vlines: switches });
    const finite = rate.filter(Number.isFinite);
    let increases = 0;
    for (let k = 0; k + 1 < V.length; k++) if ((k + 1) % hold !== 0 && V[k + 1] > V[k] + 1e-18) increases++;
    descRead.set('beta', rows.length ? `${Math.min(...beta).toFixed(3)} – ${Math.max(...beta).toFixed(3)}` : '–');
    descRead.set('rate', finite.length ? `${Math.min(...finite).toFixed(2)} – ${Math.max(...finite).toFixed(2)}` : '–');
    descRead.set('inc', String(increases), increases ? 'bad' : 'good');
    descRead.set('err', rows.length ? fmt(1e3 * Math.sqrt(2 * V[V.length - 1]), 3) : '–');
    descClock.textContent = `t = ${clock(run.k * dt)}`;
    const last = rows[rows.length - 1];
    rhoBox.replaceChildren(...(last ? last.rates.slice().reverse().map((rho, i) => {
        const bar = h('div', { class: 'bar' }, h('i'));
        bar.firstChild.style.width = `${Math.min(100, 100 * rho / (run.k_t * 1.05))}%`;
        return h('div', { class: 'factor' }, h('span', {}, label(`$\\rho_${i + 1}$ [1/s]`)), bar, h('b', {}, rho.toFixed(2)));
    }) : []));
}

// ================================================================ Step 3: active barriers (S3)
let sim3 = null, s3Running = false, s3Last = 0, s3Seen = 0;
const s3Active = [];
const s3Card = card('$V=\\tfrac12\\|e\\|^2$ in the closed loop of S3 [m²]', 'measured on the plant');
const s3Canvas = h('canvas', { class: 'plot-canvas tall' });
s3Card.body.append(s3Canvas, seriesLegend([{ label: '$V$', color: palette().path }, { label: 'obstacle barrier active', color: palette().obstacle, dash: 'band' },
    { label: 'insertion barrier active', color: palette().fulcrum, dash: 'band' }, { label: 'span-domain barrier active', color: palette().accent, dash: 'band' }]));
const s3Plot = new LinePlot(s3Canvas, { xLabel: 'time [s]', logY: true, yMin: 1e-8 });
const s3Events = h('ul', { class: 'events' });
const s3EventsCard = card('Targets', 'T3 and T5 are infeasible by design');
s3EventsCard.body.append(s3Events);
leftPanels.scope.append(s3Card.el, s3EventsCard.el);
const s3Set = card('Closed loop with the QP (19)', 'S3, laparoscopic fulcrum');
const s3Run = button('▶ Run', () => setS3Running(!s3Running), { primary: true });
const s3Clock = h('span', { class: 'clock' }, '0:00');
s3Set.body.append(h('p', { class: 'control-help' }, label('When an inequality of the QP is active, the closed loop is no longer the control law of Proposition 2: $V$ can settle at a nonzero value, here at T3 (inside a sphere) and T5 (beyond the insertion limit).')),
    h('div', { class: 'toolbar' }, s3Run, button('↺ Restart', () => restartS3()), s3Clock));
railPanels.scope.append(s3Set.el);

function restartS3() {
    sim3 = new Simulation(new Plant(defaultConfig()), SCENARIOS.S3, paperMap);
    s3Active.length = 0; s3Seen = 0;
    if (current === 'scope') {
        scene.clearTrail(); scene.setObstacles(SCENARIOS.S3.obstacles); scene.setFulcrum(FULCRUM); scene.setLabels([]);
        scene.setTargets(SCENARIOS.S3.targets, 0, SCENARIOS.S3.infeasible);
        scene.setState(sim3.state, { tendonOffsets: offsets(sim3.plant) });
    }
    drawS3();
}

function setS3Running(flag) {
    s3Running = flag && sim3 && !sim3.finished;
    s3Run.replaceChildren(label(s3Running ? '❚❚ Pause' : '▶ Run'));
    if (s3Running) { s3Last = performance.now(); requestAnimationFrame(s3Loop); }
}

function s3Loop(now) {
    if (!s3Running || current !== 'scope') return;
    const dt = Math.min((now - s3Last) / 1000, 0.1);
    s3Last = now;
    const goal = sim3.time + dt * 8, start = performance.now();
    let index = sim3.index;
    while (sim3.time < goal && !sim3.finished && performance.now() - start < 30) {
        sim3.step();
        if (sim3.log.t.length > s3Seen) {
            s3Seen = sim3.log.t.length;
            const cmd = sim3.last;
            const active = cmd.rows.filter((r) => Math.abs(dot(r.grad, cmd.qdot) + r.alpha * (r.h - r.eps)) < 1e-7 || r.h < r.eps).map((r) => r.kind);
            s3Active.push(new Set(active));
            scene.pushTrail(sim3.log.tip[s3Seen - 1]);
        }
    }
    scene.setState(sim3.state, { tendonOffsets: offsets(sim3.plant) });
    scene.setReference(sim3.reference);
    if (sim3.index !== index) scene.setTargets(SCENARIOS.S3.targets, sim3.index, SCENARIOS.S3.infeasible);
    drawS3();
    if (sim3.finished) setS3Running(false); else requestAnimationFrame(s3Loop);
}

function intervals(t, flags, kind, color) {
    const out = [];
    let start = null;
    flags.forEach((f, k) => {
        if (f.has(kind) && start === null) start = t[k];
        if ((!f.has(kind) || k === flags.length - 1) && start !== null) { out.push({ x0: start, x1: t[k], color }); start = null; }
    });
    return out;
}

function drawS3() {
    const p = palette(), log = sim3.log, t = log.t;
    const V = log.error.map((e) => 0.5 * e * e);
    const spans = [...intervals(t, s3Active, 'obstacle', alpha(p.obstacle, 0.16)), ...intervals(t, s3Active, 'insertion', alpha(p.fulcrum, 0.16)),
        ...intervals(t, s3Active, 'span', alpha(p.accent, 0.12))];
    const v = [];
    for (let k = 1; k < log.target.length; k++) if (log.target[k] !== log.target[k - 1]) v.push({ x: t[k] });
    s3Plot.set({ series: [{ x: t, y: V, color: p.path }], spans, vlines: v });
    s3Clock.textContent = `t = ${clock(sim3.time)}`;
    s3Events.replaceChildren(...sim3.events.map((e) => h('li', {}, h('b', {}, `T${e.target}`),
        h('span', {}, e.outcome === 'reached' ? `reached at ${e.t.toFixed(1)} s` : `no further decrease, ${e.t.toFixed(1)} s`), h('span', {}, `${(1e3 * e.error).toFixed(1)} mm`))));
}

// ================================================================ steps
let current = null;
function activate(key) {
    if (current === key) return;
    setDescRunning(false); setS3Running(false);
    current = key;
    stepDefs.forEach((d, i) => {
        tabButtons[i].setAttribute('aria-selected', String(d.key === key));
        leftPanels[d.key].classList.toggle('hidden', d.key !== key);
        railPanels[d.key].classList.toggle('hidden', d.key !== key);
    });
    scene.clearTrail(); scene.setReference(null); scene.setObstacles([]); scene.setTargets([]); scene.setLabels([]);
    if (key === 'rank') updateRank();
    if (key === 'descent') restartDescent();
    if (key === 'scope') restartS3();
    try { history.replaceState(null, '', `${location.pathname}${location.search}#${key}`); } catch { /* sandboxed */ }
}

onThemeChange(() => {
    scene.refreshTheme();
    if (current === 'rank') updateRank();
    if (current === 'descent') { scene.setTargets(SCENARIOS.S2.targets, Math.min(Math.floor(run.k / run.holdSteps), 4)); scene.setFulcrum(SCENARIOS.S2.fulcrum); drawDescent(); }
    if (current === 'scope') { scene.setObstacles(SCENARIOS.S3.obstacles); scene.setFulcrum(FULCRUM); scene.setTargets(SCENARIOS.S3.targets, sim3.index, SCENARIOS.S3.infeasible); drawS3(); }
});

const initial = location.hash.slice(1);
activate(stepDefs.some((d) => d.key === initial) ? initial : 'rank');
if (new URLSearchParams(location.search).has('run')) { if (current === 'descent') setDescRunning(true); if (current === 'scope') setS3Running(true); }
