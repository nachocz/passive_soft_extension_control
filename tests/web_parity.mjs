// Runs the JavaScript port of the simulation (docs/js/sim) with Node.js and prints its logs
// as JSON; tests/test_web_parity.py compares them with the Python package.
//     node tests/web_parity.mjs [S2] [seconds]
import fs from 'node:fs';
import { defaultConfig, Plant, SpanMap, identify, validationError } from '../docs/js/sim/model.js';
import { EEConstraint, NominalRun, SCENARIOS, Simulation, rankReport } from '../docs/js/sim/control.js';

const key = process.argv[2] || 'S2';
const seconds = Number(process.argv[3] || 10);
const config = defaultConfig();
const plant = new Plant(config);
const data = JSON.parse(fs.readFileSync(new URL('../assets/span_map.json', import.meta.url), 'utf8'));
const map = SpanMap.fromData(data, config);

const home = plant.evaluate(plant.q0, false);
const sim = new Simulation(plant, SCENARIOS[key], map);
sim.run(seconds);

const small = identify(new Plant(config), { grid: 5, range: 0.006 });
const nominal = new NominalRun(new Plant(config), SCENARIOS.S2.targets.slice(0, 1),
    new EEConstraint('fulcrum', { c: SCENARIOS.S2.fulcrum }), { hold: 1.0 });
while (!nominal.done) nominal.step();
const rank = rankReport(plant, home, new EEConstraint('position', { pStar: home.p_B }));

process.stdout.write(JSON.stringify({
    config,
    home: { spans: home.spans, tensions: home.tensions, tip: home.p_D, rest: plant.rod.rest },
    run: { t: sim.log.t, q: sim.log.q, qdot: sim.log.qdot, tip: sim.log.tip, error: sim.log.error, spans: sim.log.spans,
        barriers: sim.barrierLog, events: sim.events },
    identification: { coef: small.map.coef, skipped: small.data.skipped, validation: validationError(small.map, new Plant(config), 20).rms },
    nominal: nominal.rows.map((r) => [r.V, r.beta, r.grad]),
    rank: { svJdef: rank.svJdef, rankJdef: rank.rankJdef, boundSecond: rank.boundSecond },
}));
