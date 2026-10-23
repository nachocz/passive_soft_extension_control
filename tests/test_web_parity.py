"""The JavaScript port of the simulation (docs/js/sim) gives the same results as the Python package.

Runs tests/web_parity.mjs with Node.js (skipped without it).
"""
import json
import shutil
import subprocess
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from softext import Config, EEConstraint, Plant, Simulation, paper_span_map, scenario  # noqa: E402
from softext.analysis import nominal_run, rank_report  # noqa: E402
from softext.config import DEFAULTS  # noqa: E402
from softext.identification import identify, validation_error  # noqa: E402

SECONDS = 20.0


@unittest.skipIf(shutil.which("node") is None, "Node.js is not installed")
class WebParity(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        result = subprocess.run(["node", str(ROOT / "tests" / "web_parity.mjs"), "S2", str(SECONDS)],
                                capture_output=True, text=True, check=True, cwd=ROOT)
        cls.js = json.loads(result.stdout)
        cls.config = Config()
        cls.plant = Plant(cls.config)

    def test_defaults(self):
        self.assertEqual(json.loads(json.dumps(DEFAULTS)), self.js["config"])

    def test_plant(self):
        home = self.plant.evaluate(self.config.q0, warm=False)
        np.testing.assert_allclose(self.js["home"]["spans"], home.spans, atol=1e-12)
        np.testing.assert_allclose(self.js["home"]["tensions"], home.tensions, atol=1e-9)
        np.testing.assert_allclose(self.js["home"]["tip"], home.p_D, atol=1e-11)
        np.testing.assert_allclose(self.js["home"]["rest"], self.plant.rod.rest, atol=1e-12)

    def test_closed_loop(self):
        sim = Simulation(self.config, "S2", plant=Plant(self.config))
        logs = sim.run(SECONDS)
        js = self.js["run"]
        self.assertEqual(len(js["t"]), len(logs["t"]))
        np.testing.assert_allclose(js["q"], logs["q"], atol=1e-7)
        np.testing.assert_allclose(js["qdot"], logs["qdot"], atol=1e-6)
        np.testing.assert_allclose(js["tip"], logs["tip"], atol=1e-7)
        np.testing.assert_allclose(js["spans"], logs["spans"], atol=1e-8)
        for name, values in logs["barriers"].items():
            np.testing.assert_allclose(np.array(js["barriers"][name], float), values, atol=1e-7, err_msg=name)
        self.assertEqual([(e["target"], e["outcome"]) for e in js["events"]],
                         [(e["target"], e["outcome"]) for e in logs["events"]])

    def test_identification(self):
        span_map, data = identify(Plant(self.config), grid=5, span_range=0.006)
        self.assertEqual(self.js["identification"]["skipped"], data["skipped"])
        np.testing.assert_allclose(self.js["identification"]["coef"], span_map.coefficients, rtol=1e-6, atol=1e-9)
        self.assertAlmostEqual(self.js["identification"]["validation"], validation_error(span_map, Plant(self.config), 20), places=8)

    def test_nominal_run_and_rank(self):
        s2 = scenario("S2")
        run = nominal_run(Plant(self.config), s2.targets[:1], self.config.q0, EEConstraint("fulcrum", c=s2.fulcrum), hold=1.0)
        python = np.array([[r["V"], r["beta"], r["grad"]] for r in run["rows"]])
        np.testing.assert_allclose(self.js["nominal"], python, rtol=1e-6, atol=1e-12)
        home = self.plant.evaluate(self.config.q0, warm=False)
        report = rank_report(self.plant, home, EEConstraint("position", p_star=home.p_B))
        self.assertEqual(self.js["rank"]["rankJdef"], report["rank_J_def"])
        self.assertEqual(self.js["rank"]["boundSecond"], report["bound_second"])
        np.testing.assert_allclose(self.js["rank"]["svJdef"][:2], report["sv_J_def"][:2], rtol=1e-6)


if __name__ == "__main__":
    unittest.main()
