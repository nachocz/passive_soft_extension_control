"""Checks of the model (Sec. III), the controller (Sec. IV), and the analysis (Sec. V), and the
reproduction of the paper's simulator and of its simulation results (Sec. VI)."""
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from softext import Config, EEConstraint, Plant, Simulation, paper_span_map, scenario  # noqa: E402
from softext.analysis import nominal_run, rank_report  # noqa: E402
from softext.barriers import Fulcrum, Obstacle, fulcrum_rows, obstacle_row  # noqa: E402
from softext.identification import identify, validation_error  # noqa: E402
from softext.model import differential, span_jacobian  # noqa: E402
from softext.qp import solve_qp  # noqa: E402


def fd(fn, q, h=1e-6):
    return np.stack([(np.asarray(fn(q + h * e)) - np.asarray(fn(q - h * e))) / (2 * h) for e in np.eye(len(q))], axis=-1)


class Model(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = Config()
        cls.plant = Plant(cls.config)
        cls.q = cls.config.q0 + np.array([0.05, -0.04, 0.06, -0.08, 0.1, 0.3])

    def test_simulator_of_the_paper(self):
        """The plant equals the UR10 plant of the paper's simulator at q0 (values of that simulator)."""
        home = self.plant.evaluate(self.config.q0, warm=False)
        np.testing.assert_allclose(home.spans, [0.15213985, 0.40434223], atol=1e-8)
        np.testing.assert_allclose(home.tensions, [1.50099887, 1.50099887], atol=1e-8)
        np.testing.assert_allclose(home.p_D, [-0.77178767, -0.09203239, 0.07099849], atol=1e-8)

    def test_span_jacobian(self):
        """(4) against finite differences of (2)."""
        J = span_jacobian(self.plant, self.plant.arm.frames(self.q))
        np.testing.assert_allclose(J, fd(self.plant.spans, self.q), atol=1e-8)

    def test_distal_jacobian(self):
        """(9) with the exact J_span against finite differences of the tip of the plant."""
        state = self.plant.evaluate(self.q, warm=False)
        J = differential(self.plant, state.frames, state.p_D, state.spans, None, state.z).J_t
        np.testing.assert_allclose(J, fd(lambda q: self.plant.evaluate(q, warm=False).p_D, self.q), atol=1e-6)

    def test_constraint_jacobians(self):
        """J_c = -d e_c / d q (12) for the position constraint and the fulcrum."""
        state = self.plant.evaluate(self.q)
        J_B = differential(self.plant, state.frames, state.p_D, state.spans, None, state.z).J_B
        for c in (EEConstraint("position", p_star=state.p_B + 0.01), EEConstraint("fulcrum", c=scenario("S2").fulcrum)):
            np.testing.assert_allclose(c.jacobian(state.frames, J_B), -fd(lambda q: c.error(self.plant.arm.frames(q)), self.q), atol=1e-8)

    def test_barrier_gradients(self):
        span_map = paper_span_map(self.config)
        cfg = self.config.barriers
        fulcrum = Fulcrum(scenario("S2").fulcrum, aperture_radius=0.05)
        obstacle = Obstacle("box", "box", [-0.74, -0.06, 0.12], 0.04)

        def rows(q):
            frames = self.plant.arm.frames(q)
            spans = self.plant.spans(q)
            d = differential(self.plant, frames, self.plant.evaluate(q).p_D, spans, span_map)
            points = frames[6][:3, 3] + span_map.predict_shape(spans) @ frames[6][:3, :3].T
            return fulcrum_rows(fulcrum, frames, d.J_B, cfg) + [obstacle_row(obstacle, points, d.J_p, cfg["obstacle"])]

        numeric = fd(lambda q: [r.h for r in rows(q)], self.q)
        np.testing.assert_allclose(np.array([r.grad for r in rows(self.q)]), numeric, atol=1e-6)

    def test_qp_kkt(self):
        rng = np.random.default_rng(0)
        M = rng.normal(size=(6, 6))
        H, g = M @ M.T + 0.1 * np.eye(6), rng.normal(size=6)
        A = np.vstack((rng.normal(size=(4, 6)), np.eye(6), -np.eye(6)))
        b = np.concatenate((rng.normal(size=4) - 1.0, np.full(12, -0.3)))
        x = solve_qp(H, g, A, b)
        slack = A @ x - b
        self.assertGreater(slack.min(), -1e-10)
        active = np.abs(slack) < 1e-9
        lam = np.linalg.lstsq(A[active].T, H @ x + g, rcond=None)[0]
        np.testing.assert_allclose(A[active].T @ lam, H @ x + g, atol=1e-9)
        self.assertGreater(lam.min(), -1e-9)


class Analysis(unittest.TestCase):
    def test_rank_bound(self):
        """Proposition 1 at q0 for the three EE constraints, and Corollary 1 with both anchors on the EE."""
        config = Config()
        plant = Plant(config)
        home = plant.evaluate(config.q0, warm=False)
        for c in (EEConstraint("free"), EEConstraint("position", p_star=home.p_B), EEConstraint("fulcrum", c=scenario("S2").fulcrum)):
            r = rank_report(plant, home, c)
            self.assertLessEqual(r["rank_J_def"], r["bound_first"])
            self.assertLessEqual(r["bound_first"], r["bound_second"])
            self.assertEqual(r["rank_J_def"], 2)
        tendons = [dict(t, anchor_link=6, anchor_fraction=0.5) for t in config.raw["tendons"]]
        on_ee = Plant(config.replace(tendons=tendons))
        state = on_ee.evaluate(config.q0, warm=False)
        r = rank_report(on_ee, state, EEConstraint("free"))
        self.assertEqual(r["rank_J_lq"], 0)
        self.assertEqual(r["rank_J_def"], 0)

    def test_descent(self):
        """Proposition 2 and Corollary 3 on the plant: V never increases, beta in [0.87, 0.93] (Fig. 7)."""
        config = Config()
        s2 = scenario("S2")
        run = nominal_run(Plant(config), s2.targets[:2], config.q0, EEConstraint("fulcrum", c=s2.fulcrum))
        V = np.array([r["V"] for r in run["rows"]])
        hold = int(round(run["hold"] / run["dt"]))
        steps = np.diff(V)
        steps[hold - 1::hold] = 0.0          # target switches
        self.assertTrue(np.all(steps <= 1e-15))
        beta = [r["beta"] for r in run["rows"]]
        self.assertGreater(min(beta), 0.87)
        self.assertLess(max(beta), 0.93)


class Results(unittest.TestCase):
    def test_identification(self):
        """Sec. VI-A: validation RMS of the identified tip, 1.5 mm with the probing range of +/-10 mm."""
        plant = Plant(Config())
        self.assertAlmostEqual(1e3 * validation_error(paper_span_map(), plant), 1.5, delta=0.1)
        span_map, data = identify(plant)
        self.assertEqual(len(data["spans"]), 104)
        self.assertAlmostEqual(1e3 * validation_error(span_map, plant), 1.5, delta=0.1)

    def test_s2(self):
        """Sec. VI-C: in S2 every target is reached and the tool axis stays within 0.1 mm of the fulcrum."""
        logs = Simulation(Config(), "S2").run(120)
        self.assertEqual([e["outcome"] for e in logs["events"]], ["reached"] * 5)
        self.assertLess(logs["ee_deviation"].max(), 1e-4)
        for name, h in logs["barriers"].items():
            self.assertGreaterEqual(np.nanmin(h), 0.0, name)

    def test_s3(self):
        """Sec. VI-D: in S3 the error settles about 50 mm from T3 and T5, and the other targets are reached."""
        logs = Simulation(Config(), "S3").run(200)
        outcome = {e["target"]: e for e in logs["events"]}
        for k in (1, 2, 4, 6):
            self.assertEqual(outcome[k]["outcome"], "reached")
        for k in (3, 5):
            self.assertEqual(outcome[k]["outcome"], "stalled")
            self.assertGreater(outcome[k]["error"], 0.04)


if __name__ == "__main__":
    unittest.main()
