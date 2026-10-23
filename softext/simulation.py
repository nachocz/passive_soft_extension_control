"""Closed-loop simulation of a scenario (Sec. VI-A, Table IV).

The plant integrates the joint rates at 100 Hz and solves the equilibrium of the
soft extension; the camera reports the tip and the body points at 20 Hz with
Gaussian noise; the controller runs at 50 Hz on the joint angles and the latest
measurement. The tip reference moves toward the current target at a bounded
speed. A target is reached when the measured tip error stays below the arrival
tolerance for the hold time; the run moves on when the error stops decreasing.
The noise-free state of the plant is logged for evaluation only.
"""
from __future__ import annotations

import copy

import numpy as np

from .barriers import Fulcrum
from .config import Config
from .constraints import EEConstraint
from .controller import Controller
from .identification import SpanMap, paper_span_map
from .plant import Plant
from .rng import Random
from .scenarios import FULCRUM, Scenario, scenario as get_scenario


def advance_reference(reference: np.ndarray, goal: np.ndarray, speed: float, dt: float) -> np.ndarray:
    delta = goal - reference
    distance = float(np.linalg.norm(delta))
    return goal.copy() if distance <= speed * dt else reference + delta * (speed * dt / distance)


class Simulation:
    def __init__(self, config: Config | None = None, scenario: Scenario | str = "S2", span_map: SpanMap | None = None,
                 *, exact: bool = False, adapt: bool = True, barriers: bool = True, obstacle_barrier: bool = True,
                 spectral_filter: str = "dls", noise: float | None = None, seed: int | None = None, plant: Plant | None = None,
                 constraint: str | None = None, aperture_radius: float = 0.05, span_barrier: bool = True):
        self.config = config = config or Config()
        self.scenario = get_scenario(scenario) if isinstance(scenario, str) else scenario
        self.plant = plant or Plant(config)
        self.plant.reset()
        self.span_map = copy.deepcopy(span_map or paper_span_map(config))
        self.span_map.reset()
        self.adapt = adapt
        sensing, rate = config.sensing, config.sensing["plant_rate"]
        self.noise = sensing["noise"] if noise is None else noise
        self.rng = Random(sensing["seed"] if seed is None else seed)
        self.dt = 1.0 / rate
        self.sense_every = int(round(rate / sensing["rate"]))
        self.control_every = int(round(rate / config.control["rate"]))
        self.q = config.q0.copy()
        self.qdot = np.zeros(6)
        self.state = self.plant.evaluate(self.q)
        # EE constraint of the scenario, or another one: free, position, fulcrum, or "aperture", the
        # fulcrum at a finite aperture imposed by barriers only (r_c = 0), as on the real setup.
        s = self.scenario
        kind = constraint or s.constraint
        point = s.fulcrum if s.fulcrum is not None else FULCRUM
        if kind == "position":
            ee = EEConstraint("position", p_star=self.state.p_B.copy())
        elif kind == "fulcrum":
            ee = EEConstraint("fulcrum", c=point)
        else:
            ee = EEConstraint("free")
        fulcrum = Fulcrum(point, aperture_radius=aperture_radius if kind == "aperture" else 0.0)             if kind in ("fulcrum", "aperture") else None
        self.controller = Controller(self.plant, ee, self.span_map, s.obstacle_objects(), fulcrum,
                                     barriers=barriers, obstacle_barrier=obstacle_barrier, span_barrier=span_barrier,
                                     exact=exact, spectral_filter=spectral_filter)
        self.targets = np.array(s.targets, float)
        t = config.targets
        self.tolerance, self.release = t["tolerance"], t["tolerance"] + t["hysteresis"]
        control_dt = self.dt * self.control_every
        self.hold_ticks = int(round(t["hold"] / control_dt))
        self.stall_ticks = int(round(t["stall"] / control_dt))
        self.budget_ticks = int(round(t["budget"] / control_dt))
        self.stall_progress = t["stall_progress"]
        self.tick, self.index, self.finished = 0, 0, False
        self._reset_target()
        self.events = []
        self.log = {key: [] for key in ("t", "q", "qdot", "tip", "tip_measured", "reference", "goal", "target",
                                        "error", "spans", "tensions", "ee_deviation", "qp", "body_points")}
        self.barrier_log: dict[str, list] = {}       # h of every barrier row, per control step
        self.barrier_margins: dict[str, float] = {}  # eps_h
        self.measured, self._previous = None, None
        self._measure()
        self.reference = self.measured["tip"].copy()

    @property
    def time(self) -> float:
        return self.tick * self.dt

    def _reset_target(self) -> None:
        self.held = self.stalled = self.target_ticks = 0
        self.best = np.inf

    def _measure(self) -> None:
        state, n = self.state, self.noise
        tip = state.p_D + n * self.rng.normal_array(3)
        points = state.body_points + n * self.rng.normal_array(state.body_points.shape)
        self.measured = {"tip": tip, "points": points}

    def _control(self) -> None:
        if self.finished:
            self.qdot = np.zeros(6)
            return
        cfg = self.config.control
        goal = self.targets[self.index]
        tip = self.measured["tip"]
        if self.adapt:
            # Online secant correction, at every control step from the joint angles and the latest
            # measurement, expressed in {B} so that the motion of the arm is not credited to the soft
            # extension (span changes below broyden_min_step only decay the correction).
            s = self.state
            local = s.R_B.T @ (tip - s.p_B)
            if self._previous is not None:
                self.span_map.update(s.spans, self._previous[0], local - self._previous[1])
            self._previous = (s.spans.copy(), local)
        self.reference = advance_reference(self.reference, goal, cfg["reference_speed"], self.dt * self.control_every)
        command = self.controller.command(self.q, tip, self.reference, z=self.state.z)
        self.qdot = command.qdot
        self._record(command, goal)
        # target bookkeeping on the measured error
        error = float(np.linalg.norm(goal - tip))
        if error < self.tolerance:
            self.held += 1
        elif error >= self.release:
            self.held = 0
        dwelling = self.held > 0 and error < self.release
        improved = error <= self.best - self.stall_progress
        if improved:
            self.best = error
        self.stalled = 0 if (dwelling or improved) else self.stalled + 1
        self.target_ticks += 1
        outcome = ("reached" if self.held >= self.hold_ticks else "stalled" if self.stalled >= self.stall_ticks
                   else "budget" if self.target_ticks >= self.budget_ticks else None)
        if outcome:
            true_error = float(np.linalg.norm(goal - self.state.p_D))
            self.events.append({"target": self.index + 1, "t": self.time, "outcome": outcome, "error": true_error})
            if self.index + 1 < len(self.targets):
                self.index += 1
                self._reset_target()
            else:
                self.finished = True

    def _record(self, command, goal) -> None:
        s, log = self.state, self.log
        log["t"].append(self.time)
        log["q"].append(self.q.copy())
        log["qdot"].append(command.qdot.copy())
        log["tip"].append(s.p_D.copy())
        log["tip_measured"].append(self.measured["tip"].copy())
        log["reference"].append(self.reference.copy())
        log["goal"].append(goal.copy())
        log["target"].append(self.index + 1)
        log["error"].append(float(np.linalg.norm(goal - s.p_D)))
        log["spans"].append(s.spans.copy())
        log["tensions"].append(s.tensions.copy())
        log["ee_deviation"].append(self.controller.constraint.deviation(s.frames))
        log["qp"].append(command.qp)
        log["body_points"].append(s.body_points.copy())
        n = len(log["t"])
        for row in command.rows:
            values = self.barrier_log.setdefault(row.name, [])
            values.extend([np.nan] * (n - 1 - len(values)))
            values.append(row.h)
            self.barrier_margins[row.name] = row.eps

    def step(self) -> None:
        """One plant tick."""
        sense, control = self.tick % self.sense_every == 0, self.tick % self.control_every == 0
        if (sense or control) and self.tick > 0:
            self.state = self.plant.evaluate(self.q)
        if sense and self.tick > 0:
            self._measure()
        if control:
            self._control()
        limits = self.plant.arm.limits
        self.q = np.clip(self.q + self.dt * self.qdot, limits[:, 0], limits[:, 1])
        self.tick += 1

    def run(self, duration: float | None = None) -> dict:
        """Run until the last target is reached or abandoned (or for `duration` seconds)."""
        end = np.inf if duration is None else self.time + duration
        while not self.finished and self.time < end - 1e-9:
            self.step()
        return self.logs()

    def logs(self) -> dict:
        out = {key: np.array(value) for key, value in self.log.items()}
        n = len(out["t"])
        out["barriers"] = {name: np.array(v + [np.nan] * (n - len(v))) for name, v in self.barrier_log.items()}
        out["margins"] = dict(self.barrier_margins)
        out["events"] = list(self.events)
        return out
