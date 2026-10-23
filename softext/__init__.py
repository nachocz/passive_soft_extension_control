"""Actuation and control of passive tendon-driven soft extensions through rigid manipulator motion.

Reference implementation of the model (Sec. III), the controller (Sec. IV), and
the analysis (Sec. V) of the paper, with the simulated UR10 plant of Sec. VI.
"""
from .config import Config
from .plant import Plant, State
from .identification import SpanMap, identify, paper_span_map
from .constraints import EEConstraint
from .barriers import Fulcrum, Obstacle
from .controller import Controller
from .scenarios import SCENARIOS, scenario
from .simulation import Simulation

__all__ = ["Config", "Plant", "State", "SpanMap", "identify", "paper_span_map", "EEConstraint", "Fulcrum",
           "Obstacle", "Controller", "SCENARIOS", "scenario", "Simulation"]
