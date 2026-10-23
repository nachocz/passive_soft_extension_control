# Actuation and Control of Passive Tendon-Driven Soft Extensions Through Rigid Manipulator Motion

Code, interactive pages, and video for the paper *Actuation and Control of Passive Tendon-Driven Soft Extensions Through Rigid Manipulator Motion*.

A passive soft extension is mounted at the end effector (EE) of an industrial manipulator, and its tendons are anchored to arm links upstream of the EE. Joint motion changes the exterior spans `ℓᵒᵢ` of the tendons and therefore their tensions, which bend the soft extension, so the manipulator actuates the soft extension with its own joints. When the task constrains the EE, for example a position constraint or a laparoscopic fulcrum, the controller regulates the tip with the null-space motion of the EE constraint: a damped least-squares control law on a differential model that combines the analytic arm and tendon kinematics with a span-to-deformation map identified from probing motions, and a quadratic program with control barrier functions for obstacle clearance, the fulcrum, and the identified span domain. The analysis bounds the rank of the tendon-transmitted deformation under an EE constraint and shows that a weighted tip-error function does not increase under the nominal control law.

This repository contains:

- **Interactive pages** (`docs/`): the model, the controller in closed loop on the simulated UR10 of the paper (scenarios S1–S3), and the analysis, with the equations of the paper next to each simulation.
- **Video** (3 min): [`docs/video/softext_video.mp4`](docs/video/softext_video.mp4), also on the overview page: the mechanism, the simulations, and the six experiments on the real setup.
- **Python implementation** (`softext/`, with numpy; matplotlib for the figures): the simulated UR10 with its Cosserat-rod soft extension and two elastic tendons, the differential model, the identification from probing motions, the EE constraints, the null-space control law, the barriers and the QP, and the quantities of the analysis, in the notation of the paper.
- **Scripts that reproduce the simulation results**: Figs. 7 and 8, the identification errors of Sec. VI-A, and Proposition 1 at the initial configuration.

## Interactive pages

| Page | Content |
|---|---|
| `index.html` | summary, Fig. 1, video, the results figures, citation |
| `model.html` | Sec. III: span kinematics (2)–(4), soft response (5)–(7), tip velocity (8)–(9), identified model (10)–(11); move the joints by hand or along the null space of an EE constraint, follow the spans, the tensions, and the transport and deformation terms of the tip velocity, and identify the span map from probing motions |
| `control.html` | Secs. IV and VI: EE constraints (12)–(15), tip task (16)–(18), barriers and QP (19) in closed loop on S1–S3; change the EE constraint (also a fulcrum at a finite aperture, as on the real setup), the model, the gains, the barriers, and the noise, and add targets |
| `analysis.html` | Sec. V: the bound (22) on the rank of the constrained deformation Jacobian with movable anchors and slack tendons; the descent of `V` under the nominal control law (25) with the exact or identified model and the damped-inverse or transpose filter (Fig. 7); a closed-loop run in which active barriers stop the descent |

To open them, run `python run.py` (Python standard library only), which serves `docs/` at http://127.0.0.1:8765 and opens the browser, or open `docs/index.html` directly. In the top bar, ◐ switches between the light and dark themes, and ♪ plays optional background music (credits in [`docs/audio/CREDITS.md`](docs/audio/CREDITS.md)).

## Reproducing the simulation results

```bash
pip install -r requirements.txt
```

Run the scripts from the repository root. The figures are written to `outputs/figures/` (PDF and PNG).

| Result | Script | Time |
|---|---|---|
| Fig. 7, convergence of the nominal controller with the exact and the identified model (Sec. V-B) | `python figures/fig7_stability.py` | 20 s |
| Fig. 8, the scenarios S1–S3 (Sec. VI) | `python figures/fig8_scenarios.py` | 1 min |
| Identification: validation error 1.5 mm with a probing range of ±10 mm, 5.5 mm with ±22 mm (Sec. VI-A) | `python examples/identify.py` | 5 s |
| Proposition 1 at the initial configuration, for the three EE constraints, anchors on the EE (Corollary 1), and a slack tendon | `python examples/rank_bound.py` | 2 s |

The plant of `softext/` is the simulated UR10 of the paper: at the initial configuration its spans, tensions, and tip position agree with the paper's simulator to 10⁻⁸, and `assets/span_map.json` is the span map that the paper's simulator identified and its simulations used. The closed-loop simulations of the paper were run in a ROS 2 simulator built on the same plant, identified map, controller, and noise level; `softext.Simulation` runs the same loop without ROS (plant at 100 Hz, measurements at 20 Hz, control at 50 Hz). Its runs are therefore close to those of Sec. VI but not equal, because the noise realization and the message timing differ: all nine targets of S1 and all five of S2 are reached, with the EE within 0.11 mm and 0.04 mm of its constraint (0.18 mm and 0.04 mm in the paper), and in S3 the error settles 50 mm from T3 and 56 mm from T5 (48 mm and 55 mm in the paper). Fig. 7 does not depend on the noise and is reproduced: β between 0.88 and 0.92, `V` never increases, and the descent rate with the identified model stays above 2.3 s⁻¹.

`python -m unittest discover tests -v` (25 s) checks the plant against the paper's simulator, the Jacobians (4), (9), (12), and the barrier gradients against finite differences, the QP solver, Propositions 1 and 2 on the plant, the identification errors, and the outcomes of S2 and S3, and runs the JavaScript port of the pages against the package (with Node.js).

## Running the controller

```bash
python examples/run_scenario.py S2                         # laparoscopic fulcrum -> outputs/S2/summary.png
python examples/run_scenario.py S1 --exact                 # exact J_span of the plant instead of the identified model
python examples/run_scenario.py S2 --no-obstacle-barrier   # the soft extension then enters the polyp
python examples/run_scenario.py S2 --constraint aperture   # fulcrum at a finite aperture, imposed by barriers only
python examples/run_scenario.py S3 --identify              # with a span map identified by probe()
```

```python
from softext import Config, Simulation

sim = Simulation(Config(), "S2")              # plant, identified span map, controller, and noise of Sec. VI
logs = sim.run()                              # until the last target is reached or abandoned
print(logs["events"], logs["ee_deviation"].max())
```

The controller uses the joint angles, the measured tip and body points, the arm kinematics, the anchors and entry points of the tendons, and the identified span map. The parameters of the Cosserat rod and of the tendons are used only by the simulated plant.

## Paper and code

| Paper | Code |
|---|---|
| entry points and exterior spans (2), `J_ℓq` (3)–(4) | `plant.Plant.endpoints`, `plant.Plant.spans`, `model.span_jacobian` |
| tension linearization (5), `C_τ` (6), `J_span` (7) of the simulated rod | `model.soft_response`, `rod.Rod` |
| `J_transport` (8), `J_D` (9) | `model.transport_jacobian`, `model.distal_jacobian`, `model.differential` |
| identified model (10), body-point Jacobians (11), span domain `L`, online secant correction | `identification.SpanMap`, `identification.probe`, `identification.identify` |
| EE constraints and `J_c` (12), Table II | `constraints.EEConstraint` |
| `N_c` (13), restoring velocity (14), nominal command (15) | `constraints.projector`, `controller.Controller.command` |
| tip error (16), `A` (17), damped least squares (18), spectral filter (23)–(24) | `controller.Controller.command`, `controller.spectral_velocity` |
| barriers (Table III) and the QP (19) | `barriers`, `controller.Controller.command`, `qp.solve_qp` |
| `J_def` (20), the rank bound (22), Corollary 1 | `analysis.rank_report` |
| on-manifold control law (25), `V`, `V̇`, `β`, `ρᵢ` (26)–(32) | `analysis.nominal_run`, `analysis.descent` |
| simulation setup (Table IV), scenarios (Table V) | `simulation.Simulation`, `scenarios` |

## Simulation model and parameters

- The arm is a UR10 with standard DH parameters; `q0 = (0, −70, 95, −115, −90, 0)°`, with the tool axis pointing down. The frame `{B}` is DH frame 6.
- The soft extension is an inextensible, unshearable Cosserat rod of 320 mm and radius 9 mm (`E = 1.1 MPa`, `G = 0.4 MPa`) with piecewise-constant curvature on five sections, so its shape coordinates `z` have 15 entries; gravity is neglected, as in the simulations of the paper, so its shape in `{B}` depends on the exterior spans alone. Its equilibrium minimizes the elastic and tendon energy, which is convex in `z`, by Newton's method.
- Two tendons enter at `ᴮbᵢ = 5.5 mm · (cos φᵢ, sin φᵢ, 0)` with `φ = 0°, 90°`, run parallel to the centerline, and are anchored at 75 % of DH link 4 (wrist 1) and at 60 % of DH link 3 (forearm). Their tension is `kₐ,ᵢ eᵢ` with `kₐ,ᵢ = 260 N/m` for a stretch `eᵢ > 0`, zero when slack, and saturates at 14 N; the rest lengths give 1.5 N at `q0`.
- The parameters of the controller and of the barriers are those of the simulations (`config.yaml`): `k_t = 1.4 s⁻¹`, `k_c = 3.0 s⁻¹`, `ξ = μ = 0.04 m`, `w_c = 200`, joint rates up to 0.30 rad/s, a tip reference that moves to each target at 0.10 m/s, and the arrival criterion of 4 mm for 8 s. The obstacle clearance is computed to the polyline through the five body points predicted by the identified model, minus a constant margin of 12 mm.

## Repository layout

```
softext/       Python package (config, arm, rod, plant, model, identification, constraints, barriers, qp,
               controller, scenarios, simulation, analysis, plotting, runs, rng)
examples/      run_scenario.py, identify.py, rank_bound.py, export_web_data.py
figures/       fig7_stability.py, fig8_scenarios.py
tests/         checks of the model, the controller, the analysis, and the results; checks of the pages
assets/        span_map.json, the span map identified by the paper's simulator
docs/          interactive pages and video
tools/         build of the interactive pages (see tools/README.md)
config.yaml    parameters of the plant and of the controller
run.py         local server for docs/
```

Notes on editing and building the interactive pages are in [`tools/README.md`](tools/README.md).

## Citation

```bibtex
@article{softext,
  author  = {<Author 1> and <Author 2>},
  title   = {Actuation and Control of Passive Tendon-Driven Soft Extensions Through Rigid Manipulator Motion},
  journal = {IEEE Transactions on Robotics},
  year    = {<year>},
  volume  = {<volume>},
  number  = {<number>},
  pages   = {<pages>},
  doi     = {<doi>},
}
```

## Third-party files

- KaTeX (MIT license), `docs/vendor/katex/`.
- three.js r160 (MIT license), `docs/vendor/three/`.
- Background music: Erik Satie in public-domain recordings, and Kevin MacLeod (incompetech.com) under CC BY 4.0; attributions in `docs/audio/CREDITS.md`.

## Acknowledgment

The Python package, its JavaScript port, the interactive pages, and the scripts of this repository were developed with the assistance of Claude Code (Anthropic's Claude, model Claude Opus 5.5), under the direction of the authors, who checked the code and the results it reproduces and are responsible for the content of the repository.
