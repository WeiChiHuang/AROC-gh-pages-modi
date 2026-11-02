AROC website numerical results

Source commits:
AROC: 6ba0295754d935b2bf540fab904ebc235a2d7a5c
CORA v2026.0.1: 6cb79424fc4085c1af10a13236bcca9776887683
YALMIP R20250626: 61160192b05af3772980fad7137825ff1ea64eb2
Original website: 33e8493d46de2154ca56572eb7bcd61a604d4e61

Environment: MATLAB R2024b Update 9, MATLAB Optimization and Control System
Toolboxes; SeDuMi 1.3. MOSEK and CasADi were not installed for these runs.
No MATLAB path or preferences were permanently changed.

Setup: add the AROC, CORA, YALMIP, and SeDuMi directories to the MATLAB path.
In a LOCAL COPY of AROC, change the solver setting in
terminalRegion/linSys/src/setOptsInternal.m from mosek to sedumi.
Then run reproduce_website_results.m. The output directory is
aroc-website-results under the current directory. No model constraints are
changed by the backend substitution.

Linear MPC is adapted from unitTests/testLinSysMPC.m: same terminal polytope,
x0=[3;-3], horizon=8, N=40, with realTime=false for repeatable offline export.
The terminal region is not newly synthesized for the MPC example; the
separate terminal-region example computes a different set.
The input and final simulation-point checks are asserted.

Terminal-region example: fixed zonotope, N=10, dt=0.1, 10 generators,
state box +/-[10;4], input +/-1, disturbance +/-[0.1;0.1].
Ten sampled extreme-disturbance trajectories are simulated. Assertions
check their state/input constraints, initial-region membership, and final
target-set membership at tolerance 1e-2. Sampling is an empirical check,
not an independent formal proof. See run-report.json for outcomes.

The PNG and SVG files are exported from the actual MATLAB computations.
MAT files retain result objects, parameters, options, and sampled trajectories.
Workflow diagrams elsewhere on the site are labelled schematics.
No real-time, all-benchmark, or all-algorithm validation claim is made.

Conformant synthesis: run reproduce_conformance.m with AROC/CORA on the path.
Fits three bundled platoon trajectories using interval uncertainties and
MATLAB quadratic programming, group=6, measErr=true, mu=0.1. The first
fitting trajectory passes full-state containment at 101/101 matched time
stamps (tolerance 1e-6). This is not held-out validation.
Reachable-set timePoint includes t=0; measurements must be aligned using
their time values, not shifted array indices.
