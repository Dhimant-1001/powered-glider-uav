# Powered Glider UAV — EC4.402 Project 18

Hand-launched, high-aspect-ratio powered glider (1.40 m span, AR 10) designed to fly within a 100 m × 20 m field.

## Where to look
- `design/07_final/design_spec.md` — **final design (single source of truth)**
- `design/07_final/three_view.png` — dimensioned three-view
- `design/07_final/final_analysis.py` — reproduces all numbers (AeroSandbox + NeuralFoil)
- `design/08_bom/Powered_Glider_BOM.xlsx` — component list (Robu links, priority colour-coded)
- `design/01_theory` … `05_mission` — stage-1 studies: course theory, construction & mass model, airfoil selection, propulsion sizing, field/mission constraints
- `design/00_critique_stage1.md` — review reconciling the stage-1 studies
- `design/06_concepts/` — partial concept work (superseded by 07_final)
- `projectSpecification.png` — project brief

## Key numbers
Span 1.40 m · AR 10 · S 0.196 m² · SD7037 wing · AUW ≈ 600 g · CG 63.5 mm aft of LE (SM 12 %) · V_stall 6.9 m/s · V_op 9 m/s · L/D max ≈ 15 · A2212 1000KV + 9×5 folding prop · 3S LiPo · ~14 min endurance.

Run scripts with Python 3 + `aerosandbox==4.2.10`, `neuralfoil`, numpy, matplotlib.
