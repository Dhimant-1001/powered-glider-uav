# Design review: contradictions, checks and design inputs for integrated sizing

The five specialist packages cannot all be right at once. The mission study's target wing loading of **16 N/m² cannot be built** with the recommended printed-PLA-Aero airframe plus the propulsion the propulsion study chose. The weight model puts the realistic floor at **about 19–21 N/m² for spans around 1.8–2.0 m**. The design inputs in section 4 move the design point to that floor and tighten the turn rules so the field constraint still holds.

I checked several numbers by re-running the specialists' own models with reconciled inputs. The scripts are `/private/tmp/claude-501/-Users-dhimantbhuva-Documents-Intro-UAV-Project/ed8a1eff-d373-436e-a669-034ae3af7105/scratchpad/sweep.py`, `.../scratchpad/cand.py` and `.../scratchpad/prop.py`. No project files were changed.

---

## 1. Contradictions between the specialists

**C1. Wing loading target (the main conflict)**
- Mission study: 16 N/m² target, 14.8–17.8 N/m² window.
- Construction study: says about 22 g/dm² (21.6 N/m²) "is what the field needs". That is wrong by the mission study's own rule: 21.6 N/m² only fits with 45° bank, a 16 m turn width and CL_max 1.0, or with CL_max of at least 1.1.
- Propulsion study: base case is 21.8 N/m², yet it advises 15–17 N/m² for turns.

**C2. The weight model makes 16 N/m² unreachable**
- With the recommended construction (printed PLA Aero, carbon spar, Depron tails), full avionics (F405-WTE flight controller, GPS, 4 servos) and the mass model's original light propulsion (30 g motor, 3S 450 mAh pack), 16 N/m² needs a span of 2.48 m and 1005 g at AR 10.
- With the propulsion study's hardware it needs 2.9 m and about 1.38 kg.
- The mission study's example of 450 g on a 1.74 m span has no support in the weight model. Even the original light assumptions give **591 g (21.1 N/m²)** at 1.74 m, AR 11.

**C3. Propulsion weight**
- The mass model assumed motor 30 g, battery 42 g (3S 450), ESC 12 g and prop/spinner 16 g, about 100 g in total.
- The propulsion study chose A2212 (50 g), 9×5 folding prop (18 g), 3S 1000 mAh (85 g) and Skywalker 20A (19 g), plus about 6 g of mount: 172–178 g.
- That is **+72–78 g**, about +2.5 N/m² at 0.3 m².
- None of the construction study's AUW tables (543–575 g) include it. Corrected, the 1.6 m / AR 10 case is about 640 g, not 565 g.

**C4. CL_max is used inconsistently**

| Source | CL_max used |
|---|---|
| Construction | 1.2 |
| Propulsion | 1.1 |
| Mission | 1.0 |
| Slides' worked example | 1.6 (not applicable here) |

- The section data (NeuralFoil) give Cl_max 1.29–1.31 at Re 60–80k. XFOIL tends to over-predict this.
- The realistic 3D clean value with washout is **1.0–1.1**. Size with 1.0 and treat 1.1 as upside only.

**C5. Cruise CL versus airfoil choice**
- At a realistic 19–20 N/m² and 8 m/s, cruise CL is **0.54–0.57**. Turns at 45° and 8.2 m/s need CL of about 0.75–0.8.
- The airfoil study tuned its pick (S7075) for section Cl 0.9–1.05. S7075 has its documented drag hump exactly at Cl 0.4–0.8.
- Its own polars (Ncrit 9) show SD7037 has lower drag in the band that matters:

| Re | Cl | S7075 Cd (×1e4) | SD7037 Cd (×1e4) |
|---|---|---|---|
| 60k | 0.5 | 285 | 256 |
| 60k | 0.6 | 290 | 252 |
| 60k | 0.7 | 280 | 243 |
| 60k | 0.8 | 259 | 233 |
| 80k | 0.5 | 228 | 194 |
| 80k | 0.6 | 223 | 187 |
| 80k | 0.7 | 210 | 181 |
| 80k | 0.8 | 194 | 179 |

- That is **7–15 % lower drag**, a later stall (11.2° vs 10.5°), and UIUC wind-tunnel data exist for SD7037 to check XFLR5 against.
- **I recommend SD7037 as the primary airfoil.** S7075 becomes the alternate. SA7038 is effectively equivalent to SD7037.

**C6. Zero-lift drag (CD0)**
- Propulsion used 0.025; mission used 0.030.
- A rough buildup gives about 0.030–0.035: wing profile drag of about 0.024–0.027 at cruise Cl and Re 70k, plus 0.006–0.008 for pod, boom, tails and folded prop.
- Use 0.032 until AeroBuildup and XFLR5 replace it. That gives a parabolic L/D_max of about 15; expect 11–13 in reality.

**C7. Tail section**
- Construction: 3 mm Depron flat plate.
- Airfoil study: HT14 (7.5 %, printed).
- **Resolution:** Depron plate with a rounded leading edge, analysed as the 3 % flat plate. HT14 printed is the fallback. It adds about 15 g at the tail, which needs roughly 30 g of nose mass to balance.

**C8. Tail volume coefficients**
- Mass model: V_H 0.45. Slides: sailplane value 0.50, aircraft table 0.66–1.1.
- **Use V_H 0.50–0.60 and V_v 0.025–0.035.** The higher V_v is because a long wing has strong adverse yaw and rudder coordination is planned.

**C9. Thrust-to-weight**

| Source | T/W |
|---|---|
| Theory | 0.5–0.7 |
| Mission | ≥ 0.6 (target 0.6–0.8) |
| Propulsion feasibility rule | ≥ 0.75 |
| Chosen combo | 1.35 at 0.5 kg |

- The combo is not wrong; the margin is good for a beginner's hand launch. But the 3S 1000 mAh pack is oversized for the mission (see section 4, propulsion).

**C10. Span limits**
- Mission caps span at 1.8 m for roll rate.
- The weight model needs **1.8–2.0 m** for ≤ 20 N/m² unless the airframe gets lighter.
- 1.8 m is the practical sweet spot. 2.0 m needs about 20° aileron throw to hold roll rate.

**C11. Tip chord**
- The airfoil study wants a tip chord of at least 0.11 m (tip Re ≥ about 45k).
- AR 12 at 1.6 m with taper 0.6 gives about 0.10 m.
- Use taper ≥ 0.65, or a double trapezoid, and AR ≤ 11 at spans ≤ 1.8 m.

---

## 2. Numbers that are wrong, unsourced or weak

**Error in the mission file:** it says pb/2V = 0.08 is about 55°/s at 8 m/s and 1.6 m span.
- The correct value is p = 0.08 × 2 × 8 / 1.6 = 0.80 rad/s = **46°/s**. 55°/s corresponds to pb/2V of about 0.096.
- The mission study's own aileron table (43 and 57°/s) is correct.
- Rolling 0 to 40° in 0.8 s actually needs pb/2V of about 0.09–0.10, so the 20–25° aileron throw recommendation still stands.

**Checks that passed:**
- Turn radius 6.83 m at 7.5 m/s and 40° bank.
- Stall speed 5.42 m/s at 16 N/m² with CL_max 1.0.
- Gust load increment 0.52 at 1 m/s.
- Propulsion: pitch speed 17.9 m/s; cruise drag 0.31 N (0.5 kg at L/D 16).
- Construction: shell areal density 8.06 g/dm².
- Theory: 39° bank for R 8 m at 8 m/s.

**Assumptions flagged as weak:**
- **PLA Aero density 0.65 g/cm³.** This drives everything. At 0.85 the aircraft gains about 80 g. A density test print is mandatory before freezing the span.
- **The 3S 650 mAh pack mass of 56 g** is my assumption.
- **Throw speed of 7 m/s** is unmeasured.
- **Spoileron approach L/D of about 7** is unmeasured and cannot be checked in XFLR5. It needs flight testing.
- **Grass friction μ 0.4** is assumed.
- **Propulsion thrust and current** are ±15 %.
- **NeuralFoil drag** is probably 10–30 % optimistic compared with a wind tunnel.
- **The 3DLabPrint Bungee Prop (15.1 g/dm²) is not a fair comparison.** It carries a receiver and 2 servos only, with no flight controller or GPS. Our airframe carries about 35–45 g more in avionics.

**Re-run of the propulsion model with A2212 + 9×5 folding prop:**

| AUW | Battery | Wing loading | T/W | Cruise current | Beginner endurance | Physics endurance |
|---|---|---|---|---|---|---|
| 0.60 kg | 3S 1000 mAh | 20 N/m² (1.8 m, AR 11) | 1.12 | 1.16 A | 22.7 min | 39.6 min |
| 0.60 kg | 3S 650 mAh (assumed 56 g) | 20 N/m² | 1.12 | 1.16 A | 14.7 min | 25.4 min |
| 0.65 kg | 3S 1000 mAh | 21.6 N/m² | 1.04 | 1.23 A | 20.7 min | 37.1 min |

---

## 3. Missing before integrated VLM sizing (wing + tail + CG + stability in AeroSandbox)

1. **Layout table:** a position (x, z) for every mass item, giving the CG and the inertias Ixx, Iyy and Izz. XFLR5's stability analysis needs these. Also a component layout in the pod: motor, ESC, battery bay with ±30 mm travel, flight controller, GPS, and where the wing saddle sits.
2. **Wing and tail incidence, washout and dihedral:**
   - wing incidence about +2–3°
   - decalage about 1.5–3° so the tail carries a downward load at trim (CL_t < 0, as the slides require)
   - washout 1.5–2.5°
   - dihedral or polyhedral: none was specified, and spiral and Dutch-roll checks need it
3. **Static margin target and a neutral-point method.** Use VLM / XFLR5 rather than the theory file's formula, which uses 0.25 for the aerodynamic centre.
4. **Control-surface geometry and hinge stations**, including the elevator and rudder deflections needed to trim at CL 0.8 and to hold the 45° turn.
5. **A CD0 estimate** from an AeroBuildup of pod, boom and tails at the real Re, plus a trim-drag check.
6. **A tail Re and stall check:** about 33–47k at 80–100 mm chord. Confirm the flat-plate or HT14 tail does not stall at maximum elevator in a 45° turn at the slowest speed.
7. **A propeller-wash and thrust-line moment check:** 1–2° of downthrust, 2° of right thrust.
8. **A coupled mass–sizing loop:** span and S drive mass, mass drives W/S, W/S drives the turn and stall constraints. The specialists' models have never been run together.
9. **Questions only the student can answer** (see section 5).

---

## 4. Consolidated design inputs for the next stage

**Mass and loading**

| Input | Value |
|---|---|
| AUW | planning 600 g; acceptable range 540–650 g; hard cap 650 g |
| W/S | target 18–20 N/m² (planning 19.5); hard ceiling 21 N/m²; 16 N/m² only possible with a ≥ 2.2 m span or no flight controller, so not pursued |

**Turn rule (replaces the mission study's 40°/14 m rule)**
- 45° bank (flight-controller bank limit 45°), turn speed 8.2–8.5 m/s.
- Turn radius 6.9–7.4 m; 180° turn width 14–15 m including roll-in.
- Margin: turn speed ≥ 1.15 × stall speed in the turn.
- Allowable W/S: 19.6 N/m² for 14 m width at CL_max 1.0; 21.6 at CL_max 1.1; 22 for 16 m width at CL_max 1.0.
- The gust margin is thin at 20 N/m², so fly only in wind below 2–3 m/s with crosswind ≤ 1 m/s.

**Speeds and lift coefficients**

| Item | Value |
|---|---|
| CL_max, 3D clean | 1.0 for sizing (1.1 upside); must be confirmed in XFLR5 by the local Cl reaching Cl_max at about 70 % semispan |
| Flaperon droop | 5–10° for launch and slow flight, worth about +0.1 (assumed) |
| V_stall | 5.8–6.1 m/s (CL_max 1.0) |
| Launch | at least 7–7.5 m/s with motor on |
| V_cruise | 8.0–8.5 m/s at CL 0.50–0.57 |
| V_approach | about 7.5 m/s |
| Touchdown | about 6.3 m/s |
| Operating CL band | 0.5–0.8 |

**Airfoils**
- Wing: **SD7037** (9.2 %), using the blunt-trailing-edge file `sd7037_te05.dat` for the printed wing. Alternate S7075.
- Tail: 3 mm flat plate (Depron); alternate HT14.

**Reynolds numbers**
- Wing: root about 85–100k, tip about 55–65k.
- Tail: about 35–50k.
- Analyse at Ncrit 9 and 7.

**Wing geometry**
- Baseline: b = 1.8 m, AR 11, S ≈ 0.295 m², c̄ ≈ 0.164 m.
- Taper 0.65, or a double trapezoid: rectangular to 40 % semispan, then taper to 0.6. Root chord about 0.20 m, tip chord ≥ 0.12 m.
- Washout 2°.
- Polyhedral: 0° inner, 5–7° on the outer panels from about 60 % semispan (a segment joint). That is about 3–4° effective, chosen so the spiral mode is neutral or slightly stable.
- Explore: span 1.7–2.0 m × AR 10–12, keeping tip chord ≥ 0.115 m.

**Construction**
- Printed shell with a carbon spar and carbon boom, as the construction study recommends, but **only if a test coupon proves density ≤ 0.55–0.6 g/cm³**.
- At 0.65 g/cm³ the 1.8 m aircraft is about 650–675 g (22–23 N/m²): too heavy.
- Fallback order:
  1. hybrid printed leading-edge box with Depron skins (about 629 g at 1.8 m / AR 11, 21 N/m²)
  2. LW-PLA, imported (579 g, 19.3 N/m²)
- Masses come from the mass model with A2212, 3S 650 and Skywalker 20A, V_H 0.55, fuselage length 0.62 b, 5 % contingency included.
- Tail: Depron 3 mm. Boom: 8×6 carbon. Spar: 6×4 stepping down to 4×2.

**Propulsion**
- **A2212/13T 1000 KV + 9×5 folding prop on a 30–35 mm spinner + Skywalker 20A.**
- **Battery: 3S 650 mAh instead of the 1000 mAh pack.** It saves 29 g, gives about 15 min of beginner endurance against about 10 min needed, and keeps T/W about 1.1 at 0.6 kg.
- Lighter option: EMAX CF2822 (−11 g; Indian stock not confirmed).
- Tractor layout, ESC brake on, 1–2° downthrust and 2° right thrust, motor mount printed in PETG or ASA.

**Avionics**
- Matek F405-WTE on ArduPilot, Matek M10Q GPS, and **an airspeed sensor (about 6 g)**. The airspeed sensor is strongly recommended because the deliverables include comparing XFLR5 predictions with flight.
- 4 × 5 g servos (ES9051 or ES9052MD), all mounted in the pod or wing, none at the tail.

**Control surfaces**

| Surface | Span / chord | Throw |
|---|---|---|
| Ailerons (one servo each; flaperon droop 5–10°, spoileron up 15–25°) | 50–95 % semispan, 25 % chord | ±20–25°, about 50 % differential |
| Elevator | 35–40 % of tail chord | ±20° |
| Rudder | 35–40 % of fin chord | ±25° |

- Roll target: pb/2V ≥ 0.09.
- Rudder is mixed with aileron for turn coordination.

**Tail type: conventional (cruciform-low)**
- Flat Depron stabiliser on top of the boom, fin above it, small printed clamp on the 8×6 boom.
- Why not the alternatives:
  - It maps directly onto the slides' V_H and V_v equations (L14) and is the simplest to set up in XFLR5.
  - Printing is trivial.
  - On a belly landing the pod takes the impact and the boom-mounted stabiliser stays above the grass.
  - **V-tail** saves about 10 g and a little drag, but couples roll and yaw for a beginner, needs a printed V-joint that breaks on cartwheels, and both surfaces are working at Re about 35k.
  - **T-tail** needs a stiff, heavier fin, a pushrod run up the fin and has flutter risk with Depron.
- Sizing: V_H 0.55 (range 0.50–0.60), V_v 0.030 (range 0.025–0.035), tail arm about 0.65–0.70 m.
- At 1.8 m that gives S_h ≈ 3.8 dm² and S_v ≈ 2.3 dm², with the tail trimmed to CL_t < 0.

**Stability targets**
- Static margin 10–15 % of c̄ with the battery adjusting CG over ±5 mm.
- Cm_α < 0; Cn_β > 0 and Cl_β < 0 (from XFLR5 stability analysis).
- Spiral time-to-double > 20 s or stable; Dutch roll damping ratio > 0.1.

**Air and operations**
- ρ 1.09 kg/m³ (hot day 1.06), ν 1.7e-5 m²/s.
- Climb 2–3 m/s at about 15–20°; pattern altitude 15–20 m.
- Racetrack along the 100 m axis.
- Landing from about 5 m on final, using spoilerons.

---

## 5. Questions for the student

1. **May the aircraft fly over the sidelines (above the 20 m width) at altitude, or must it stay strictly above the field?** This is the single biggest design driver. Relaxing it allows a lighter-margin design, 16–20 m turns and a span of about 1.5 m.
2. How long must the altitude hold last, and is there a minimum total flight time?
3. Is importing LW-PLA acceptable? And can you print and weigh a 100 × 100 mm single-wall PLA Aero test panel now to measure its density?
4. Is GPS, or an airspeed sensor, required or allowed? What is the budget cap in ₹?
5. Which radio transmitter do you have (ELRS or other)? This decides the receiver.

---

## 6. Uncertainties to carry forward
- **Airframe density:** ±80 g.
- **3D CL_max:** ±0.1, which moves the allowed W/S by ±2 N/m².
- **Thrust and current:** ±15 %.
- **Section drag:** probably optimistic by 10–30 %.
- **Throw speed:** about 6–8 m/s.
- **Margin overall:** the design closes with only about 1–2 N/m² to spare. Any mass growth over about 30 g must be paid for with span, about +0.06 m per 30 g at AR 11.
