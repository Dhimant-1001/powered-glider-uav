# Propulsion sizing: small hand-launched powered glider

Files in this folder:
- `propulsion_model.py`: the parametric model. Run `python3 propulsion_model.py --mass 0.5 --span 1.5 --AR 10 --CD0 0.025 --e 0.85`.
- `run_output.txt`: console output from the default run. `propulsion_results.json` has every number.
- `prop_data/`: measured propeller data from the UIUC Propeller Data Site.
- `power_required.png`, `thrust_vs_drag.png`, `rate_of_climb.png`: plots.

Site air: ρ = 1.09 kg/m³ (Hyderabad, about 540 m, 30 °C). The hot-day check uses 1.06. g = 9.81.

---

## 1. Recommendation

| | **PRIMARY** | **ALT-1 (better quality, same performance)** | **ALT-2 (lighter motor)** | **Fallback without a folding prop** |
|---|---|---|---|---|
| Motor | **A2212/13T 1000 KV** outrunner, 3.17 mm shaft, ~50 g | SunnySky X2212-III 980 KV (fixed-wing version), 57 g | EMAX CF2822 1200 KV, 39 g | A2212/13T 1000 KV |
| Prop | **9×5 folding** (glider blades on a 30–35 mm hub and spinner, 3.17 mm collet), ~18 g | 9×5 folding | 9×5 folding | APC-type 9×6E fixed (≈14 g) |
| Battery | **3S 1000 mAh 30C LiPo** (Orange), 85 g, XT30/XT60 | 3S 1000 mAh 30C | 3S 1000 mAh 30C | 3S 1000 mAh 30C |
| ESC | **Hobbywing Skywalker 20A** (2–3S, 5 V/2 A linear BEC, programmable brake), 19 g | Skywalker 20A | Skywalker 20A | Skywalker 20A |
| Propulsion + battery mass | **172 g** (+ about 6 g for the printed mount and screws) | 179 g | 161 g | 168 g |
| Static thrust at 11.4 V (model) | **673 g** (range 570–770 g) | 652 g | 729 g | 659 g |
| Static current, battery side | **9.2 A** (10.3 A on a full pack) | 8.9 A | 12.0 A | 10.6 A |
| Static electrical power | 101 W | 97 W | 130 W | 116 W |
| Static rpm | 8 470 | 8 340 | 8 820 | 8 210 |
| Geometric pitch speed / zero-thrust speed | 17.9 / 22.9 m/s | 17.7 / 22.6 | 18.7 / 23.8 | 20.8 / 25.2 |
| Static T/W at 0.50 kg (at 0.65 kg) | **1.35** (1.04) | 1.30 | 1.46 | 1.32 |
| Approx. price | motor ₹320–400, prop+spinner ₹800–1 200 (imported), ESC ₹880–990, battery ₹970 | motor ≈₹2 240 + GST | motor ≈₹900–1 200 | prop ₹150–450 |

**Why the primary.** The A2212/13T 1000 KV is the most widely stocked brushless motor in India: Robu, Robobazar, Robodo, Flyrobo and Atomlab all list it at ₹320–600. On 3S with a 9×5 folding prop it gives T/W ≈ 1.35 at 0.5 kg. Its static current of 9.2 A stays well inside the motor's 12–13 A/60 s rating, the ESC's 20 A rating (margin 2.2×) and the battery's real C capability (9.2 A is about 9C).

**Same airframe, other props.** A 10×6 folding prop on the same motor gives 900 g at 12.6 A. Use it only if the AUW grows past 0.65 kg.

**2S option, if wanted.** A2212/10T 1400 KV + 10×6 folding + 2S 1300 mAh gives 670 g at 13.3 A. That needs a 30 A ESC, and the extra current brings no benefit.

**Rejected combos:**
- A2212-1000 on 2S: 461 g. T/W is too low once the ±15 % model uncertainty is applied.
- CF2822 on 2S with 9×5: T/W 0.75.
- 9×4 folding: pitch speed 14.8 m/s is below 2 × V_cruise.
- A2212-1400 on 3S with 9×5: 17.8 A, which exceeds the motor's 16 A rating.

---

## 2. Requirements: equations and numbers

Drag polar (whole aircraft): `CD = CD0 + CL²/(π e AR)`, `CL = 2 n W /(ρ V² S)`.

| Quantity | Equation |
|---|---|
| Level-flight thrust and power (lecture L13 eq. 11–12) | `T = D = ½ρV²S·CD`, `P_req = D·V` |
| Stall speed | `V_s = sqrt(2W /(ρ S CLmax))`, with CLmax = 1.1 for the whole aircraft at low Re |
| Min-power speed (best endurance) | `CL = sqrt(3 CD0/k)`, `k = 1/(π e AR)` |
| Best L/D | `L/D_max = ½ /sqrt(CD0·k)` |
| Level turn of radius R | `tan φ = V²/(g R)`, `n = 1/cos φ`, `V_s,turn = V_s·sqrt(n)`, `D_turn = D(V, n)` |
| Steady climb (exact, lift = W cos γ) | `sin γ = (T − D(V, n = cos γ))/W`, `ROC = V sin γ`, `P_climb = T·V` |
| Hand launch at V_L = max(1.2 V_s, 6 m/s) | `T(V_L) ≥ D(V_L) + 0.3·m·g + W·sin 5°` (≥ 0.3 g forward acceleration while starting a 5° climb) |
| Static thrust needed | `T0 ≥ max(0.7 W, T_climb/(1 − V/V0), T_launch/(1 − V_L/V0))`, with linear thrust lapse to V0 = 2.5 V_cruise |
| Pitch speed | `V_p = rpm/60 · pitch`. Guideline: ≥ 2–2.5 V_cruise |
| Propeller | `T = CT(J) ρ n² D⁴`, `P_shaft = CP(J) ρ n³ D⁵`, `J = V/(nD)` (UIUC measured CT/CP) |
| Motor (3-constant) | `I = (d·V_oc − ω/K_v)/(R_m + R_esc + d² R_batt)`, `Q = (I − I0)/K_v`, `K_v = 0.88·KV·2π/60` |
| Battery | `I_batt = d·I + I_avionics`, `t_end = 0.8·C_mAh·3.6 / I_avg` (80 % usable) |
| Endurance theory (L13 eq. 23–24) | `t_e = E·(L/D)/(m g V)` |

Representative airframes (ρ = 1.09, CD0 = 0.025, e = 0.85, CL_cruise = 0.6, climb 3.5 m/s):

| Design | S [m²] | W/S [N/m²] | L/D max | V_s | V_cruise | D_cruise | P_aero cruise | 7 m turn at V_c | T_climb 3.5 m/s | P_aero climb | **T0 required** | pitch speed needed |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| D1 0.40 kg, b 1.3, AR 10 | 0.169 | 23.2 | 16.3 | 6.2 | 8.4 | 0.25 N | 2.1 W | 46°, n 1.44 | 1.69 N | 16 W | 319 g (T/W 0.80) | 17–21 m/s |
| **D2 0.50 kg, b 1.5, AR 10** | 0.225 | 21.8 | 16.3 | 6.0 | 8.2 | 0.31 N | 2.6 W | 44°, n 1.39 | 2.17 N | 20 W | **410 g (T/W 0.82)** | 16–20 m/s |
| D3 0.55 kg, b 1.6, AR 11 | 0.233 | 23.2 | 17.1 | 6.2 | 8.4 | 0.34 N | 2.8 W | 46°, n 1.44 | 2.32 N | 23 W | 438 g (0.80) | 17–21 m/s |
| D4 0.65 kg, b 1.6, AR 10, CD0 .03, e .75 | 0.256 | 24.9 | 14.0 | 6.4 | 8.7 | 0.48 N | 4.2 W | 48°, n 1.49 | 2.75 N | 28 W | 520 g (0.80) | 17–22 m/s |
| Low-W/S: 0.45 kg, b 1.7, AR 10 | 0.289 | 15.3 | 16.3 | 5.1 | 6.8 | 0.28 N | 1.9 W | **8 m turn at 30.5°, n 1.16, 1.25 × V_s,turn** | 2.27 N | 18 W | 428 g (0.95) | 14–17 m/s |

**Field constraint (for the aerodynamics and layout team).** A 180° turn inside a 20 m wide ground means R ≈ 7–8 m: a 14–16 m circle plus 2–3 m clearance on each side. Flown at 1.25 × the turn stall speed with CLmax 1.1, that limits wing loading:

| R | bank 30° | bank 35° | bank 40° |
|---|---|---|---|
| 7 m | W/S ≤ 13.2 N/m² (V = 6.3 m/s) | ≤ 15.1 (6.9 m/s) | ≤ 16.9 (7.6 m/s) |
| 8 m | ≤ 15.1 (6.7 m/s) | ≤ 17.3 (7.4 m/s) | ≤ 19.4 (8.1 m/s) |

So for gentle 30–35° beginner turns the wing loading should be about **15–17 N/m²**, not 22–25. For example, 0.45 kg on S ≈ 0.27–0.29 m² (b 1.6–1.7 m at AR 9–10). This is an aerodynamics and layout decision. **Propulsion is unaffected:** the primary combo has T/W ≥ 1.0 for every case above.

**Why T/W ≈ 0.8–1.4 static for a beginner hand launch.**
1. The derived floor is 0.75–0.95. Thrust lapse means the 9×5 prop gives only about 75 % of static thrust at the 7 m/s launch speed. The aircraft still needs 0.3 g of forward acceleration plus a 5° climb with zero altitude margin.
2. A beginner's throw is often weak or nose-high. Extra thrust is what gets the aircraft out of that before it hits the ground.
3. The model is uncertain by ±15 % (Section 4), and a hot day costs about 2 %. The pack sags from 4.1 to 3.6 V/cell, which costs 20 % of thrust.

T/W 1.35 nominal keeps T/W ≥ 1.0 in the worst case (0.65 kg). Excess thrust costs nothing because throttle is proportional. Launch at about 70–75 % throttle, which gives 400–440 g, and use full throttle only to recover. An upper bound of about 2 is set by torque roll and motor and ESC current.

**Climb within the field.** With the primary combo at 0.50 kg:
- A practical 25° climb at 9.4 m/s needs 73 % throttle at 4.4 A battery current. ROC is 4.0 m/s.
- It reaches 20 m AGL in 4.6 s and **39 m of ground distance**, well inside the 100 m length.
- At full throttle the maximum ROC is about 9 m/s at 12 m/s (γ ≈ 49°). Do not do this as a beginner. It is the reserve for recovery.

---

## 3. Performance of the primary combo (A2212-1000 + 9×5 folding + 3S 1000)

Throttle sweep at 11.4 V (3.8 V/cell, mid-discharge), ρ 1.09. These are model values, uncertain by ±15 %:

| Throttle | rpm (static) | Static thrust [g] | I_batt [A] | P_el [W] | g/W | Thrust at V_c 8.2 m/s [g] | I at V_c [A] | η_prop | η_motor |
|---|---|---|---|---|---|---|---|---|---|
| 0.4 | 3 709 | 129 | 1.33 | 15 | 8.6 | 40 | 1.16 | 0.65 | 0.66 |
| 0.5 | 4 581 | 197 | 1.94 | 22 | 9.0 | 97 | 1.84 | 0.68 | 0.75 |
| 0.6 | 5 423 | 276 | 2.81 | 32 | 8.7 | 165 | 2.77 | 0.65 | 0.79 |
| 0.7 | 6 235 | 365 | 3.94 | 44 | 8.2 | 240 | 3.98 | 0.61 | 0.81 |
| 0.8 | 7 014 | 461 | 5.37 | 60 | 7.7 | 325 | 5.50 | 0.57 | 0.82 |
| 1.0 | 8 471 | **673** | **9.16** | **101** | 6.7 | 511 | 9.45 | 0.51 | 0.82 |

The I_batt and I-at-V_c columns include 0.5 A of avionics current (FC, GPS, receiver and four servos through a linear BEC).

Cruise at 8.2 m/s (D = 0.31 N):
- About 38 % throttle and **1.1 A** total from the battery, roughly 12 W including avionics.
- In the 7 m turns (n = 1.39) it needs about 41 % throttle.
- The average over the racetrack is 1.10 A. Of this, the motor's share is only about 6–7 W.

**Endurance.** Usable capacity is 80 % of 1000 mAh, i.e. 800 mAh. The mission is a hand launch, a climb to 20 m, 8 min of racetrack with two go-around re-climbs, glides and landing:
- Pure physics: about 177 mAh for the 8–9 min mission, and **≈43 min** of racetrack on 800 mAh.
- **Conservative beginner profile** (25 % of the time climbing at 73 % throttle, 60 % level, 15 % gliding): average 1.83 A, giving **≈26 min**.
- If the airframe is draggy (L/D ≈ 10: CD0 0.05, e 0.75): 1.35 A level, 35 min (physics) or 23 min (beginner profile).

Required capacity for 10 min with the beginner profile plus a 30 % reserve: 1.83 A × 10 min / 0.8 × 1.3 ≈ 500 mAh. **A 3S 1000 mAh pack therefore gives ≥ 2× margin.** A 3S 850 mAh pack (≈72 g, 22 min beginner profile) is acceptable if mass is tight. A 3S 450–650 mAh pack would be pushed to 15–20C at full throttle, so it is not recommended. The 1000 mAh pack doubles as the movable CG ballast (Section 6).

Sensitivity of the primary combo (all at 0.50 kg unless stated):

| Case | T0 [g] | T/W | I0 [A] | ROC max [m/s] | Endurance, physics / beginner [min] |
|---|---|---|---|---|---|
| nominal (ρ 1.09, 3.8 V/cell) | 673 | 1.35 | 9.2 | 9.0 | 43 / 26 |
| hot day ρ 1.06 | 659 | 1.32 | 9.0 | 8.8 | 43 / 26 |
| full pack 4.1 V/cell | 769 | 1.54 | 10.3 | 11.0 | 45 / 28 |
| end of pack 3.6 V/cell | 612 | 1.22 | 8.4 | 7.8 | 41 / 25 |
| CD0 0.05, e 0.75 (L/D ≈ 10) | 673 | 1.35 | 9.2 | 8.1 | 35 / 23 |
| 0.65 kg | 673 | 1.04 | 9.2 | 6.9 | 35 / 20 |
| 0.45 kg, b 1.7 (W/S 15) | 673 | 1.50 | 9.2 | 9.7 | 49 / 31 |

**ESC.**
- **Hobbywing Skywalker 20A:** 20 A continuous, 25 A burst for 10 s, 5 V/2 A linear BEC, 2–3S, 19 g.
- Programmable items: **Brake (enabled by default)**, battery type, LVC mode and threshold, soft-start, timing.
- Margin is 20 A / 10.3 A (full-pack static) = 1.9×.
- **The brake must stay ENABLED.** It stops the prop so the folding blades fold back for gliding and landing.
- If the chosen flight controller has its own regulators (Matek F405-WING/WTE, SpeedyBee F405 Wing), you can leave the ESC's BEC red wire unused. A BLHeli_S 20A ESC with brake enabled is an acceptable lighter substitute.

**Battery C-rating.** The peak is 10.3 A on 1.0 Ah, i.e. 10C. A label of "30C" (real capability about 15C) gives ≥ 1.5× margin. Use XT30 or XT60 connectors and 18–20 AWG leads.

---

## 4. Model validation (how much to trust the numbers)

**Propeller data.** Thrust and torque come from measured CT/CP at J = 0, taken from the UIUC Propeller Data Site, Vol. 1 and 3. That includes Aeronaut CAM *folding* 9×4, 9×5, 9×6, 10×5 and 10×6 (`ancf_*`), plus APC 9×6E, 8×6E and 10×4.7SF. The data are used over J = 0 to 0.75 at 5 000–7 500 rpm. Generic Chinese 9×5 folding blades are assumed to behave like the CAM 9×5. This is an **assumption**.

Comparison with manufacturer bench data at the **measured** rpm:

| Bench case | Measured | UIUC prediction at same rpm | Full model at 100 % (motor + prop) |
|---|---|---|---|
| T-Motor AT2312 KV1150 + APC 9×6, 11.12 V | 8 960 rpm, 18.0 A, 1 029 g, 0.160 N·m | T × 1/1.19, Q × 1/1.17 | 8 988 rpm, 15.4 A, 870 g |
| T-Motor AT2312 KV1150 + APC 10×4.7, 10.78 V | 7 656 rpm, 22.6 A, 1 244 g | T × 1/1.18, Q × 1/1.17 | 7 944 rpm, 20.2 A, 1 133 g |
| SunnySky X2212 KV980 + 1047, 11.1 V | 7 100 rpm, 13.2 A, 870 g | T × 1/0.96 | 7 458 rpm, 15.1 A, 998 g |

Calibration: the effective loaded KV is 0.88 × the label KV, and the ESC, wiring and losses are lumped as R = 0.07 Ω. With these, the model rpm is within 1–5 % of measured. Thrust and current fall *between* the two manufacturers' data sets: T-Motor's stand reads 15–19 % higher than UIUC, SunnySky's reads 4 % lower. **Treat model thrust and current as ±15 %.** That is why the feasibility rule requires 0.85 × T/W ≥ 0.75.

**XFLR5 and flight-test hook.** Once the real airframe polar from XFLR5 is available, run `propulsion_model.py --CD0 … --e … --mass …`. In flight, log battery current at steady cruise with the FC's current sensor and compare against the "I at V_c" column. This is one of the deliverable "prediction vs flight" comparisons.

---

## 5. Tractor (nose) vs pusher: decision

| Criterion | Tractor nose, folding prop (**chosen**) | Pusher (behind the wing or on a pylon) |
|---|---|---|
| Hand-launch safety | Prop is ahead of the nose. The launching hand grips the fuselage under the wing, about 25 cm behind the disc. | Prop sits right beside the launching hand and arm at the trailing edge. This is a well-known injury mode. |
| Belly landing on grass | With brake on, the blades fold back flat along the fuselage, so nothing sticks out to break. | A prop at the rear or below the boom strikes the ground. A folding pusher only works on a pylon above the wing. |
| CG / balance | The 68 g motor, prop and spinner at the nose offset the long tail moment, so **no ballast is needed** (Section 6). | Motor mass sits aft of the CG. It needs 50–100 g of nose ballast or a longer nose. |
| Efficiency | Clean inflow. The slipstream over the wing root and tail improves control at launch. | Ingests the wake from the fuselage and wing. Propeller efficiency is about 5–10 % lower and the prop is noisier. |
| Glide | Folded blades add little drag. | A fixed pusher prop windmills and adds drag. |
| Build | One printed nose mount, with 2° right thrust and 1–2° down thrust built in. | Needs a pylon or boom mount. Thrust-line offset changes pitch trim with throttle. |

---

## 6. Motor position and CG (for the layout and stability team)

`x_cg = Σ m_i x_i / Σ m_i`. A nose-mounted motor is the most effective balance mass in the aircraft:
- Motor, prop and spinner are about 68 g at about 0.28 m ahead of a typical CG. That is 0.019 kg·m nose-down moment.
- It roughly cancels a tail group of horizontal stab, vertical stab and linkages (≈30 g at ≈0.65 m aft of the CG, 0.020 kg·m).
- The **battery (85 g) then acts as the trim mass.** Sliding it Δx moves the CG by `Δx_cg = m_batt/m_total · Δx ≈ 0.17 Δx`. So ±30 mm of battery travel gives ±5 mm of CG, about ±3 % MAC for a MAC of about 0.15–0.17 m.
- Leave a 40–50 mm battery bay with velcro under the wing leading edge, so the XFLR5 CG target can be met exactly.
- With a pusher (motor 0.15 m aft of the CG) the deficit is about 0.029 kg·m. That would need roughly 100+ g of nose ballast at a 0.25 m arm.

**Build notes for the motor mount (3D printed on the P1S):**
- Print in **PETG or ASA, not PLA**. The motor case reaches 50–70 °C after full-throttle climbs, and PLA softens at about 55 °C.
- Use 4 × M3 screws on the 16/19 mm bolt pattern.
- Build in 2° right thrust and 1–2° down thrust.
- Leave ≥ 5 mm of cooling gap behind the bell and add an air inlet in the nose.
- The spinner diameter (30–35 mm) sets the nose cross-section.

**ArduPilot / INAV.** Use TAKEOFF mode or FBWA with throttle armed but held until the throw is detected. In ArduPilot: `TKOFF_THR_MINACC` ≈ 15 m/s², `TKOFF_THR_DELAY` ≈ 2 (0.2 s), `TKOFF_THR_MAX` ≈ 75 %, so the prop spins up only after the aircraft leaves the hand. Set battery failsafe at 3.5 V/cell under load.

---

## 7. Sources

- Lecture notes L13/L14 (IIIT-H EC4.402): Pa > D·V_a, endurance t_e = E(L/D)/(mgV).
- UIUC Propeller Data Site, Vol. 1 (APC E/SF), https://m-selig.ae.illinois.edu/props/volume-1/propDB-volume-1.html, and Vol. 3 (Aeronaut CAM folding `ancf_*`), https://m-selig.ae.illinois.edu/props/volume-3/propDB-volume-3.html. Raw files are copied into `prop_data/`.
- T-Motor AT2312 long-shaft bench tables (KV1150/1400, APC 8×6, 9×6, 10×4.7, with torque), https://store.tmotor.com/goods.php?id=788 and https://uav-en.tmotor.com/2019/Motors_0226/211.html
- SunnySky X2212 KV980: 0.6 A/10 V, 92 mΩ, 26 A for 30 s, 57 g, https://www.readymaderc.com/products/details/sunny-sky-x-series-v3-x2212-980kv. Bench point 1047 at 11.1 V, 13.2 A, 870 g, 7 100 rpm from https://flymod.net/en/item/sunnysky_x2212_kv980_brushless_motor (retailer copy of the maker's table). India: Robu "Sunny Sky X fixed wing Series III X2212-III 980KV"; Electropi lists ₹2 239 + GST, https://www.electropi.in/sunny-sky-x-fixed-wing-series-iii-motor-x2212-iii-980kv
- A2212/13T 1000 KV specs (0.5 A @10 V, 90 mΩ, 12–13 A/60 s, 47–53 g): https://components101.com/motors/2212-brushless-motor, https://sharvielectronics.com/product/a2212-1000kv-bldc-brushless-motor/, https://robojax.com/product_view.php?id=1. 3S/1045 ≈ 800 g: https://quartzcomponents.com/products/a2212-1000kv-bldc-drone-motor. India prices ₹319–399: https://robobazar.in/product/a2212-1000kv-bldc-brushless-motor, https://atomlabindia.com/?p=8611, https://robodo.in/products/a2212-13t-brushless-motor-1000-kv-bldc-airplane-outrunner-brushless-motor
- A2212/10T 1400 KV datasheet (0.7 A, 65 mΩ, 16 A/60 s, 47 g, 3.17 mm shaft): https://cdn.shopify.com/s/files/1/0798/6901/8399/files/a2212-brushless-motor.pdf
- EMAX CF2822 1200 KV (39 g, 0.9 A @10 V, 150 mΩ, 16 A/60 s): https://www.amainhobbies.com/emax-cf2822-1200kv-brushless-motor-emx-mt-0487/p694265, https://mikroelectron.com/Product/EMAX-CF2822-1200KV-Brushless-Motor. India: https://kitsguru.com/products/emax-cf2822-1200kv-brushless-motor-for-rc-airplane-multicopter (stock and price not verified).
- Hobbywing Skywalker 20A manual (20/25 A, linear 5 V/2 A BEC, 19 g, brake programmable): https://www.manualsdir.com/manuals/737937/hobbywing-skywalker-20a-skywalker-40a-skywalker-40a-ubec.html. India price ₹884–985: https://www.moglix.com/hobbywing-skywalker-esc-20a-speed-controller/mp/msn457mlomln9j, https://indianrobostore.com/product/hobbywing-skywalker-20a-esc-speed-controller-for-drone, https://zbotic.in/product/hobbywing-skywalker-20a-ubec/
- Orange 3S 1000 mAh 30C: 85 g, 72×34×19 mm, https://robocraze.com/products/11-1-v-1000mah-orange-lithium-polymer-battery-pack. ₹966 on Robu, https://robu.in/?p=23749. Orange 3S 1500 mAh 30C: 115 g, ₹1 299, https://st.robu.in/product/orange-11-1v-1500mah-3s-30c-lipo-battery-pack-xt60-connector. 3S 850: https://robu.in/?p=54622 (mass ≈72 g **assumed**).
- Folding props: there is no reliable Indian stock of glider folding props with a 3.17 mm collet. Examples to import or match: 10×6 folding with spinner and 3.17/4 mm collet, https://www.flyingtech.co.uk/product/volantex-asw-28-v2-v3-phoenix-v2-1060-10x6-folding-propeller/; 9×5 folding with hub and spinner (4 mm), https://vxb.com/products/rc-folding-propeller-9x5-with-hub-and-spinner-cap; collet adapter 3.17–4 mm (India), https://zbotic.in/product/a2212-motor-prop-adapter-collet-type-propeller-adapter/. Multirotor "folding" props (e.g. Evelta 9×5 glass-nylon) fold sideways, not backward, so **do not use them**.

**Assumptions to revisit:**
- Whole-aircraft CD0 0.025 and e 0.85 are the task defaults. Sensitivity was run to CD0 0.05 and e 0.75.
- CLmax 1.1.
- 0.5 A avionics current.
- Battery IR 15 mΩ/cell.
- Generic folding 9×5 performs like the Aeronaut CAM 9×5.
- Mid-discharge 3.8 V/cell.
- Prices are list prices seen in Nov 2026 and change often.
