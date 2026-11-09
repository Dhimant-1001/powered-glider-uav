# 05 Mission and field constraints: constraint diagram and design point

Script: `mission_constraints.py`. Run it with `/opt/anaconda3/bin/python3 mission_constraints.py`. All numbers below come from `mission_results.json`.
Figures: `fig1_constraint_diagram.png`, `fig2_turn_geometry.png`, `fig3_field_pattern.png`, `fig4_gust_speed_window.png`.

The analysis uses a point-mass model with a parabolic polar. Assumed preliminary aero: CL_max(3D, clean) = 1.0 (range 0.90–1.15), CD0 = 0.030, AR = 11, e = 0.85, CL_α ≈ 5.0 /rad, MAC ≈ 0.15 m. Air: ρ = 1.09 kg/m³ (Hyderabad, 540 m, 30 °C), with a hot-day check at ρ = 1.06. η_total (prop × motor × ESC) = 0.45 is assumed. Once XFLR5 results exist, its CL_max, CD0, e and polar replace these values. Re-run the script with them.

---

## 1. The binding constraint is the 20 m width

The aircraft must stay above the field footprint, so a 180° reversal has to fit within the 20 m width. Allowing 3 m per side for pilot error gives a target turn width of **≤ 14 m**. The absolute maximum is **≤ 16 m** (2 m per side).

* Level turn: R = V²/(g tan φ), n = 1/cos φ, V_s,turn = V_s·√n.
* Safety rule in the turn: V_turn ≥ 1.15·V_s·√n. Equivalently, CL_turn ≤ 0.76·CL_max.
* Combining both: **W/S ≤ ½ ρ CL_max V_turn² cos φ / 1.15²**. Here V_turn is the largest speed whose 180° turn fits in 14 m. That turn includes a 60°/s roll-in and roll-out, simulated numerically in `turn_track()`.

| Bank φ | n | Max turn speed for 14 m | Max turn speed for 16 m | Max W/S (CL_max 1.0) | Hot day |
|---|---|---|---|---|---|
| 30° | 1.15 | 6.3 m/s | 6.7 m/s | 14.0 N/m² | 13.6 |
| 35° | 1.22 | 6.9 m/s | 7.4 m/s | 16.0 | 15.6 |
| **40°** | **1.31** | **7.5 m/s** | 8.0 m/s | **17.8** | **17.3** |
| 45° | 1.41 | 8.2 m/s | 8.7 m/s | 19.4 | 18.8 |
| 50° | 1.56 | 8.8 m/s | 9.5 m/s | 20.6 | 20.1 |

* A 30° "gentle" bank does not fit at any sensible speed: it needs ≤ 6.3 m/s, which is barely above stall. **Turns must be flown at 40–45° bank.** A novice can hold 40° comfortably with a flight controller in a stabilised mode (ArduPilot FBWA, INAV ANGLE). Set the bank limit to 45° (`ROLL_LIMIT_DEG=45` in ArduPilot, or the INAV max bank angle setting).
* Roll rate barely changes the turn width. At 40° bank and 7.5 m/s the width is 14.3 m at 40°/s, 14.0 m at 60°/s and 13.8 m at 90°/s. Roll rate matters for reversals and corrections (Section 7).
* CL_max matters a lot. With CL_max = 1.15 (flaps or a better airfoil) the W/S limit at 40° rises to about 20.5 N/m². With CL_max = 0.9 it falls to about 16.

## 2. Hand launch

The motor is at full throttle at release, so the throw only has to clear stall with margin. The rule used is V_throw ≥ 1.15·V_s, giving W/S ≤ ½ ρ CL_max (V_throw/1.15)².

**No measured data on how fast an untrained person can throw a model aircraft was found, so the throw speed is an assumption.** A few running steps plus an over-arm javelin-style push gives an estimated **6–8 m/s; 7 m/s is the design value and 6 m/s the pessimistic one**. For reference, PX4's throw-launch detection threshold defaults to 5 m/s (https://docs.px4.io/main/en/flight_modes_mc/throw_launch). Launching into the wind adds the wind speed to the airspeed.

* Launch at 7 m/s: W/S ≤ 20.2 N/m². Launch at 6 m/s: W/S ≤ 14.8 N/m².
* At the design point (V_s = 5.4 m/s), V_throw/V_s is 1.29 at 7 m/s and 1.11 at 6 m/s. The 6 m/s case still works because thrust accelerates the aircraft after release.
* **Thrust needed after release:** reaching 1.3·V_s within 1.5 s from a 6 m/s throw while climbing at 8° needs a dynamic T/W of at least 0.29 at about 7 m/s. **Recommend static T/W ≥ 0.6** (normal hand-launch practice, because thrust falls off with speed). This number goes to the propulsion agent.

## 3. Climb-out and pattern altitude

* Target: ROC ≥ 2 m/s at 8 m/s, which gives a climb angle of 14.5°. Reaching 20 m takes 10 s and covers 78 m of ground in calm air, or 48 m into a 3 m/s headwind. The aircraft is therefore at roughly 15–20 m by the first turn at the far end. Turns during the climb are fine.
* Pattern altitude: **15–25 m**. At this height a stall recovery has room (about 5–10 m height loss for this class), and the aircraft stays easy to see. A 1.6 m span seen from 100 m subtends about 0.9°.

## 4. Altitude hold: racetrack along the 100 m length

* Usable box: 90 m × 14 m (5 m end margins, 3 m side margins).
* Design turn: 7.5 m/s at 40° bank, R = 6.8 m. With roll-in and roll-out the turn is 14.0 m wide and takes **3.6 s**. Straight legs are **70.9 m** each, flown at 8 m/s in **8.9 s**.
* **Lap time is 24.9 s.** A 60 s altitude hold is 2.4 laps and 120 s is 4.8 laps. The aircraft spends 29% of its time banked.
* Power for level cruise or a sustained 40° turn is only about 14 W/kg electrical. Climb sets the power requirement (Section 9).

## 5. Descent and landing

The high L/D that the spec asks for creates a landing problem in a 100 m field.

| Final glide from | Clean L/D ≈ 12 | Parabolic L/D_max 15.6 | Spoilerons L/D ≈ 7 | Crow / spoiler L/D ≈ 5 |
|---|---|---|---|---|
| 10 m | **120 m (does not fit)** | 157 m | 70 m | 50 m |
| 5 m | 60 m | 78 m | **35 m** | 25 m |

* Approach at 1.3·V_s = 7.0 m/s. Touchdown at 1.1·V_s = 6.0 m/s.
* The flare float in ground effect adds about 11 m (L/D ≈ 15 assumed).
* Belly skid on mown grass, with μ ≈ 0.4 assumed: ground roll about 4.5 m in calm air, about 1 m into a 3 m/s headwind.
* **Total landing distance from 5 m on final: about 50 m with spoilerons, about 75 m clean.**
* **Procedure:** lose height in a 30° circling descent over the downwind half of the field (sink ≈ 0.9 m/s, motor off). Turn onto final at about 5 m and ≥ 20 m from the far end. Use the spoilerons to adjust the glide path.
* **Requirement:** two separate aileron servos so the flight controller can mix spoilerons (both ailerons up 15–25°). This gives an approach L/D ≤ 8. Set the ESC brake on so the propeller stops.

## 6. Wind (< 3 m/s)

* **Wind along the 100 m axis (preferred):** a 180° turn drifts 3 × 3.6 ≈ 11 m along the field. This costs field length, not width. Ground speed is 4.5 m/s upwind and 11 m/s downwind at 7.5–8 m/s airspeed. The cruise speed of 8 m/s is about 2.7 times the wind speed, which is enough to penetrate it.
* **Wind across the 20 m width is the critical case.** At 40° bank and 7.5 m/s, a downwind-drifting turn grows to these widths:

| Crosswind | 40° bank | 45° bank | 50° bank |
|---|---|---|---|
| 0 m/s | 14.0 m | 11.9 m | 10.3 m |
| 1.0 m/s | 17.6 | 15.2 | 13.3 |
| 1.5 m/s | 19.4 | 16.8 | 14.8 |
| 3.0 m/s | 24.7 | 21.7 | 19.2 |

* **Operating rules:**
  * Orient the racetrack, launch and landing along the field's long axis, into wind.
  * Fly only when the crosswind component is ≤ 1 m/s. Under that condition, bank 45° on the turn that drifts outward.
  * Otherwise fly in the early morning, when wind is usually calmest.

## 7. Controllability and control surfaces

**Recommendation: ailerons + elevator + rudder + throttle, with each aileron on its own servo (5 channels, 4 servos).**

* **Why ailerons rather than rudder with dihedral:**
  * A 40° turn in a 14 m width needs a quick, positive roll-in.
  * The aircraft must reverse bank on demand for the figure-of-eight and the left and right roll requirement.
  * Separate aileron servos are needed for spoilerons and flaperons, which the landing requires (Section 5).
  * Rudder-dihedral roll on a high-AR wing is slow and sideslip-driven.
* **Why keep a rudder:** a high-AR wing at CL ≈ 0.7 in a turn has strong adverse yaw. The flight controller mixes rudder with aileron to coordinate the turn, and the rudder also helps on the landing approach.
* **Roll targets:** pb/2V ≥ 0.08, which is about 55°/s at 8 m/s with a 1.6 m span. Time from 0 to 40° bank ≤ 0.8 s. Time for a 40° to −40° reversal ≤ 1.6 s.
* **Strip-theory check:** ailerons at 55–95% of the semispan with cf/c = 0.25 give Cl_δa ≈ 0.34 /rad and Cl_p ≈ −0.84. With a 0.7 empirical correction, the roll performance by span is:

| Aileron deflection | pb/2V | Roll rate, b = 1.4 m | Roll rate, b = 1.6 m | Roll rate, b = 1.8 m |
|---|---|---|---|---|
| 15° | 0.074 | 49°/s | 43°/s | 38°/s |
| 20° | 0.099 | 65°/s | 57°/s | 50°/s |

**Design for ±20–25° aileron throw.** A longer span lowers the roll rate, which is another reason to keep the span at about 1.8 m or less.

## 8. Gust sensitivity versus wing loading

Pratt–Walker formula at 8 m/s and W/S = 16 gives K_g = 0.38. A 1 m/s gust gives Δn ≈ 0.52 and a 1.5 m/s gust gives Δn ≈ 0.78. In both cases the gust angle (U/V) is very large at these speeds.

The gust-stall margin requires CL_cruise + ΔCL_gust ≤ CL_max:

* 1.5 m/s gust: needs V ≥ 7.0 m/s, which coincides with 1.3·V_s.
* 2 m/s gust: at 8 m/s, the peak CL is 0.93, still below CL_max = 1.0.

The flyable speed window at W/S = 16 is therefore **7.0 m/s (gust and stall margin) to 7.5 m/s (field turn limit) in turns**, and 8–9 m/s on the straights. The window closes at about W/S = 18 N/m² (`fig4_gust_speed_window.png`).

Lower W/S makes the aircraft bounce around more in gusts (`fig4_gust_speed_window.png`, panel a). This is a reason to stay near 16 and not go much lighter. Fly in calm conditions below 3 m/s.

## 9. Constraint diagram (`fig1_constraint_diagram.png`)

**Upper bounds on W/S (CL_max = 1.0, ρ = 1.09):**

* Turn fits 14 m at 40°: **17.8** (17.3 on a hot day)
* Turn at 45°: 19.4
* Hand launch at 7 m/s: 20.2
* Hand launch at 6 m/s: 14.8 (soft bound, because thrust helps)

**Lower bounds on W/S:**

* Efficient cruise at the 8 m/s wind-penetration speed (L/D ≥ 75% of L/D_max): **14.8**
* Gust bounce (soft): about 14
* Buildability: about 13.7. 3D-printed LW-PLA models of 1.2–1.4 m span reach 15–24 g/dm²:
  * 3DLabPrint Bungee Prop: 1180 mm span, 21.2 dm², 320 g, 15.1 g/dm² (https://3dlabprint.com/wp-content/uploads/2023/11/Bungee_Prop_userguide23.pdf)
  * Eclipson Model V: 1370 mm span, 16 dm², 380 g, 24 g/dm² (https://learn.colorfabb.com/tag/eclipson)
  * For comparison, hobby park-flyers and gliders typically sit at 12–35 g/dm² (https://www.stefanv.com/rcstuff/qf200211.html).

**Feasible window: W/S ≈ 14.8–17.8 N/m² (15–18 g/dm²).**

**Power:** the 2 m/s climb needs 58 W/kg electrical and a 3 m/s climb needs 80 W/kg. Cruise needs 14 W/kg. With a 1.5× margin, **install ≥ 90 W/kg; 100–120 W/kg is recommended.** For 450 g that is about 45–55 W electrical, for example a 2S or 3S battery with a 22xx-class motor and a folding or fixed 7–9″ propeller. The propulsion agent sizes this.

**Robustness check:** the design point W/S = 16 still satisfies the turn rule exactly (V/V_s,turn = 1.15) if XFLR5 shows CL_max is only 0.9. With CL_max = 1.0 the margin is 1.21.

---

## 10. Quantified requirements table

| # | Requirement | Min | **Target** | Max | Driver |
|---|---|---|---|---|---|
| R1 | Wing loading W/S [N/m²] | 14.8 | **16.0** | 17.8 (17.3 hot) | turn width / efficiency, build |
| R1 | Wing loading W/S [g/dm²] | 15.1 | **16.3** | 18.1 | same |
| R2 | Stall speed V_s (clean, 1 g) [m/s] | – | **5.4** (5.5 hot, 5.7 at CL_max 0.9) | 5.7 | launch, turn |
| R3 | 3D CL_max (clean) | 0.90 | **≥ 1.0** | – | XFLR5 must confirm |
| R4 | Cruise speed (straight legs) [m/s] | 7.0 (1.3 V_s) | **8.0** | 9.0 | wind penetration, gust margin |
| R5 | Cruise CL | 0.40 | **0.46** (V = 8) to 0.52 (V = 7.5) | 0.6 | efficiency |
| R6 | Turn speed [m/s] | 7.0 | **7.5** | 7.5 (14 m) / 8.0 (16 m) | field width |
| R7 | Bank angle in 180° turns | 35° | **40°** | 45° (FC limit) | field width |
| R8 | Load factor in turn | – | **1.31** | 1.41 | structure: design limit n ≥ 3 for launch/gust |
| R9 | Turn width incl. roll-in (calm) [m] | – | **14.0** | 16 | 20 m field |
| R10 | V_turn / V_s,turn | 1.15 | **1.21** | – | stall margin in turn |
| R11 | Hand-launch speed [m/s] | 6 | **7** | – | untrained thrower (assumed) |
| R12 | Static thrust / weight | 0.5 | **0.6–0.8** | – | launch acceleration, climb |
| R13 | Electrical power / mass [W/kg] | 60 | **100–120** | – | 2–3 m/s climb × 1.5 |
| R14 | Rate of climb [m/s] | 1.5 | **2.0–3.0** | – | reach 20 m within field length |
| R15 | Pattern altitude [m] | 15 | **20** | 30 | recovery room, visibility |
| R16 | Lap time (90 m racetrack) [s] | – | **≈ 25** | – | hold "set duration" (60 s ≈ 2.4 laps) |
| R17 | Approach L/D with spoilerons | 5 | **≤ 7–8** | 8 | land within ≈ 50 m from 5 m |
| R18 | Clean L/D (cruise) | 10 | **≥ 12** | – | spec: efficient flight |
| R19 | Touchdown speed [m/s] | – | **6.0** | 6.5 | belly landing on grass, about 5 m roll |
| R20 | Roll helix pb/2V | 0.07 | **0.08–0.10** | – | 0→40° in ≤ 0.8 s |
| R21 | Aileron throw | ±15° | **±20–25°** | – | R20; up-only 20° for spoileron |
| R22 | Control set | – | **2 × aileron servo + elevator + rudder + ESC** | – | turns, coordination, spoilerons |
| R23 | Wind limit | – | **≤ 3 m/s along field, ≤ 1 m/s across** | – | crosswind turn drift |
| R24 | Chord Reynolds number | 4.8e4 (stall) | **7e4 (cruise)** | 9e4 | airfoil must be chosen for Re 50–90k |
| R25 | Wing span | 1.4 m | **1.6–1.8 m** | 1.8 m | mass budget, roll rate, P1S segments |

**Mass budget implied by W/S = 16 N/m²:** S = m·g/16.

| AR | b = 1.4 m | b = 1.6 m | b = 1.8 m |
|---|---|---|---|
| 9 | S = 0.218 m², 355 g | S = 0.284 m², 463 g | S = 0.360 m², 587 g |
| **11** | S = 0.178 m², 290 g | **S = 0.233 m², 379 g** | **S = 0.295 m², 480 g** |
| 13 | S = 0.151 m², 245 g | S = 0.197 m², 321 g | S = 0.249 m², 406 g |

**Rule of thumb: every 100 g of all-up mass needs about 6.1 dm² of wing.** A realistic 3D-printed build with electronics weighs 400–500 g, so AR ≈ 10–12 calls for b ≈ 1.6–1.8 m and a chord of about 0.15–0.16 m.

## 11. Recommended design point

* **W/S = 16 N/m² (16.3 g/dm²).** Examples: 450 g on 0.276 m², or 1.74 m × 0.158 m at AR 11.
* V_s = 5.4 m/s.
* Cruise: 8 m/s, CL ≈ 0.46.
* Turns: 7.5 m/s at 40° bank, n = 1.31, R = 6.8 m, CL_turn = 0.68.
* Hand launch at 7 m/s with throttle on.
* Climb at 2–3 m/s to 20 m.
* Racetrack lap ≈ 25 s.
* Land from about 5 m on final using spoilerons, at about 6 m/s touchdown.

**Min-sink and best-L/D sanity check.** With the parabolic polar, CL* (best L/D) = √(CD0/K) = 0.94, so V_(L/D)max ≈ 5.6 m/s, which is roughly V_s. CL for minimum sink (1.63) is above CL_max. Two consequences follow:

* For this low-W/S, high-AR class, the theoretical best-L/D and min-sink points lie at or beyond the stall. The safe operating point at CL ≈ 0.46–0.55 gives L/D ≈ 12 (about 80% of L/D_max ≈ 15.6). This is the usual trade for a field-limited glider: fly at 1.3–1.5·V_s rather than at the polar optimum.
* XFLR5 should be used to choose an airfoil whose low-drag bucket covers **CL 0.4–0.75 at Re 50–90k**. That moves the real best-L/D point down onto the operating CL. Report the XFLR5 "low-speed operating point" as about 7–7.5 m/s at CL ≈ 0.5–0.55. That is 1.3·V_s, the slowest speed that still leaves gust and stall margin.

## 12. Assumptions that need checking

* The 6–8 m/s throw speed is an assumption. Measure it by filming a throw of a 450 g dummy airframe.
* The μ ≈ 0.4 belly-skid friction on grass is an assumption.
* η_total = 0.45 is an assumption.
* CD0 = 0.03, e = 0.85 and CL_max = 1.0 are placeholders until XFLR5 results exist.
* The aileron effectiveness carries a 0.7 strip-theory correction.
* Pratt K_g is used for gust loads.
* The wind-drift turns assume a constant bank. A real pilot will tighten the bank on the downwind half of the turn.
