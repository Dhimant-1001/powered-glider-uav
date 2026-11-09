# Course Theory Summary — EC4.402 Intro to UAV Design (Lectures 11–14, fixed wing)

Source: H. Kandath, EC4.402 slides, "Lecture 11.pdf" (Sep 29), "Lecture 12.pdf" (Oct 01), "Lecture 13.pdf" (Oct 06), "Lecture 14.pdf" (Oct 09).
Reference cited by the instructor: R.W. Beard & T.W. McLain, *Small Unmanned Aircraft: Theory and Practice*, Princeton UP, 2012.
Next lecture after L14 = "Using XFLR" (L14 p.28) — XFLR5 is the instructor's prescribed tool for computing forces/moments (L12 p.32: "Software: CFD, e.g. XFLR; Experiments: wind tunnel").

Notation: reference `L13 p.12 eq.(9)` = Lecture 13, slide page 12, equation number as printed. L12 and L13 repeat much of L11; L14 repeats much of L13 and adds tail sizing.

Items marked **[SUPPLEMENT — not in slides]** are standard textbook relations added because the project needs them (turn, climb, glide, static margin, battery). They are consistent with the instructor's notation but were *not* taught on L11–L14 slides; justify them in the report with a textbook citation (e.g. Beard & McLain; Raymer, *Aircraft Design: A Conceptual Approach*; Anderson, *Introduction to Flight*).

---

## 1. Requirements, flight phases, design cycle (L11 p.4–11)

**Size classes (L11 p.4):** UAV; Micro Air Vehicle (MAV) wingspan ≤ 15 cm; Nano Air Vehicle wingspan ≤ 7.5 cm.

**Requirement specifications (L11 p.5):**
- Operating velocity
- Range — total distance traversed without refuelling/recharging
- Endurance — total time of flight without refuelling/recharging
- Payload — weight carried in addition to what is essential for flying
- Wind — head, tail, cross
- Altitude
- Safety
- Maneuverability

**Phases of flight (L11 p.6):** Take-off, Climb, Cruise, Loiter, Descent, Landing, (Aggressive maneuvers — fighter UAV).

**Take-off methods (L11 p.7):** Runway, Bungee launch, **Hand launch** (our case).

**Subsystems (L11 p.8–9):**
- Hardware: configuration, structures, propulsion, sensors & actuators, computer, payload, GCS, radio & data telemetry.
- Software: navigation (sensor interface), guidance/motion planning, control system (actuator interface), health monitoring, payload data processing, GCS (data processing/display, mission planner).

**Design cycle (L11 p.10–11):** Conceptual → Preliminary → Detailed.
Conceptual design = CONOPS (concept of operation) → requirement specifications → market survey → sizing and layout → component identification → analysis and optimization (theory and simulations).

**Parts of a UAV (figure, L11/L12/L13 p.3–4):** fuselage, wing, ailerons, horizontal tail + elevator, vertical tail + rudder, propulsion.

---

## 2. Variables, frames and equations of motion (L11 p.13–19; L12 p.5–9)

Inertial frame: position x, y, z; velocity v_x, v_y, v_z; acceleration a_x, a_y, a_z. **Altitude h = −z** (NED).
Body frame: velocity u, v, w; accelerations u̇, v̇, ẇ; angular rates p, q, r. Euler angles ψ (yaw), θ (pitch), φ (roll).

**Force equation (Coriolis), L11 eq.(1):**
$$ (\dot{\bar V}_B)_I = (\dot{\bar V}_B)_B + \bar\Omega_B \times \bar V_B = \frac{\bar F}{m}, \quad \bar V_B=[u,v,w]^T,\ \bar\Omega_B=[p,q,r]^T $$
$$ \bar F = [F_{XB},F_{YB},F_{ZB}]^T = \bar F_g + \bar F_a + \bar F_t $$ (gravity + aerodynamic + propeller thrust)

**Moment equation, L11 eq.(2)–(3):**
$$ \bar J(\dot{\bar\Omega}_B)_I = \bar J(\dot{\bar\Omega}_B)_B + \bar\Omega_B\times \bar J\bar\Omega_B = \bar M,\quad \bar J=\begin{bmatrix}J_{xx}&J_{xy}&J_{xz}\\J_{yx}&J_{yy}&J_{yz}\\J_{zx}&J_{zy}&J_{zz}\end{bmatrix},\ \bar M=[M_{XB},M_{YB},M_{ZB}]^T $$

**Euler-rate kinematics, L11 eq.(4)–(5):** $\dot{\bar E} = T_1(\bar E)\bar\Omega_B$, $\bar E=[\phi,\theta,\psi]^T$
$$ T_1=\begin{bmatrix}1&\sin\phi\tan\theta&\cos\phi\tan\theta\\0&\cos\phi&-\sin\phi\\0&\sin\phi\sec\theta&\cos\phi\sec\theta\end{bmatrix} $$
(Note: the text extraction shows "cosφ secθ" in the first row; the standard/correct entry is cosφ tanθ.)

**Position kinematics, L11 eq.(6)–(7):** $\dot{\bar P}=T_2(\bar E)\bar V_B$
$$ T_2=\begin{bmatrix}c\theta c\psi & s\phi s\theta c\psi - c\phi s\psi & c\phi s\theta c\psi + s\phi s\psi\\ c\theta s\psi & s\phi s\theta s\psi + c\phi c\psi & c\phi s\theta s\psi - s\phi c\psi\\ -s\theta & s\phi c\theta & c\phi c\theta\end{bmatrix} $$

**Longitudinal plane (L11 p.24):** $\alpha=\tan^{-1}(w/u)$, $\theta=\alpha+\gamma$ (γ = flight-path angle; γ = 0 ⇒ constant altitude).
**Lateral plane (L12 p.30, L14 p.23):** sideslip $\beta=\sin^{-1}(v/V_a)$.

**Control surfaces (L12 p.7):** ailerons → rolling moment M_XB; elevator → pitching moment M_YB; rudder → yawing moment M_ZB.

---

## 3. Wing geometry (L11 p.19–21; L12 p.10–12)

- Rectangular wing: wing area $S_w = b\,c$; **aspect ratio** $AR = b^2/S_w = b/c$ (L12 p.10).
- **Mean aerodynamic chord** (L11 eq.(8)): $\bar c = \dfrac{2}{S}\displaystyle\int_0^{b/2} c(y)^2\,dy$
- **Taper ratio**: $\lambda = c_{tip}/c_{root}$, λ ≤ 1 (L11 p.20; L13/L14 eq.(5)).
- Airfoil terms (L11 p.21): leading edge, trailing edge, chord line, mean camber line, camber, thickness, chord c.
- NACA 4-digit example (L12 p.12): NACA 2409 = 2 % camber at 40 % chord, 9 % thickness.
- Clark Y (L12 p.17, L13 p.7): max thickness 11.7 % at 28 % chord, max camber 3.4 % at 42 % chord.
- Forces on an airfoil (L11 p.22): L ⟂ relative wind, D ∥ relative wind, R resultant; moment M_{c/4} about quarter chord. Lift arises from pressure difference: upper surface low pressure, lower surface high (L12 p.13).

---

## 4. Lift, drag, pitching moment (L11 eq.(9)–(14); L13 eq.(1)–(6))

$$ F_{Lw} = C_{Lw}\tfrac12\rho V_a^2 S_w, \qquad C_{Lw}=C_{L\alpha}\alpha + C_{L0} $$
$$ F_{Dw} = C_{Dw}\tfrac12\rho V_a^2 S_w, \qquad C_{Dw}=C_{D2}\alpha^2 + C_{D1}\alpha + C_{D0} $$
$$ M_w = C_{Mw}\tfrac12\rho V_a^2 S_w \bar c, \qquad C_{Mw}=C_{M\alpha}\alpha + C_{M0} $$

Naming convention (L12 p.28): wing (3-D) uses capitals C_Lw, C_Dw, C_Mw, C_Lα, C_Mα; airfoil (2-D) uses lower case C_l, C_d, C_m, C_lα, C_mα.

**Reference curves (L11 p.26, L13 p.7):** Clark Y at AR = 6: C_L ≈ 0 at α ≈ −5°, linear to C_Lmax ≈ 1.55 at α ≈ 18–19°, then stall; C_D ≈ 0.01 minimum near −5…0°, ≈ 0.04 at 10°, ≈ 0.13 at 20°.
Clark Y 2-D polar (L14 p.7): C_l max ≈ 1.35 at ~13°, C_m ≈ −0.08 to −0.09 near 0°, **C_l/C_d max ≈ 72 near α ≈ 5°** (low-Re XFLR-style plot).

**Effect of camber (L12 p.29, L14 p.8):** positive camber shifts the lift curve up/left (lift at α = 0, higher C_lmax); symmetric has C_l = 0 at α = 0; negative camber shifts it down (used for tail, see §8).

**Body-axis aerodynamic forces (L12 eq.(22)–(25)):**
$$ X_a = \tfrac12\rho V_a^2 S\,(C_L\sin\alpha - C_D\cos\alpha),\quad Y_a=\tfrac12\rho V_a^2 S\,C_y,\quad Z_a=\tfrac12\rho V_a^2 S\,(-C_L\cos\alpha - C_D\sin\alpha) $$
C_L, C_D = f(α, q, δ_e); C_y = f(β, p, r, δ_a, δ_r).

**Moments (L12 eq.(26)–(28)):**
$M_{XB}=\tfrac12\rho V_a^2 S b\,C_{MX}$, $M_{YB}=\tfrac12\rho V_a^2 S\bar c\,C_{MY}$, $M_{ZB}=\tfrac12\rho V_a^2 S b\,C_{MZ}$; C_MX, C_MZ = f(β,p,r,δ_a,δ_r), C_MY = f(α,q,δ_e).

---

## 5. Steady level flight, airspeed selection, L/D (L12 eq.(11)–(16); L13 eq.(7)–(12))

**Simplified longitudinal equilibrium:**
$$ L + T\sin\alpha = mg,\qquad T\cos\alpha = D;\qquad \text{small }\alpha:\ L=mg,\ T=D $$

**Required C_L at airspeed:** $C_{Lw} = \dfrac{mg}{\tfrac12\rho V_a^2 S_w}$, then α = (C_Lw − C_L0)/C_Lα.

**Stall speed (L12 p.19):** lowest airspeed at C_Lw = C_Lmax:
$$ V_{stall}=\sqrt{\frac{2mg}{\rho S_w C_{L\max}}} $$

**Airspeed selection (L12 eq.(13)–(14)):** $\dfrac{L}{D}=\dfrac{mg}{T} \Rightarrow T=\dfrac{mg}{L/D}$. **Thrust required is minimum when L/D is maximum.**
$$ \frac{L}{D}=\frac{C_{Lw}\tfrac12\rho V_a^2S_w}{C_{Dw}\tfrac12\rho V_a^2S_w}=\frac{C_{Lw}}{C_{Dw}} $$

**Upper limit on airspeed (L13 eq.(11)–(12)):** thrust available T_a > T (= D); power available P_a > T·V_a = D·V_a. Maximum speed is where P_a curve meets P_required curve. Example plot (L13 p.13): 6-inch prop on 0.15 m MAV — P_available ≈ 6.7 W at 10–12 m/s falling to 0 near 24 m/s; P_required ≈ 1.7 W min near 11 m/s, rising; intersection ≈ 20 m/s.

### Worked example in slides (L12 p.19–22, L13 p.9–12)
m = 4 kg, rectangular wing b = 3 m, c = 0.5 m (S = 1.5 m², AR = 6), C_Lw = 4α + 0.35 (α in rad), ρ = 1.1 kg/m³, C_Dw = 8.5α² − 0.04α + 0.11, C_Lmax = 1.6.

| V_a (m/s) | α (deg) | Drag (N) | L/D |
|---|---|---|---|
| 5 | 22.2 | 28.3 | — |
| 6 | 13.9 | 17.8 | 2.20 |
| 7 | 8.9 | 12.5 | 3.14 |
| 8 | 5.6 | 9.9 | 3.96 |
| 9 | 3.4 | 9.2 | **4.27 (max)** |
| 10 | 1.86 | 9.7 | 4.05 |

At V_a = 10 m/s, C_Lw = 4×9.81/(0.5·1.1·10²·1.5) = 0.48 ⇒ α = 0.0325 rad (1.86°). V_stall = 5.5 m/s at C_Lmax = 1.6. (Verified numerically.) The best operating point is the (V_a, α) that maximizes L/D (~9 m/s here).

---

## 6. Drag components and wing design parameters (L13 p.18–31; L14 p.5–15)

**Wing design parameters (L13/L14 p.5):** airfoil, area, planform (rectangle, trapezoidal, elliptical, Zimmerman, inverse Zimmerman, circular), aspect ratio, taper ratio, wing tip, wing sweep, dihedral.

**Airfoil selection criteria (L14 p.6):** high lift (C_l), low drag (C_d), low pitching moment (C_m).

**Finite-wing lift slope (L14 eq.(1)):**
$$ C_{L\alpha}\approx\frac{C_{l\alpha}}{1+\dfrac{C_{l\alpha}}{\pi AR}} $$
"Hint: Increase the aspect ratio (AR) of the wing." (C_Lα → C_lα as AR → ∞.)

**Reynolds number (L14 eq.(2)):** $Re=\dfrac{\bar c V_a\rho}{\mu}$, μ = 1.801×10⁻⁵ kg/(m·s) (instructor's value; we use 1.86×10⁻⁵ at 30 °C). Airfoil polars in XFLR5 must be run at the wing's actual Re.

**Drag polar (L14 eq.(3)–(4)):** drag = profile (independent of lift) + induced (due to lift):
$$ C_{Dw} = C_{Dw0} + \frac{C_{Lw}^2}{\pi\,AR\,e},\qquad 0.85\le e\le 0.95 $$
e (span efficiency) is maximum for the elliptical planform.

**Induced drag (L14 p.11):** pressure difference top-to-bottom causes spillage around the wing tips (tip vortices) → drag due to lift. Reduced by higher AR, better planform, wing tips.

**Planforms (L14 p.9):** elliptical, regular (rectangular), sweepback, high taper — shown with spanwise lift distribution; elliptical gives ideal distribution.

**Taper ratio (L14 p.12):** λ = c_tip/c_root ≤ 1. High λ → closer to rectangular → higher induced drag. Low λ reduces induced drag but reduces wing area for a given span.

**Wing tip (L14 p.13):** winglets/raked tips reduce induced drag.

**Wing sweep (L13 p.30 "Yaw stability", L14 p.14):** sweepback tends to correct yaw — the advancing wing presents more frontal area/drag and yaws the aircraft back into the wind.

**Dihedral (L13 p.31 "Roll stability", L14 p.15):** in positive sideslip the relative wind component v·sinΓ increases section lift on the lower (into-wind) wing and decreases it on the other ⇒ restoring rolling moment ⇒ roll stability.

**Inverse Zimmerman planform (L14 p.17–18):** ideal when the maximum linear dimension is restricted. Example: MAV, max dimension ≤ 15 cm:

| AR | a (cm) | b (cm) | c (cm) | Area (cm²) | (C_L/C_D)max | α at (C_L/C_D)max (deg) | C_Lmax | C_L^1.5/C_D (max) |
|---|---|---|---|---|---|---|---|---|
| 1.2 | 11 | 14.14 | 4 | 166.6 | 5.68 | 12.5 | 0.45 | 45.56 |
| 1.3 | 11 | 14.8 | 3.5 | 168.6 | 5.96 | 12 | 0.478 | 48.69 |
| 1.4 | 10.5 | 14.84 | 3 | 157.4 | 6.25 | 11.5 | 0.509 | 50.06 |
| 1.5 | 9.5 | 14.73 | 3 | 144.6 | 6.45 | 11 | 0.54 | 50.02 |
| 1.6 | 8.8 | 14.83 | 3 | 137.42 | 6.66 | 10.5 | 0.571 | 50.88 |
| 1.7 | 8.1 | 14.82 | 3 | 129.2 | 6.87 | 10 | 0.598 | 51.38 |
| 1.8 | 7.5 | 14.84 | 3 | 122.41 | 7.02 | 10 | 0.628 | 52 |

Lesson from the table: L/D and C_L^1.5/C_D rise monotonically with AR → this is the instructor's demonstration of "increase AR". The table format (AR sweep vs (C_L/C_D)max, α_opt, C_L, C_L^1.5/C_D) is a good template for our own XFLR5 AR trade study.

---

## 7. Endurance and range — electric (L13 eq.(23)–(25); L14 eq.(6)–(8))

L, D, C_L, C_D here are for the **whole UAV**. Energy E = P × t_e.
$$ t_e=\frac{E}{P}=\frac{E}{T V_a}=\frac{E\,(L/D)}{mg\,V_a} $$
With $L/D = C_L/C_D$ and $V_a=\sqrt{\dfrac{mg}{0.5\rho S C_L}}\propto \dfrac{1}{\sqrt{C_L}}$:
$$ t_e \propto \frac{C_L^{1.5}}{C_D}\quad\Rightarrow\ \text{fly at max } C_L^{3/2}/C_D \text{ for max endurance} $$
$$ R_e = t_e V_a = \frac{E\,(L/D)}{mg} = \frac{E\,(C_L/C_D)}{mg}\quad\Rightarrow\ \text{fly at max } C_L/C_D \text{ for max range} $$

**[SUPPLEMENT — not in slides] practical form:** $E = V_{batt}\cdot Q_{Ah}\cdot 3600\cdot f_{usable}$ (f_usable ≈ 0.8 for LiPo), and $P_{elec}=\dfrac{D V_a}{\eta_{prop}\eta_{motor}\eta_{ESC}}$ + avionics power, so $t_e=\dfrac{E\,\eta_{tot}}{D V_a + P_{avionics}\eta_{tot}}$. The slide formula assumes η = 1.

---

## 8. Pitching moment, trim and longitudinal stability (L12 p.24–27; L13 p.14–17; L14 p.4, 19–21)

**Moment contributions:**
- Wing: $C_{Mw}=C_{M\alpha}\alpha + C_{M0}$
- Horizontal tail: $C_{MH}=C_{MH\alpha}\alpha_H + C_{MH0}$
- Elevator: $C_{ME}=C_{ME}\delta_e + C_{ME0}$
- Total: $C_M = C_{Mw}+C_{MH}+C_{ME}$, $M_{YB}=C_M\tfrac12\rho V_a^2S_w\bar c$

**Steady flight conditions:** L = mg, T = D **and** M_YB = 0 ⇒ C_Mw + C_MH + C_ME = 0. Idea: make C_MH opposite in sign to C_Mw; use C_ME (elevator) for fine adjustment.

**Stability criterion (L12 p.27, L14 p.4):** *Centre of Gravity ahead of the Aerodynamic Centre* (AC = point about which the pitching moment is independent of α).

**Longitudinal trim with tail (L14 eq.(9)–(10)), C_ME ≈ 0:** (x_ac, x_cg measured from wing LE as fractions of c̄; l_t = distance CG→tail AC)
$$ -0.5\rho V_a^2S_w C_{m,L=0}\bar c - 0.5\rho V_a^2 S_w C_{Lw}(x_{ac}\bar c - x_{cg}\bar c) - 0.5\rho V_a^2 S_t C_{Lt} l_t = 0 $$
$$ -C_{m,L=0} - C_{Lw}(x_{ac}-x_{cg}) - \frac{S_t\,l_t}{S_w\,\bar c}\,C_{Lt} = 0 $$
**Horizontal tail volume ratio (L14 eq.(11)):** $V_{Ht}=\dfrac{S_t\,l_t}{S_w\,\bar c}$
NB (L14 p.20): **C_Lt < 0 is needed for trim** (with CG ahead of AC and a cambered wing with negative C_m0) ⇒ mount the horizontal tail at a negative angle of attack (negative incidence relative to wing / decalage) and/or use a negatively cambered tail airfoil.

**Historical V_H data (L14 p.21):** Cessna 172 0.76; Piper PA-46 0.66; Alenia G222 0.85; Fokker 100 1.07; Lake LA-250 0.8; Boeing 747 0.81; A340 1.11; Pilatus PC-12 1.08; Eurofighter 0.063; F/A-18 0.49.

**[SUPPLEMENT — not in slides] neutral point and static margin** (needed for the "CG estimate" deliverable and directly output by XFLR5 stability analysis):
$$ h_n \approx h_{ac,w} + \eta_t V_H\frac{C_{L\alpha,t}}{C_{L\alpha,w}}\left(1-\frac{d\epsilon}{d\alpha}\right),\quad SM = h_n - h_{cg},\quad C_{m\alpha} = -C_{L\alpha}\,SM<0 $$
with h_ac,w ≈ 0.25, dε/dα ≈ 2C_Lα,w/(πAR), η_t ≈ 0.9. Typical RC/UAV target SM ≈ 5–15 % c̄ (≈10 % for a beginner trainer). In XFLR5: the CG position at which C_m–α slope is zero is the neutral point; trim α is where C_m = 0 with C_L > 0.

---

## 9. Vertical tail and directional stability (L14 p.22–26)

Side force mainly from the vertical tail:
$$ F_y = C_y\tfrac12\rho V_a^2S_w = -C_{Lv}\tfrac12\rho V_a^2 S_v,\qquad C_y = C_{y\beta}\beta = -C_{Lv}\frac{S_v}{S_w},\quad C_{y\beta}<0 $$
Yawing moment:
$$ M_{ZB} = C_n\,0.5\rho V_a^2S_wb = (C_{n\beta}\beta)\,0.5\rho V_a^2S_wb $$
moment = side force × l_vt ⇒ $C_{n\beta}=-C_{y\beta}\dfrac{l_{vt}}{b}$, **C_nβ > 0** required (weathercock stability).
$$ M_{ZB}=C_{Lv}\,0.5\rho V_a^2S_v l_{vt} = C_{Lv}\frac{l_{vt}S_v}{bS_w}0.5\rho V_a^2S_wb $$
**Vertical tail volume ratio (L14 eq.(20)):** $V_{vt}=\dfrac{l_{vt}S_v}{b\,S_w}$ (l_vt = CG → vertical-tail AC).

**Typical tail volume coefficients (L14 p.26, Raymer table):**

| Type | c_HT | c_VT |
|---|---|---|
| **Sailplane** | **0.50** | **0.02** |
| Homebuilt | 0.50 | 0.04 |
| GA single engine | 0.70 | 0.04 |
| GA twin | 0.80 | 0.07 |
| Agricultural | 0.50 | 0.04 |
| Twin turboprop | 0.90 | 0.08 |
| Flying boat | 0.70 | 0.06 |
| Jet trainer | 0.70 | 0.06 |
| Jet fighter | 0.40 | 0.07 |
| Military cargo/bomber | 1.00 | 0.08 |
| Jet transport | 1.00 | 0.09 |

For a small powered glider, the sailplane/homebuilt rows apply: V_H ≈ 0.5 (small RC aircraft often use 0.4–0.6), V_V ≈ 0.02–0.04 (small RC: 0.03–0.04 recommended for good rudder authority in turns at low speed).

---

## 10. Control surface sizing (L14 p.27)

- **Elevator chord = 25–50 % of horizontal-tail chord; rudder chord = 25–50 % of vertical-tail chord.**
- **Aileron (Raymer historical guideline chart):** total aileron span / wing span vs aileron chord / wing chord. Read from chart: c_a/c = 0.10 → span fraction ≈ 0.70–0.90; 0.15 → ≈ 0.55–0.68; 0.20 → ≈ 0.47–0.55; 0.25 → ≈ 0.40–0.47; 0.30–0.34 → ≈ 0.35–0.42. Typical RC choice: c_a/c ≈ 0.20–0.25 with ailerons over ~50 % of span (outboard).

---

## 11. Flight performance relations needed for the 100 m × 20 m field — [SUPPLEMENT — not in slides]

These are not on L11–L14 but follow from the taught EOM (L11 eq.(1)) and lift equation; cite Beard & McLain Ch. 3–4 / Anderson.
- **Coordinated level turn:** $n = L/W = 1/\cos\phi$; $R = \dfrac{V_a^2}{g\tan\phi}$; $\dot\psi = \dfrac{g\tan\phi}{V_a}$; stall speed in turn $V_{s,turn}=V_{stall}\sqrt{n}$. A 180° turn needs lateral width 2R ⇒ for 20 m width, R ≤ ~8 m incl. margin ⇒ e.g. V = 8 m/s needs φ ≥ 39°, n ≥ 1.29 (tight!). This makes the 20 m width the dominant constraint: low wing loading / low cruise speed.
- **Climb:** $\sin\gamma = \dfrac{T-D}{W}$, rate of climb $RC = V_a\sin\gamma = \dfrac{P_a - P_{req}}{W}$ (P_a = T·V_a from L13 eq.(12)).
- **Glide (power off):** $\tan\gamma_{glide} = \dfrac{D}{L} = \dfrac{1}{L/D}$ ⇒ glide ratio = L/D; sink rate $V_s = V_a\sin\gamma \approx \sqrt{\dfrac{2W}{\rho S}}\dfrac{C_D}{C_L^{3/2}}$ (min sink at max C_L^1.5/C_D — same point as max electric endurance).
- **Hand launch:** launch speed ≈ 1.2–1.3 V_stall (thrown at ~8–10 m/s); thrust-to-weight T/W ≈ 0.5–0.7 recommended for safe climb-out from a hand launch.
- **Wing loading:** W/S directly sets V_stall and turn radius: $V_{stall}=\sqrt{2(W/S)/(\rho C_{Lmax})}$.

---

## 12. Design procedure as taught (step by step)

1. **CONOPS & requirement specs** (L11 p.5, 11): operating velocity, endurance/range, payload, wind, altitude, safety, maneuverability, launch method (hand launch). Map to flight phases: take-off, climb, cruise, loiter, descent, landing.
2. **Market survey / sizing & layout** (L11 p.11): pick configuration (conventional wing + horizontal + vertical tail, tractor/pusher), estimate total mass m.
3. **Wing design** (L13/L14 p.5): choose airfoil (high C_l, low C_d, low |C_m|) evaluated at the wing's Re = c̄ V_a ρ/μ; choose area S_w, planform, AR (high AR ⇒ higher C_Lα and lower induced drag), taper λ, tips, sweep (yaw stability), dihedral (roll stability). Compute c̄ from eq.(8).
4. **Aerodynamic model** (L11 eq.(9)–(14), L14 eq.(3)): obtain C_L(α), C_D(α) (drag polar C_D0 + C_L²/(πARe)), C_M(α) — via **XFLR5** (L12 p.32; next lecture "Using XFLR").
5. **Operating point** (L12 p.18–22): for each V_a solve C_L = mg/(½ρV_a²S_w) → α; compute D, L/D; check V_stall at C_Lmax; choose V_a at max L/D (min thrust, max range) or max C_L^1.5/C_D (max endurance) above stall with margin.
6. **Propulsion check** (L13 eq.(11)–(12)): T_a > D and P_a > D·V_a across the speed range; P_a vs P_req curves give V_max.
7. **Endurance/range** (L14 eq.(6)–(8)): t_e = E(L/D)/(mgV_a), R_e = E(C_L/C_D)/(mg) → size battery energy E.
8. **Longitudinal trim & stability** (L14 eq.(9)–(11)): CG ahead of AC; choose V_Ht (≈0.5 sailplane/homebuilt) → S_t l_t = V_Ht S_w c̄; tail at negative incidence / negative camber so C_Lt < 0 trims; verify C_M = 0 at the operating α with C_Mα < 0.
9. **Directional stability** (L14 eq.(12)–(20)): choose V_vt (0.02–0.04) → S_v l_vt = V_vt b S_w; check C_nβ > 0, C_yβ < 0.
10. **Control surfaces** (L14 p.27): elevator & rudder chord 25–50 % of tail chords; aileron chord/span from historical chart.
11. **Analysis & optimisation** in simulation (XFLR5) → preliminary → detailed design → fabricate → flight test → compare predictions with flight.
