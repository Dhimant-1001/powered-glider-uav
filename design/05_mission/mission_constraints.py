#!/usr/bin/env python3
"""
Mission & field-constraint analysis for the hand-launched powered glider
(EC4.402 Powered Glider UAV project, IIIT Hyderabad).

Translates "hand launch, climb, cruise, turn, hold altitude, descend and land
comfortably inside a 100 m x 20 m football ground" into quantitative
requirements on wing loading W/S, stall speed, CL_max, cruise/turn speed,
bank angle, roll authority, power-to-weight and approach glide ratio.

Run:  /opt/anaconda3/bin/python3 mission_constraints.py
Out:  fig1_constraint_diagram.png, fig2_turn_geometry.png,
      fig3_field_pattern.png, fig4_gust_roll.png, mission_results.json
      (all written next to this script)

Everything here is first-order (point-mass, parabolic polar). The XFLR5
polars produced later replace CL_max, CD0, e and L/D.
"""
import json
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))

# ---------------------------------------------------------------- palette
# Categorical order from the dataviz reference palette (fixed, not cycled)
C1, C2, C3, C4, C5, C6, C7, C8 = ("#2a78d6", "#eb6834", "#1baf7a", "#eda100",
                                  "#e87ba4", "#008300", "#6250d6", "#e34948")
INK, INK2, GRID = "#222222", "#5f5e58", "#d9d8d2"
plt.rcParams.update({
    "figure.dpi": 130, "axes.edgecolor": INK2, "axes.labelcolor": INK,
    "xtick.color": INK2, "ytick.color": INK2, "axes.grid": True,
    "grid.color": GRID, "grid.linewidth": 0.6, "axes.spines.top": False,
    "axes.spines.right": False, "font.size": 9, "lines.linewidth": 2.0,
    "legend.frameon": False,
})

# ---------------------------------------------------------------- inputs
g = 9.81
RHO = 1.09          # Hyderabad, ~540 m, ~30 C
RHO_HOT = 1.06      # hot afternoon (~40 C)
NU = 1.7e-5         # kinematic viscosity

# Preliminary aero (to be replaced by XFLR5 results)
CLMAX = 1.00                     # design 3D clean CL_max (range 0.9-1.15)
CLMAX_RANGE = (0.90, 1.15)
CD0 = 0.030                      # whole aircraft, low Re, fixed prop/fuselage
AR = 11.0
E_OSW = 0.85
K_IND = 1.0 / (np.pi * AR * E_OSW)
CLA = 2 * np.pi * AR / (AR + 2.0 + 0.0) * 0.95   # ~5.0 /rad finite wing (Helmbold-like, low-Re knockdown)
ETA_TOT = 0.45                   # prop x motor x ESC at 7-10 m/s (assumed)
CBAR = 0.15                      # representative MAC [m] for gust/Re

# Field & mission
FIELD_L, FIELD_W = 100.0, 20.0
MARGIN_SIDE = 3.0                # each side -> usable 14 m (target)
MARGIN_SIDE_MIN = 2.0            # each side -> 16 m (absolute max)
MARGIN_END = 5.0
D_TARGET = FIELD_W - 2 * MARGIN_SIDE       # 14 m
D_MAX = FIELD_W - 2 * MARGIN_SIDE_MIN      # 16 m
WIND = 3.0

# Safety factors
K_TURN = 1.15      # V_turn >= 1.15 * V_stall(turn)  (CL_turn <= 0.76 CL_max)
K_LAUNCH = 1.15    # V_throw >= 1.15 * V_stall (motor at full power at release)
K_CRUISE = 1.30    # V_cruise >= 1.3 V_stall

V_THROW = 7.0      # assumed achievable untrained over-arm throw (+ few steps); 6 = pessimistic
V_THROW_LOW = 6.0

PHI_DESIGN = 40.0  # deg, design bank for 180 deg turns
P_ROLL = 60.0      # deg/s, design roll rate (roll-in modelled as a ramp)

V_CRUISE = 8.0
V_TURN = 7.5
ROC_REQ = 2.0      # m/s climb at V_CRUISE


# ---------------------------------------------------------------- helpers
def stall_speed(ws, clmax=CLMAX, rho=RHO):
    return np.sqrt(2 * ws / (rho * clmax))


def turn_radius(V, phi_deg):
    return V ** 2 / (g * np.tan(np.radians(phi_deg)))


def turn_track(V, phi_deg, p_degps=P_ROLL, dpsi_deg=180.0, wind=(0.0, 0.0),
               dt=0.002):
    """Ground track of a 180-deg turn starting on +x heading, turning left (+y),
    with linear roll-in at p, constant bank, symmetric roll-out.
    Returns dict with lateral/along extents (incl. straight leg at y=0)."""
    phi_max = np.radians(phi_deg)
    p = np.radians(p_degps)
    t_roll = phi_max / p
    # heading change during roll-in
    ts = np.arange(0, t_roll, dt)
    psi_in = np.sum(g * np.tan(p * ts) / V) * dt
    psi_target = np.radians(dpsi_deg)
    x = y = psi = t = 0.0
    xs, ys = [0.0], [0.0]
    phase = "in"
    tr = 0.0
    while True:
        if phase == "in":
            phi = min(p * tr, phi_max)
            if tr >= t_roll:
                phase = "hold"
        if phase == "hold":
            phi = phi_max
            if psi >= psi_target - psi_in:
                phase, tr = "out", 0.0
        if phase == "out":
            phi = max(phi_max - p * tr, 0.0)
            if phi <= 0.0:
                break
        psi += g * np.tan(phi) / V * dt
        x += (V * np.cos(psi) + wind[0]) * dt
        y += (V * np.sin(psi) + wind[1]) * dt
        xs.append(x)
        ys.append(y)
        tr += dt
        t += dt
    xs, ys = np.array(xs), np.array(ys)
    return dict(x=xs, y=ys, lat=ys.max() - min(ys.min(), 0.0),
                along=xs.max() - min(xs.min(), 0.0), t=t, psi_end=np.degrees(psi))


def ws_turn_limit(V, phi_deg, clmax=CLMAX, rho=RHO, k=K_TURN):
    """Max W/S such that V >= k * Vs * sqrt(n) in a level turn at bank phi."""
    return 0.5 * rho * clmax * V ** 2 * np.cos(np.radians(phi_deg)) / k ** 2


def v_max_for_width(phi_deg, width, p=P_ROLL):
    """Largest speed whose 180-deg turn (with roll-in/out) fits laterally in width."""
    lo, hi = 3.0, 20.0
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        if turn_track(mid, phi_deg, p, dt=0.004)["lat"] <= width:
            lo = mid
        else:
            hi = mid
    return lo


def power_to_weight(ws, V, n=1.0, roc=0.0, rho=RHO, cd0=CD0, k=K_IND, eta=ETA_TOT):
    """Electrical W per kg of mass for level/turning flight + climb rate."""
    q = 0.5 * rho * V ** 2
    p_per_N = V * (q * cd0 / ws + k * n ** 2 * ws / q) + roc   # W per N of weight
    return p_per_N * g / eta                                   # W per kg


def pratt_gust(ws, V, U, rho=RHO, a=CLA, c=CBAR):
    mu = 2 * ws / (rho * c * a * g)
    Kg = 0.88 * mu / (5.3 + mu)
    dn = rho * V * a * Kg * U / (2 * ws)
    dCL = a * Kg * U / V
    return dn, dCL, Kg


# ---------------------------------------------------------------- 1. turns
results = {}
phis = [30, 35, 40, 45, 50]
vmax_target = {ph: v_max_for_width(ph, D_TARGET) for ph in phis}
vmax_max = {ph: v_max_for_width(ph, D_MAX) for ph in phis}
ws_turn_target = {ph: ws_turn_limit(vmax_target[ph], ph) for ph in phis}
results["turn"] = {
    f"phi{ph}": dict(V_max_14m=round(vmax_target[ph], 2),
                     V_max_16m=round(vmax_max[ph], 2),
                     n=round(1 / np.cos(np.radians(ph)), 3),
                     WS_max_14m_CL1=round(ws_turn_target[ph], 1),
                     WS_max_14m_CL1_hot=round(ws_turn_limit(vmax_target[ph], ph, rho=RHO_HOT), 1))
    for ph in phis}

# ---------------------------------------------------------------- 2. bounds
ws_launch = 0.5 * RHO * CLMAX * (V_THROW / K_LAUNCH) ** 2
ws_launch_low = 0.5 * RHO * CLMAX * (V_THROW_LOW / K_LAUNCH) ** 2
ws_turn40 = ws_turn_target[PHI_DESIGN]
ws_turn40_hot = ws_turn_limit(vmax_target[PHI_DESIGN], PHI_DESIGN, rho=RHO_HOT)
# efficiency / wind-penetration lower bound: at the 8 m/s cruise speed needed to
# penetrate 3 m/s wind (V >= 2.5 Vw) the aircraft should still fly at
# L/D >= 75 % of L/D_max  ->  CL_cr >= x * CL*, 2x/(1+x^2) = 0.75
CL_LDMAX = np.sqrt(CD0 / K_IND)
_x = (2 / 0.75 - np.sqrt((2 / 0.75) ** 2 - 4)) / 2
ws_wind = 0.5 * RHO * V_CRUISE ** 2 * _x * CL_LDMAX
# buildability: 3D-printed LW-PLA aircraft of 1.2-1.4 m span reach 15-24 g/dm^2
ws_build = 14.0 * 0.981   # 14 g/dm^2 -> N/m^2 (=13.7)
results["bounds_N_m2"] = dict(launch_7ms=round(ws_launch, 1), launch_6ms=round(ws_launch_low, 1),
                              turn40_14m=round(ws_turn40, 1), turn40_14m_hot=round(ws_turn40_hot, 1),
                              eff_LD75pct_at_8ms=round(ws_wind, 1), build_min=round(ws_build, 1),
                              CL_LDmax=round(CL_LDMAX, 3))

# ---------------------------------------------------------------- design point
WS_D = 16.0                        # N/m^2 design (=16.3 g/dm^2)
Vs = stall_speed(WS_D)
Vs_hot = stall_speed(WS_D, rho=RHO_HOT)
Vs_cl09 = stall_speed(WS_D, clmax=0.9)
n_d = 1 / np.cos(np.radians(PHI_DESIGN))
LD_max_parab = 0.5 / np.sqrt(K_IND * CD0)
V_LD = np.sqrt(2 * WS_D / (RHO * CL_LDMAX))
CL_cr = 2 * WS_D / (RHO * V_CRUISE ** 2)
LD_cr = CL_cr / (CD0 + K_IND * CL_cr ** 2)
CL_t = 2 * WS_D * n_d / (RHO * V_TURN ** 2)
trk = turn_track(V_TURN, PHI_DESIGN)
trk_cross = turn_track(V_TURN, PHI_DESIGN, wind=(0.0, WIND))
trk_along = turn_track(V_TURN, PHI_DESIGN, wind=(-WIND, 0.0))
Re_cr = V_CRUISE * CBAR / NU
Re_st = Vs * CBAR / NU
# sensitivity of turn width to roll rate and crosswind
sens = {}
for ph in (40, 45, 50):
    for p_ in (40, 60, 90):
        sens[f"phi{ph}_p{p_}_calm"] = round(turn_track(V_TURN, ph, p_)["lat"], 1)
    for w_ in (1.0, 1.5, 2.0, 3.0):
        sens[f"phi{ph}_p60_cross{w_}"] = round(turn_track(V_TURN, ph, 60, wind=(0, w_))["lat"], 1)
        sens[f"phi{ph}_p60_cross{w_}_upwindturn"] = round(turn_track(V_TURN, ph, 60, wind=(0, -w_))["lat"], 1)
results["turn_sensitivity_V7p5"] = sens
results["design_point"] = dict(
    WS_N_m2=WS_D, WS_g_dm2=round(WS_D / 0.981, 1), Vs=round(Vs, 2), Vs_hot=round(Vs_hot, 2),
    Vs_CL0p9=round(Vs_cl09, 2), V_cruise=V_CRUISE, CL_cruise=round(CL_cr, 3),
    LD_cruise_parab=round(LD_cr, 1), LDmax_parab=round(LD_max_parab, 1),
    V_LDmax=round(V_LD, 2), CL_LDmax=round(CL_LDMAX, 3),
    V_turn=V_TURN, phi=PHI_DESIGN, n=round(n_d, 3), R=round(turn_radius(V_TURN, PHI_DESIGN), 2),
    CL_turn=round(CL_t, 3), turn_margin_V_over_Vsturn=round(V_TURN / (Vs * np.sqrt(n_d)), 3),
    turn_lat_calm=round(trk["lat"], 2), turn_time=round(trk["t"], 2),
    turn_lat_crosswind3=round(trk_cross["lat"], 2), turn_along_wind3=round(trk_along["along"], 2),
    launch_margin=round(V_THROW / Vs, 2), launch_margin_6ms=round(V_THROW_LOW / Vs, 2),
    Re_cruise=int(Re_cr), Re_stall=int(Re_st))

# implied mass for spans (the construction agent's weight budget)
mass_tab = {}
for ar in (9, 11, 13):
    for b in (1.2, 1.4, 1.6, 1.8):
        S = b ** 2 / ar
        mass_tab[f"AR{ar}_b{b}"] = dict(S_m2=round(S, 3), c=round(b / ar, 3),
                                        m_at_WSd_g=int(WS_D * S / g * 1000),
                                        m_at_14gdm2=int(14 * S * 100), m_at_18gdm2=int(18 * S * 100))
results["mass_budget"] = mass_tab

# ---------------------------------------------------------------- 3. power
pw_cruise = power_to_weight(WS_D, V_CRUISE)
pw_turn = power_to_weight(WS_D, V_TURN, n=n_d)
pw_climb = power_to_weight(WS_D, V_CRUISE, roc=ROC_REQ)
pw_climb3 = power_to_weight(WS_D, V_CRUISE, roc=3.0)
# launch: accelerate from throw to 1.3 Vs in ~1.5 s while climbing at 8 deg
D_over_W = 1 / LD_cr
T_W_launch = D_over_W + (K_CRUISE * Vs - V_THROW_LOW) / 1.5 / g + np.sin(np.radians(8))
results["power"] = dict(P_W_cruise_Wkg=round(pw_cruise, 1), P_W_turn40_Wkg=round(pw_turn, 1),
                        P_W_climb2_Wkg=round(pw_climb, 1), P_W_climb3_Wkg=round(pw_climb3, 1),
                        T_W_dynamic_launch_min=round(max(T_W_launch, D_over_W + np.sin(np.radians(8))), 2),
                        eta_total_assumed=ETA_TOT)

# ---------------------------------------------------------------- 4. pattern
R_d = turn_radius(V_TURN, PHI_DESIGN)
usable_L = FIELD_L - 2 * MARGIN_END
along_turn = trk["along"]                   # x-overshoot of a turn incl. roll-in
L_straight = usable_L - 2 * along_turn
t_lap = 2 * L_straight / V_CRUISE + 2 * trk["t"]
pattern = dict(usable_length=usable_L, straight_leg=round(L_straight, 1),
               t_straight=round(L_straight / V_CRUISE, 2), t_turn=round(trk["t"], 2),
               lap_time=round(t_lap, 1), laps_60s=round(60 / t_lap, 2),
               laps_120s=round(120 / t_lap, 2), frac_time_banked=round(2 * trk["t"] / t_lap, 2))
# climb-out
gam = np.degrees(np.arcsin(ROC_REQ / V_CRUISE))
pattern.update(climb_angle_deg=round(gam, 1), t_to_20m=round(20 / ROC_REQ, 1),
               ground_dist_to_20m_calm=round(20 / np.tan(np.radians(gam)), 1),
               ground_dist_to_20m_headwind3=round(20 / ROC_REQ * (V_CRUISE * np.cos(np.radians(gam)) - WIND), 1))
results["pattern"] = pattern

# ---------------------------------------------------------------- 5. landing
MU_GRASS = 0.4   # belly skid on mown grass (assumed 0.3-0.5)
landing = {}
V_app = 1.3 * Vs
V_td = 1.1 * Vs
for name, ld in (("clean_LD12", 12.0), ("parab_LD", LD_max_parab), ("spoileron_LD7", 7.0),
                 ("crow_LD5", 5.0)):
    for h in (5.0, 10.0):
        landing[f"{name}_h{int(h)}"] = round(h * ld, 1)
float_d = (V_app ** 2 - V_td ** 2) * 15.0 / (2 * g)        # decel in ground effect, L/D~15
roll_d = V_td ** 2 / (2 * MU_GRASS * g)
roll_d_headwind = (V_td - WIND) ** 2 / (2 * MU_GRASS * g)
landing.update(V_approach=round(V_app, 2), V_touchdown=round(V_td, 2),
               float_m=round(float_d, 1), ground_roll_m=round(roll_d, 1),
               ground_roll_headwind3_m=round(roll_d_headwind, 1),
               sink_clean_LD12=round(V_app / 12.0, 2),
               sink_spiral_phi30=round(V_TURN / 10.0 * (1 / np.cos(np.radians(30))) ** 1.5, 2))
landing["total_from_5m_spoileron"] = round(5 * 7 + float_d + roll_d, 1)
landing["total_from_5m_clean"] = round(5 * 12 + float_d + roll_d, 1)
results["landing"] = landing

# ---------------------------------------------------------------- 6. gust
gust = {}
for U in (1.0, 1.5, 2.0):
    dn, dCL, Kg = pratt_gust(WS_D, V_CRUISE, U)
    gust[f"U{U}"] = dict(dn=round(dn, 2), dCL=round(dCL, 3), Kg=round(Kg, 3),
                         CL_peak=round(CL_cr + dCL, 3))
results["gust"] = gust

# ---------------------------------------------------------------- 7. roll
roll = {}
eta1, eta2, tau = 0.55, 0.95, 0.45      # aileron 55-95 % semispan, cf/c = 0.25
Cl_da = CLA * tau * (eta2 ** 2 - eta1 ** 2) / 4
Cl_p = -CLA / 6 * 1.0                     # rectangular / mild taper
KCORR = 0.70                              # strip theory over-predicts (tip loss, hinge gap, flex)
for da in (12, 15, 20):
    pb2v = KCORR * Cl_da / abs(Cl_p) * np.radians(da)
    for b in (1.4, 1.6, 1.8):
        p = pb2v * 2 * V_CRUISE / b
        roll[f"da{da}_b{b}"] = dict(pb2V=round(pb2v, 3), p_degps=round(np.degrees(p), 1),
                                    t_0_to_40=round(40 / np.degrees(p), 2),
                                    t_40_to_m40=round(80 / np.degrees(p), 2))
results["roll"] = dict(Cl_da=round(Cl_da, 3), Cl_p=round(Cl_p, 3), table=roll)

# ================================================================= FIGURES
WS = np.linspace(6, 40, 400)
gdm = lambda x: x / 0.981

# ---- Fig 1: constraint diagram (two panels)
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4.8))
# (a) required CL_max vs W/S
Vt40 = vmax_target[40]
req_turn = lambda ph, rho=RHO: K_TURN ** 2 * WS / (0.5 * rho * vmax_target[ph] ** 2 * np.cos(np.radians(ph)))
req_launch = lambda Vt: K_LAUNCH ** 2 * WS / (0.5 * RHO * Vt ** 2)
ax1.axhspan(*CLMAX_RANGE, color=C1, alpha=0.10, lw=0)
ax1.text(7, 1.165, "available 3D CL_max (low-Re cambered airfoil), 0.90-1.15", color=INK2, fontsize=8)
ax1.plot(WS, req_turn(40), color=C2, label="180° turn fits 14 m @ 40° bank (V≥1.15 V_s,turn)")
ax1.plot(WS, req_turn(45), color=C2, ls="--", lw=1.5, label="same @ 45° bank")
ax1.plot(WS, req_turn(40, RHO_HOT), color=C2, ls=":", lw=1.5, label="40° bank, hot day ρ=1.06")
ax1.plot(WS, req_launch(V_THROW), color=C3, label="hand launch 7 m/s ≥ 1.15 V_s")
ax1.plot(WS, req_launch(V_THROW_LOW), color=C3, ls="--", lw=1.5, label="hand launch 6 m/s (weak throw)")
ax1.axvspan(6, ws_build, color=INK2, alpha=0.08, lw=0)
ax1.text(6.3, 1.62, "hard to build\n(3D-printed\nairframe)", color=INK2, fontsize=8)
ax1.axvspan(14, 18, color=C6, alpha=0.10, lw=0)
ax1.plot([WS_D], [CLMAX], "o", ms=9, color=INK, mec="white", mew=2, zorder=5)
ax1.annotate(f"design point\nW/S={WS_D:.0f} N/m² ({gdm(WS_D):.1f} g/dm²)", (WS_D, CLMAX),
             xytext=(16.5, 1.45), arrowprops=dict(arrowstyle="-", color=INK2), fontsize=8.5, color=INK)
ax1.set_xlim(6, 30); ax1.set_ylim(0.3, 1.8)
ax1.set_xlabel("Wing loading W/S [N/m²]")
ax1.set_ylabel("Required 3D CL_max [-]")
ax1.set_title("(a) Field constraints → required CL_max", loc="left", fontsize=10, color=INK)
ax1.legend(loc="lower right", fontsize=7.5)
sec = ax1.secondary_xaxis("top", functions=(gdm, lambda x: x * 0.981))
sec.set_xlabel("W/S [g/dm²]", color=INK2)

# (b) power-to-weight vs W/S
ax2.plot(WS, power_to_weight(WS, V_CRUISE), color=C1, label=f"level cruise {V_CRUISE:.0f} m/s")
ax2.plot(WS, power_to_weight(WS, V_TURN, n=1 / np.cos(np.radians(40))), color=C2,
         label=f"sustained 40° turn @ {V_TURN} m/s")
ax2.plot(WS, power_to_weight(WS, V_CRUISE, roc=ROC_REQ), color=C3, label="climb 2 m/s @ 8 m/s")
ax2.plot(WS, power_to_weight(WS, V_CRUISE, roc=3.0), color=C3, ls="--", lw=1.5, label="climb 3 m/s @ 8 m/s")
ub = min(ws_turn40, ws_launch)
ax2.axvline(ws_turn40, color=C2, lw=1.2, ls=":")
ax2.axvline(ws_launch, color=C4, lw=1.2, ls=":")
ax2.axvline(ws_build, color=INK2, lw=1.2, ls=":")
ax2.axvline(ws_wind, color=C1, lw=1.2, ls=":")
ax2.text(ws_wind + 0.2, 40, f"L/D≥75%\nmax @8 m/s\n{ws_wind:.1f}", fontsize=7.5, color=INK2)
ax2.text(ws_turn40 + 0.3, 100, f"turn limit\nCL_max=1.0\n{ws_turn40:.1f}", fontsize=7.5, color=INK2)
ax2.text(ws_launch + 0.3, 92, f"launch limit\n{ws_launch:.1f}", fontsize=7.5, color=INK2)
ax2.text(ws_build - 3.2, 40, f"build\nmin\n{ws_build:.1f}", fontsize=7.5, color=INK2)
env = np.maximum.reduce([power_to_weight(WS, V_CRUISE, roc=ROC_REQ),
                         power_to_weight(WS, V_TURN, n=1 / np.cos(np.radians(40)))])
m = (WS >= max(ws_build, ws_wind)) & (WS <= ub)
ax2.fill_between(WS[m], env[m] * 1.5, 130, color=C6, alpha=0.12, lw=0)
ax2.text(ws_build + 0.3, 124, "feasible (×1.5 power margin)", color=C6, fontsize=8)
ax2.plot([WS_D], [power_to_weight(WS_D, V_CRUISE, roc=ROC_REQ) * 1.5], "o", ms=9,
         color=INK, mec="white", mew=2, zorder=5)
ax2.set_xlim(6, 30); ax2.set_ylim(0, 130)
ax2.set_xlabel("Wing loading W/S [N/m²]")
ax2.set_ylabel("Electrical power / mass [W/kg]  (η_total=0.45)")
ax2.set_title("(b) Power-loading constraint diagram", loc="left", fontsize=10, color=INK)
ax2.legend(loc="lower right", fontsize=7.5)
sec2 = ax2.secondary_xaxis("top", functions=(gdm, lambda x: x * 0.981))
sec2.set_xlabel("W/S [g/dm²]", color=INK2)
fig.tight_layout()
fig.savefig(os.path.join(HERE, "fig1_constraint_diagram.png"))
plt.close(fig)

# ---- Fig 2: turn geometry
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4.6))
Vv = np.linspace(5, 12, 60)
cols = {30: C1, 40: C2, 45: C3, 50: C7}
for ph, c in cols.items():
    ax1.plot(Vv, 2 * turn_radius(Vv, ph), color=c, lw=1.4, ls="--")
    lat = [turn_track(v, ph, dt=0.004)["lat"] for v in Vv[::4]]
    ax1.plot(Vv[::4], lat, color=c, label=f"{ph}° bank")
ax1.axhspan(0, D_TARGET, color=C6, alpha=0.08, lw=0)
ax1.axhline(D_TARGET, color=C6, lw=1.2); ax1.axhline(D_MAX, color=C8, lw=1.2)
ax1.axhline(FIELD_W, color=INK, lw=1.2)
ax1.text(5.1, D_TARGET - 1.3, "target ≤14 m (3 m margins)", color=C6, fontsize=8)
ax1.text(5.1, D_MAX + 0.4, "max 16 m (2 m margins)", color=C8, fontsize=8)
ax1.text(5.1, FIELD_W + 0.4, "field width 20 m", color=INK, fontsize=8)
ax1.set_xlabel("True airspeed in turn V [m/s]")
ax1.set_ylabel("Width used by 180° turn [m]")
ax1.set_title("(a) Turn width: solid = with 60°/s roll-in/out, dashed = 2R", loc="left", fontsize=10)
ax1.set_ylim(0, 40); ax1.legend(loc="upper left")
# (b) max W/S allowed by turn vs bank for CLmax values
phv = np.arange(25, 61, 2.5)
vm = np.array([v_max_for_width(p, D_TARGET) for p in phv])
for cl, c, ls in ((0.9, C1, "--"), (1.0, C2, "-"), (1.15, C3, "--")):
    ax2.plot(phv, ws_turn_limit(vm, phv, clmax=cl), color=c, ls=ls, label=f"CL_max={cl}")
ax2.plot(phv, ws_turn_limit(vm, phv, clmax=1.0, rho=RHO_HOT), color=C2, ls=":", lw=1.4,
         label="CL_max=1.0, hot day")
ax2.set_xlabel("Bank angle φ [deg]")
ax2.set_ylabel("Max W/S so the 14 m turn keeps V ≥ 1.15 V_s,turn [N/m²]")
ax2.set_title("(b) Turn-limited wing loading vs bank", loc="left", fontsize=10)
for p_, v_ in zip(phv[::4], vm[::4]):
    ax2.annotate(f"{v_:.1f} m/s", (p_, ws_turn_limit(v_, p_, clmax=1.15)), xytext=(0, 6),
                 textcoords="offset points", ha="center", fontsize=7, color=INK2)
ax2.text(25, 23.2, "labels on CL_max=1.15 curve: max turn speed that fits 14 m", fontsize=7.5, color=INK2)
ax2.set_ylim(8, 24); ax2.legend(loc="lower right", fontsize=8)
fig.tight_layout()
fig.savefig(os.path.join(HERE, "fig2_turn_geometry.png"))
plt.close(fig)

# ---- Fig 3: field plan view + landing profile
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 7.5), gridspec_kw=dict(height_ratios=[1.1, 1]))
ax1.add_patch(plt.Rectangle((0, 0), FIELD_L, FIELD_W, fc="#e9f3e4", ec=C6, lw=1.5))
ax1.add_patch(plt.Rectangle((MARGIN_END, MARGIN_SIDE), usable_L, D_TARGET, fc="none",
                            ec=INK2, lw=1, ls="--"))
y0 = MARGIN_SIDE + (D_TARGET - trk["lat"]) / 2
xa, xb = MARGIN_END + along_turn, FIELD_L - MARGIN_END - along_turn
# calm racetrack (counter-clockwise)
ax1.plot([xa, xb], [y0, y0], color=C1)
ax1.plot(xb + trk["x"], y0 + trk["y"], color=C1, label="calm-air racetrack (7.5 m/s, 40°, 60°/s roll)")
ax1.plot([xb, xa], [y0 + trk["y"][-1]] * 2, color=C1)
ax1.plot(xa - trk["x"], y0 + trk["y"][-1] - trk["y"], color=C1)
# crosswind turn
ax1.plot(xb + trk_cross["x"], y0 + trk_cross["y"], color=C8, lw=1.6,
         label=f"same turn, 3 m/s crosswind (width {trk_cross['lat']:.1f} m)")
ax1.annotate("hand launch\n(into wind)", (4, 10), xytext=(-2, 21.5), fontsize=8, color=INK,
             arrowprops=dict(arrowstyle="->", color=INK2))
ax1.text(MARGIN_END + 1, MARGIN_SIDE + 0.4, "usable box 90 × 14 m", fontsize=8, color=INK2)
ax1.set_aspect("equal"); ax1.set_xlim(-5, 105); ax1.set_ylim(-4, 26)
ax1.set_xlabel("x along field [m]"); ax1.set_ylabel("y [m]")
ax1.set_title(f"(a) Plan view: lap {pattern['lap_time']} s, straight legs {pattern['straight_leg']} m",
              loc="left", fontsize=10)
ax1.legend(loc="upper right", fontsize=7.5, ncol=1)
# landing profile
xs = np.linspace(0, 140, 300)
for name, ld, c in (("clean L/D≈12", 12, C1), ("spoilerons up, L/D≈7", 7, C2), ("crow / full spoiler L/D≈5", 5, C3)):
    hh = np.clip(10 - xs / ld, 0, None)
    ax2.plot(xs, hh, color=c, label=name)
ax2.axvspan(0, FIELD_L, color=C6, alpha=0.08, lw=0)
ax2.axvline(FIELD_L, color=C6, lw=1.2)
ax2.text(FIELD_L + 1, 8.5, "end of field\n(100 m)", fontsize=8, color=C6)
ax2.set_xlabel("Distance from start of final glide [m]"); ax2.set_ylabel("Height [m]")
ax2.set_title(f"(b) Final glide from 10 m (+ float ≈{float_d:.0f} m, belly roll ≈{roll_d:.0f} m)",
              loc="left", fontsize=10)
ax2.set_xlim(0, 140); ax2.set_ylim(0, 11); ax2.legend(loc="upper right")
fig.tight_layout()
fig.savefig(os.path.join(HERE, "fig3_field_pattern.png"))
plt.close(fig)

# ---- Fig 4: gust & roll authority
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4.4))
for U, c in ((1.0, C1), (1.5, C2), (2.0, C3)):
    dn = [pratt_gust(w, V_CRUISE, U)[0] for w in WS]
    ax1.plot(WS, dn, color=c, label=f"vertical gust {U} m/s")
ax1.axvline(WS_D, color=INK2, ls=":", lw=1.2)
ax1.set_xlabel("W/S [N/m²]"); ax1.set_ylabel("Gust load-factor increment Δn (Pratt, V=8 m/s)")
ax1.set_title("(a) Gust sensitivity falls as W/S rises", loc="left", fontsize=10)
ax1.set_xlim(6, 40); ax1.legend()
# min cruise speed for gust-stall margin
Vgrid = np.linspace(3, 14, 400)
for U, c in ((1.0, C1), (1.5, C2)):
    vmin = []
    for w in WS:
        ok = [v for v in Vgrid if 2 * w / (RHO * v ** 2) + pratt_gust(w, v, U)[1] <= CLMAX]
        vmin.append(ok[0] if ok else np.nan)
    ax2.plot(WS, vmin, color=c, label=f"min V so CL + ΔCL_gust({U} m/s) ≤ CL_max")
ax2.plot(WS, K_CRUISE * stall_speed(WS), color=C7, label="1.3 V_stall")
ax2.plot(WS, [vmax_target[40]] * len(WS), color=C8, ls="--", lw=1.4,
         label=f"max turn speed for 14 m @40° ({vmax_target[40]:.1f} m/s)")
ax2.axvline(WS_D, color=INK2, ls=":", lw=1.2)
ax2.set_xlabel("W/S [N/m²]"); ax2.set_ylabel("Airspeed [m/s]")
ax2.set_title("(b) Speed window: gust-stall margin vs field turn limit", loc="left", fontsize=10)
ax2.set_xlim(6, 30); ax2.set_ylim(4, 12); ax2.legend(fontsize=7.5, loc="upper left")
fig.tight_layout()
fig.savefig(os.path.join(HERE, "fig4_gust_speed_window.png"))
plt.close(fig)

with open(os.path.join(HERE, "mission_results.json"), "w") as f:
    json.dump(results, f, indent=1)
print(json.dumps(results, indent=1))
