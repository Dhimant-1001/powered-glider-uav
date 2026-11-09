#!/usr/bin/env python3
"""
propulsion_model.py  --  Parametric propulsion sizing for the small hand-launched
powered glider (EC4.402 Intro to UAV Design, IIIT Hyderabad).

What it does
------------
1. Aircraft drag polar  CD = CD0 + CL^2/(pi e AR)  ->  D(V), P_req(V) = D V  (lecture L13 eq.12)
2. Requirement path (no hardware): level / climb / turn / launch thrust & power,
   recommended static thrust and an electrical power estimate.
3. Hardware path: real propeller data (UIUC Propeller Data Site, Aeronaut CAM
   folding blades + APC E) + 3-constant DC motor model (KV, Rm, I0) + ESC and
   battery resistance  ->  thrust/current/rpm at any airspeed and throttle.
4. Mission energy model for the 100 m x 20 m football-ground racetrack ->
   battery capacity and endurance.
5. Ranks a catalogue of motor/prop/battery combos purchasable in India and
   prints the best one for the given airframe.
6. Validates the motor+prop model against T-Motor AT2312 bench data.

Usage
-----
    python3 propulsion_model.py                         # default design + all tables/plots
    python3 propulsion_model.py --mass 0.50 --span 1.4 --AR 10 --CD0 0.025 --e 0.85

    from propulsion_model import Airframe, requirements, suggest_combo
    ac = Airframe(mass=0.5, S=0.196, AR=10, CD0=0.025, e=0.85)
    print(requirements(ac)); print(suggest_combo(ac)[0])

All SI units unless a name says otherwise (_g, _mAh, _in).
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import os
from dataclasses import dataclass, field, asdict

import numpy as np
from scipy.interpolate import PchipInterpolator
from scipy.optimize import brentq

HERE = os.path.dirname(os.path.abspath(__file__))
PROP_DIR = os.path.join(HERE, "prop_data")

# ----------------------------------------------------------------------------
# Site air (Hyderabad ~540 m, ~30 C)
# ----------------------------------------------------------------------------
RHO = 1.09          # kg/m^3 design density (hot day ~1.06)
G = 9.81
INCH = 0.0254

# Motor-model calibration (fitted in validate() to T-Motor AT2312 bench data and
# checked against SunnySky X2212-980KV / 10x4.7 bench point):
#   effective loaded KV = KV_EFF * catalogue KV  (ESC timing, real KV < label KV)
#   R_ESC_EFF = ESC FETs + wiring + connectors + ESC losses lumped as a resistance
KV_EFF = 0.88
R_ESC_EFF = 0.07


# ============================================================================
# 1. AIRFRAME / DRAG POLAR
# ============================================================================
@dataclass
class Airframe:
    mass: float = 0.50          # kg, all-up weight
    S: float = 0.196            # m^2 wing area
    AR: float = 10.0            # aspect ratio
    CD0: float = 0.025          # whole-aircraft zero-lift drag coefficient
    e: float = 0.85             # Oswald efficiency (whole aircraft)
    CLmax: float = 1.10         # whole-aircraft max lift coefficient (low Re)
    CL_cruise: float = 0.60     # design cruise lift coefficient
    rho: float = RHO

    @property
    def W(self):
        return self.mass * G

    @property
    def span(self):
        return math.sqrt(self.AR * self.S)

    @property
    def WS(self):
        return self.W / self.S

    @property
    def k(self):
        return 1.0 / (math.pi * self.e * self.AR)

    def CL(self, V, n=1.0):
        return 2 * n * self.W / (self.rho * V ** 2 * self.S)

    def drag(self, V, n=1.0):
        """Total drag [N] at airspeed V and load factor n (n=1/cos(phi) in a level turn)."""
        q = 0.5 * self.rho * V ** 2
        CL = n * self.W / (q * self.S)
        return q * self.S * (self.CD0 + self.k * CL ** 2)

    def p_req(self, V, n=1.0):
        """Aerodynamic power required [W] = D*V (lecture L13 eq. 12)."""
        return self.drag(V, n) * V

    def v_stall(self, n=1.0):
        return math.sqrt(2 * n * self.W / (self.rho * self.S * self.CLmax))

    def v_at_CL(self, CL):
        return math.sqrt(2 * self.W / (self.rho * self.S * CL))

    def v_maxLD(self):          # CL* = sqrt(CD0/k)
        return self.v_at_CL(math.sqrt(self.CD0 / self.k))

    def v_minP(self):           # CL = sqrt(3 CD0/k)  (max CL^1.5/CD, best endurance)
        return self.v_at_CL(math.sqrt(3 * self.CD0 / self.k))

    def LD_max(self):
        return 0.5 / math.sqrt(self.CD0 * self.k)

    def v_cruise(self):
        """Cruise = speed at design CL, but never below 1.3 Vs."""
        return max(self.v_at_CL(self.CL_cruise), 1.3 * self.v_stall())


def airframe_from_span(mass, span, AR, **kw):
    S = span ** 2 / AR
    return Airframe(mass=mass, S=S, AR=AR, **kw)


# ============================================================================
# 2. REQUIREMENTS (hardware independent)
# ============================================================================
FIELD_L, FIELD_W = 100.0, 20.0     # m


def turn_for_radius(ac: Airframe, V, R):
    """Bank angle, load factor and stall margin for a level turn of radius R at V."""
    phi = math.atan(V ** 2 / (G * R))
    n = 1 / math.cos(phi)
    return dict(phi_deg=math.degrees(phi), n=n, Vs_turn=ac.v_stall(n), margin=V / ac.v_stall(n))


def max_wing_loading_for_turn(R=8.0, phi_deg=30.0, margin=1.25, CLmax=1.10, rho=RHO):
    """Largest W/S [N/m^2] that lets a level turn of radius R at bank phi be flown at
    `margin` x the turn stall speed. V_turn = sqrt(g R tan(phi))."""
    phi = math.radians(phi_deg)
    V = math.sqrt(G * R * math.tan(phi))
    n = 1 / math.cos(phi)
    Vs1 = V / (margin * math.sqrt(n))
    return dict(V_turn=V, n=n, Vs_1g_max=Vs1, WS_max=0.5 * rho * Vs1 ** 2 * CLmax)


def climb_full(ac: Airframe, T, V):
    """Steady climb angle/rate for thrust T at airspeed V (lift = W cos(gamma), no small-angle approx)."""
    sg = 0.0
    for _ in range(30):
        sg_new = (T - ac.drag(V, math.sqrt(max(1 - sg ** 2, 0.05)))) / ac.W
        sg_new = min(max(sg_new, -0.99), 0.95)
        if abs(sg_new - sg) < 1e-6:
            break
        sg = 0.5 * sg + 0.5 * sg_new
    return dict(gamma_deg=math.degrees(math.asin(sg)), roc=V * sg)


def requirements(ac: Airframe, roc=3.5, h_target=20.0, R_turn=7.0, TW_min=0.70,
                 eta_total=0.40, V0_over_Vc=2.5):
    """Thrust/power needed by the airframe (no specific motor).
    roc        : design climb rate [m/s]
    R_turn     : 180-deg turn radius allowed by the 20 m wide field (7 m -> 14 m dia + 3 m margins)
    TW_min     : beginner hand-launch static T/W floor (see propulsion_sizing.md)
    eta_total  : prop*motor*ESC efficiency used for electrical power estimate
    V0_over_Vc : assumed zero-thrust speed / cruise speed for linear thrust lapse T(V)=T0(1-V/V0)
    """
    Vc = ac.v_cruise()
    Vs = ac.v_stall()
    D_c = ac.drag(Vc)
    P_c = D_c * Vc
    turn = turn_for_radius(ac, Vc, R_turn)
    D_t = ac.drag(Vc, turn["n"])
    # climb at ~1.15 Vc (a little faster than cruise for control margin)
    Vcl = 1.15 * Vc
    gamma = math.asin(min(roc / Vcl, 0.99))
    T_climb = ac.drag(Vcl, math.cos(gamma)) + ac.W * math.sin(gamma)
    P_climb = T_climb * Vcl
    # launch: thrown at ~1.2 Vs, must accelerate at >= 0.3 g while starting a gentle climb
    V_launch = max(1.2 * Vs, 6.0)
    T_launch = ac.drag(V_launch) + ac.mass * 0.3 * G + ac.W * math.sin(math.radians(5))
    V0 = V0_over_Vc * Vc
    lapse = lambda V: max(1 - V / V0, 0.05)
    T0_climb = T_climb / lapse(Vcl)
    T0_launch = T_launch / lapse(V_launch)
    T0_req = max(TW_min * ac.W, T0_climb, T0_launch)
    return dict(
        mass=ac.mass, W=ac.W, S=ac.S, span=ac.span, AR=ac.AR, WS=ac.WS, LDmax=ac.LD_max(),
        V_stall=Vs, V_minP=ac.v_minP(), V_maxLD=ac.v_maxLD(), V_cruise=Vc,
        D_cruise=D_c, P_aero_cruise=P_c, P_elec_cruise_est=P_c / eta_total,
        turn_bank_deg=turn["phi_deg"], turn_n=turn["n"], turn_stall_margin=turn["margin"],
        D_turn=D_t, P_aero_turn=D_t * Vc,
        V_climb=Vcl, roc=roc, T_climb=T_climb, P_aero_climb=P_climb,
        P_elec_climb_est=P_climb / eta_total,
        V_launch=V_launch, T_launch=T_launch,
        T_static_required=T0_req, T_static_required_g=T0_req / G * 1000,
        TW_static_required=T0_req / ac.W,
        pitch_speed_min=2.0 * Vc, pitch_speed_target=2.5 * Vc,
        t_climb_to_target=(h_target - 1.8) / roc,
        x_climb_to_target=(h_target - 1.8) / roc * Vcl * math.cos(gamma),
    )


# ============================================================================
# 3. PROPELLER (UIUC measured data)
# ============================================================================
class Prop:
    """CT(J), CP(J) from UIUC Propeller Data Site files in prop_data/.
    Static point (J=0) = mean of static runs >= 5000 rpm; dynamic curve = all
    runs whose nominal rpm is >= rpm_min (closest to our operating rpm).
    Thrust T = CT rho n^2 D^4, shaft power P = CP rho n^3 D^5, n in rev/s."""

    def __init__(self, tag, label, d_in, pitch_in, folding, mass_g, price_inr,
                 rpm_min=5900, thrust_factor=1.0):
        self.tag, self.label = tag, label
        self.D = d_in * INCH
        self.d_in, self.pitch_in = d_in, pitch_in
        self.folding, self.mass_g, self.price_inr = folding, mass_g, price_inr
        self.thrust_factor = thrust_factor
        st = np.loadtxt(glob.glob(os.path.join(PROP_DIR, f"{tag}_static_*.txt"))[0], skiprows=1)
        hi = st[st[:, 0] >= 5000]
        self.CT0, self.CP0 = hi[:, 1].mean(), hi[:, 2].mean()
        J, CT, CP = [], [], []
        for f in glob.glob(os.path.join(PROP_DIR, f"{tag}_*.txt")):
            b = os.path.basename(f)
            if "static" in b or "geom" in b:
                continue
            rpm_nom = int(b.rsplit("_", 1)[1].split(".")[0])
            if rpm_nom < rpm_min:
                continue
            d = np.loadtxt(f, skiprows=1)
            J += list(d[:, 0]); CT += list(d[:, 1]); CP += list(d[:, 2])
        o = np.argsort(J)
        J, CT, CP = np.array(J)[o], np.array(CT)[o], np.array(CP)[o]
        # bin-average overlapping runs on a J grid, prepend static point
        grid = np.arange(0.10, J.max() + 1e-9, 0.025)
        gj, gt, gp = [0.0], [self.CT0], [self.CP0]
        for a, b in zip(grid[:-1], grid[1:]):
            m = (J >= a) & (J < b)
            if m.any():
                gj.append(J[m].mean()); gt.append(CT[m].mean()); gp.append(CP[m].mean())
        self.Jd = np.array(gj)
        self._ct = PchipInterpolator(self.Jd, np.array(gt), extrapolate=True)
        self._cp = PchipInterpolator(self.Jd, np.array(gp), extrapolate=True)
        self.Jmax = self.Jd[-1]
        # zero-thrust advance ratio (effective aerodynamic pitch / D)
        try:
            self.J_T0 = brentq(lambda j: float(self._ct(j)), 0.3, self.Jmax)
        except ValueError:
            self.J_T0 = self.Jmax

    def CT(self, J):
        J = np.clip(J, 0, self.Jmax)
        return float(self._ct(J)) * self.thrust_factor

    def CP(self, J):
        J = np.clip(J, 0, self.Jmax)
        return max(float(self._cp(J)), 1e-4)


# ============================================================================
# 4. MOTOR / ESC / BATTERY
# ============================================================================
@dataclass
class Motor:
    name: str
    kv: float            # rpm/V
    Rm: float            # ohm (phase-to-phase winding resistance)
    I0: float            # A no-load current (at ~10 V)
    mass_g: float        # incl. leads; mount/prop adapter noted separately
    I_max: float         # A, short-duration max (manufacturer)
    price_inr: float
    shaft_mm: float = 3.17
    source: str = ""


@dataclass
class Battery:
    cells: int
    cap_mAh: float
    mass_g: float
    C: float
    R_cell: float = 0.015    # ohm per cell (assumed, typical small 25-30C pack)
    V_cell: float = 3.8      # V, mid-discharge open-circuit (conservative average)
    price_inr: float = 0
    source: str = ""

    @property
    def R(self):
        return self.cells * self.R_cell

    @property
    def Voc(self):
        return self.cells * self.V_cell

    @property
    def I_max(self):
        return self.cap_mAh / 1000 * self.C

    @property
    def energy_Wh(self):
        return self.cap_mAh / 1000 * self.cells * 3.7


@dataclass
class ESC:
    name: str
    I_cont: float
    mass_g: float
    R: float = R_ESC_EFF      # ohm effective (FETs + wiring + losses), calibrated
    duty_max: float = 1.00
    price_inr: float = 0
    source: str = ""


@dataclass
class Combo:
    motor: Motor
    prop: Prop
    batt: Battery
    esc: ESC
    I_avionics: float = 0.5     # A battery-side (FC+GPS+RX+4 servos through a linear BEC)

    @property
    def mass_g(self):
        return self.motor.mass_g + self.prop.mass_g + self.batt.mass_g + self.esc.mass_g

    @property
    def label(self):
        return f"{self.motor.name} + {self.prop.label} + {self.batt.cells}S {self.batt.cap_mAh:.0f}mAh"


def operating_point(c: Combo, V, throttle, rho=RHO, kv_eff=None):
    """Solve motor torque = prop torque at airspeed V and throttle (0..1).
    Returns dict(rpm, T [N], I_motor, I_batt, P_elec, P_shaft, P_thrust, effs)."""
    m, p, b, e = c.motor, c.prop, c.batt, c.esc
    d = throttle * e.duty_max
    kv_rad = m.kv * (KV_EFF if kv_eff is None else kv_eff) * 2 * math.pi / 60.0   # rad/s per V
    Kt = 1.0 / kv_rad                            # N m / A
    Rtot = m.Rm + e.R + d * d * b.R

    def I_of(w):
        return (d * b.Voc - w / kv_rad) / Rtot

    def f(w):
        n = w / (2 * math.pi)
        J = V / (n * p.D) if n > 1 else 10
        Qp = p.CP(J) * rho * n ** 2 * p.D ** 5 / (2 * math.pi)
        return Kt * (I_of(w) - m.I0) - Qp

    w_hi = d * b.Voc * kv_rad * 0.999
    if d <= 0.02 or f(w_hi) > 0 or f(1.0) < 0:
        return dict(rpm=0, T=0, I_motor=0, I_batt=c.I_avionics, P_elec=b.Voc * c.I_avionics,
                    P_shaft=0, P_thrust=0, eta_motor=0, eta_prop=0, J=0)
    w = brentq(f, 1.0, w_hi)
    n = w / (2 * math.pi)
    J = V / (n * p.D)
    I = I_of(w)
    T = p.CT(J) * rho * n ** 2 * p.D ** 4
    P_shaft = p.CP(J) * rho * n ** 3 * p.D ** 5
    I_b = d * I + c.I_avionics
    P_el = b.Voc * I_b - I_b ** 2 * b.R
    return dict(rpm=w * 60 / (2 * math.pi), T=T, I_motor=I, I_batt=I_b, P_elec=P_el,
                P_shaft=P_shaft, P_thrust=T * V,
                eta_motor=P_shaft / max(d * (b.Voc - d * I * b.R) * I, 1e-9),
                eta_prop=(T * V / P_shaft) if P_shaft > 0 else 0, J=J)


def throttle_for_thrust(c: Combo, V, T_req, rho=RHO):
    """Throttle (0..1) giving thrust T_req at V; None if impossible."""
    g = lambda t: operating_point(c, V, t, rho)["T"] - T_req
    if g(1.0) < 0:
        return None
    lo = 0.05
    while g(lo) > 0 and lo > 0.01:
        lo /= 2
    return brentq(g, lo, 1.0, xtol=1e-4)


# ============================================================================
# 5. MISSION / ENDURANCE in the 100 m x 20 m field
# ============================================================================
def racetrack_geometry(R=7.0, end_margin=5.0):
    straight = FIELD_L - 2 * R - 2 * end_margin
    lap = 2 * straight + 2 * math.pi * R
    return dict(R=R, straight=straight, lap=lap, turn_frac=2 * math.pi * R / lap)


def evaluate(ac: Airframe, c: Combo, h_target=20.0, R_turn=7.0, t_level_min=8.0,
             n_extra_climbs=2, rho=RHO, usable=0.80):
    """Full performance of a combo on an airframe (airframe mass must already include it)."""
    W = ac.W
    st = operating_point(c, 0.0, 1.0, rho)
    pitch_speed = st["rpm"] / 60 * c.prop.pitch_in * INCH
    zero_T_speed = st["rpm"] / 60 * c.prop.D * c.prop.J_T0      # aerodynamic pitch speed (static rpm)
    Vc = ac.v_cruise()
    # level cruise
    tc = throttle_for_thrust(c, Vc, ac.drag(Vc), rho)
    cr = operating_point(c, Vc, tc, rho) if tc else None
    # turn at R_turn
    trn = turn_for_radius(ac, Vc, R_turn)
    tt = throttle_for_thrust(c, Vc, ac.drag(Vc, trn["n"]), rho)
    tu = operating_point(c, Vc, tt, rho) if tt else None
    # full-throttle climb: best ROC over V in [1.2Vs, 2Vc]
    best = None
    for V in np.linspace(1.2 * ac.v_stall(), 2.0 * Vc, 40):
        op = operating_point(c, V, 1.0, rho)
        cl = climb_full(ac, op["T"], V)
        if best is None or cl["roc"] > best[1]:
            best = (V, cl["roc"], op, cl["gamma_deg"])
    Vcl, roc_max, opcl, gam_max = best
    # practical beginner climb: 25 deg climb angle at 1.15 Vc -> throttle & current
    V25 = 1.15 * Vc
    T25 = ac.drag(V25, math.cos(math.radians(25))) + W * math.sin(math.radians(25))
    t25 = throttle_for_thrust(c, V25, T25, rho)
    op25 = operating_point(c, V25, t25, rho) if t25 else opcl
    # launch acceleration at 1.2 Vs (hand throw), full throttle
    Vl = max(1.2 * ac.v_stall(), 6.0)
    opl = operating_point(c, Vl, 1.0, rho)
    a_launch = (opl["T"] - ac.drag(Vl)) / ac.mass
    # mission energy (mAh)
    roc25 = V25 * math.sin(math.radians(25))
    t_climb = (h_target - 1.8) / roc25           # mission uses the practical 25-deg climb
    x_climb = t_climb * V25 * math.cos(math.radians(25))
    geo = racetrack_geometry(R_turn)
    I_level = (1 - geo["turn_frac"]) * cr["I_batt"] + geo["turn_frac"] * tu["I_batt"] if (cr and tu) else float("nan")
    t_level = t_level_min * 60
    t_extra = n_extra_climbs * 15.0 / roc25                       # two 15 m re-climbs (go-around)
    t_glide = 3 * 20.0 / 1.2                                       # ~3 glides from 20 m at ~1.2 m/s sink
    mAh = (op25["I_batt"] * (t_climb + t_extra) + I_level * t_level + c.I_avionics * (t_glide + 60)) / 3.6
    cap_needed = mAh / usable
    endurance_min = ((usable * c.batt.cap_mAh * 3.6) - op25["I_batt"] * (t_climb + t_extra)) / I_level / 60
    # conservative beginner throttle profile: 25 % of time climbing (25-deg climb), 60 % level racetrack,
    # 15 % motor-off glide -> average current & endurance on 80 % usable capacity
    I_beg = 0.25 * op25["I_batt"] + 0.60 * I_level + 0.15 * c.I_avionics
    endurance_beginner = usable * c.batt.cap_mAh * 3.6 / I_beg / 60
    return dict(
        combo=c.label, combo_mass_g=c.mass_g,
        T_static_g=st["T"] / G * 1000, TW_static=st["T"] / W, I_static=st["I_batt"], P_static=st["P_elec"],
        rpm_static=st["rpm"], pitch_speed=pitch_speed, zeroT_speed=zero_T_speed,
        V_cruise=Vc, thr_cruise=tc, I_cruise=cr["I_batt"] if cr else None,
        P_cruise=cr["P_elec"] if cr else None, eta_prop_cruise=cr["eta_prop"] if cr else None,
        eta_motor_cruise=cr["eta_motor"] if cr else None,
        turn_bank=trn["phi_deg"], thr_turn=tt, I_turn=tu["I_batt"] if tu else None,
        V_climb=Vcl, roc_max=roc_max, gamma_max=gam_max, I_climb=opcl["I_batt"], T_climb=opcl["T"],
        thr_climb25=t25, I_climb25=op25["I_batt"], roc25=roc25, t_climb_20m=t_climb, x_climb_20m=x_climb,
        V_launch=Vl, T_launch_g=opl["T"] / G * 1000, a_launch=a_launch,
        I_level_avg=I_level, mission_mAh=mAh, cap_needed_mAh=cap_needed,
        endurance_min=endurance_min, I_beginner_avg=I_beg, endurance_beginner_min=endurance_beginner,
        batt_Imax=c.batt.I_max,
        motor_I_ok=st["I_motor"] <= c.motor.I_max, esc_margin=c.esc.I_cont / st["I_batt"],
    )


# ============================================================================
# 6. CATALOGUE (India-purchasable parts; see propulsion_sizing.md for sources)
# ============================================================================
def load_props():
    return {
        "F9x5": Prop("ancf_9x5", "9x5 folding", 9, 5, True, 18, 900),
        "F9x6": Prop("ancf_9x6", "9x6 folding", 9, 6, True, 18, 900),
        "F10x6": Prop("ancf_10x6", "10x6 folding", 10, 6, True, 20, 1000),
        "F10x5": Prop("ancf_10x5", "10x5 folding", 10, 5, True, 20, 1000),
        "F9x4": Prop("ancf_9x4", "9x4 folding", 9, 4, True, 18, 900),
        "E9x6": Prop("apce_9x6", "APC 9x6E (fixed)", 9, 6, False, 14, 450),
        "E8x6": Prop("apce_8x6", "APC 8x6E (fixed)", 8, 6, False, 11, 400),
    }


MOTORS = {
    "A2212_1000": Motor("A2212/13T 1000KV", 1000, 0.090, 0.5, 50, 13, 450, 3.17,
                        "generic A2212/13T spec: 0.5 A @10 V, 90 mOhm, 12-13 A/60 s, ~47-53 g"),
    "X2212_980": Motor("SunnySky X2212-III 980KV", 980, 0.092, 0.6, 57, 26, 2650, 3.17,
                       "readymaderc.com SunnySky X2212 980KV V3: 0.6 A/10 V, 92 mOhm, 26 A(30 s), 57 g"),
    "A2212_1400": Motor("A2212/10T 1400KV", 1400, 0.065, 0.7, 47, 16, 450, 3.17,
                        "A2212 1400KV datasheet: 0.7 A, 65 mOhm, 16 A/60 s, 47 g"),
    "CF2822_1200": Motor("EMAX CF2822 1200KV", 1200, 0.150, 0.9, 39, 16, 900, 3.0,
                         "EMAX CF2822: 0.9 A @10 V, 150 mOhm, 16 A/60 s, 39 g"),
    "AT2312_1150": Motor("T-Motor AT2312 1150KV", 1150, 0.075, 0.85, 60, 25, 3500, 4.0,
                         "T-Motor AT2312 long shaft page: 75 mOhm, 0.85 A @10 V, 25 A peak, 60 g"),
}

BATTS = {
    "3S850": Battery(3, 850, 72, 30, price_inr=900, source="Orange 3S 850 30C (Robu p=54622); mass ~70-75 g assumed"),
    "3S1000": Battery(3, 1000, 85, 30, price_inr=1000, source="Orange 3S 1000 30C, 85 g 72x34x19 mm (Robocraze), Rs 966 (Robu)"),
    "3S1300": Battery(3, 1300, 110, 30, price_inr=1200, source="Orange 3S 1300 30C, ~107 g (Flipkart), 120 g ship wt (Robu)"),
    "3S1500": Battery(3, 1500, 115, 30, price_inr=1300, source="Orange 3S 1500 30C, 115 g, Rs 1299 (Robu)"),
    "2S1300": Battery(2, 1300, 75, 30, price_inr=900, source="2S 1300 30C ~75 g (assumed typical)"),
}

ESCS = {
    "SKY20": ESC("Hobbywing Skywalker 20A (2-3S, 5V/2A linear BEC)", 20, 19, price_inr=1100,
                 source="Skywalker-20A manual: 20 A cont/25 A burst, 19 g, 5V/2A linear BEC, programmable brake"),
    "SKY30": ESC("Hobbywing Skywalker 30A (2-3S, 5V/3A BEC)", 30, 32, price_inr=1500,
                 source="Hobbywing Skywalker 30A (typical 30-32 g)"),
}


def build_combos(props):
    M, B, E = MOTORS, BATTS, ESCS
    return {
        "P1_A2212-1000_F9x5_3S1000": Combo(M["A2212_1000"], props["F9x5"], B["3S1000"], E["SKY20"]),
        "P1b_A2212-1000_F9x5_3S850": Combo(M["A2212_1000"], props["F9x5"], B["3S850"], E["SKY20"]),
        "P2_A2212-1000_F10x6_3S1000": Combo(M["A2212_1000"], props["F10x6"], B["3S1000"], E["SKY20"]),
        "P3_A2212-1000_F9x4_3S1000": Combo(M["A2212_1000"], props["F9x4"], B["3S1000"], E["SKY20"]),
        "P5_A2212-1000_E9x6fixed_3S1000": Combo(M["A2212_1000"], props["E9x6"], B["3S1000"], E["SKY20"]),
        "P4_A2212-1000_F10x6_2S1300": Combo(M["A2212_1000"], props["F10x6"], B["2S1300"], E["SKY20"]),
        "A1_X2212-980_F9x5_3S1000": Combo(M["X2212_980"], props["F9x5"], B["3S1000"], E["SKY20"]),
        "A2_X2212-980_F10x6_3S1300": Combo(M["X2212_980"], props["F10x6"], B["3S1300"], E["SKY20"]),
        "B1_A2212-1400_F9x5_2S1300": Combo(M["A2212_1400"], props["F9x5"], B["2S1300"], E["SKY20"]),
        "B2_A2212-1400_F10x6_2S1300": Combo(M["A2212_1400"], props["F10x6"], B["2S1300"], E["SKY30"]),
        "B3_A2212-1400_F9x5_3S1000": Combo(M["A2212_1400"], props["F9x5"], B["3S1000"], E["SKY30"]),
        "L1_CF2822-1200_F9x5_3S1000": Combo(M["CF2822_1200"], props["F9x5"], B["3S1000"], E["SKY20"]),
        "L2_CF2822-1200_F9x5_2S1300": Combo(M["CF2822_1200"], props["F9x5"], B["2S1300"], E["SKY20"]),
        "L3_CF2822-1200_F10x6_2S1300": Combo(M["CF2822_1200"], props["F10x6"], B["2S1300"], E["SKY20"]),
    }


def suggest_combo(ac: Airframe, props=None, TW_lo=0.75, TW_hi=2.0, roc_min=3.0, endurance_min=10.0,
                  thrust_unc=0.15, **kw):
    """Rank catalogue combos for this airframe.
    Feasible = (1-thrust_unc)*T/W >= TW_lo (beginner launch floor even if the model is 15 % optimistic),
    T/W <= TW_hi, ROC >= roc_min, endurance (80 % usable) >= endurance_min,
    ESC continuous rating >= 1.3 x static current, static motor current <= motor max,
    battery real C (=50 % of label) >= static current, pitch speed >= 2 Vcruise.
    Ranking: feasible first, then |T/W - 1.2| (sweet spot for a beginner hand launch), then mass."""
    props = props or load_props()
    combos = build_combos(props)
    out = []
    for key, c in combos.items():
        r = evaluate(ac, c, **kw)
        r["key"] = key
        r["batt_ok"] = 0.5 * c.batt.I_max >= r["I_static"]
        r["feasible"] = bool((1 - thrust_unc) * r["TW_static"] >= TW_lo and r["TW_static"] <= TW_hi
                             and r["roc_max"] >= roc_min and r["endurance_min"] >= endurance_min
                             and r["esc_margin"] >= 1.3 and r["motor_I_ok"] and r["batt_ok"]
                             and r["pitch_speed"] >= 2.0 * r["V_cruise"])
        out.append(r)
    out.sort(key=lambda r: (not r["feasible"], round(abs(r["TW_static"] - 1.2), 1), r["combo_mass_g"]))
    return out


# ============================================================================
# 7. VALIDATION of the motor+prop model against manufacturer bench data
# ============================================================================
# T-Motor AT2312 long-shaft KV1150, store.tmotor.com/goods.php?id=788 (APC props, bench supply)
# rows: throttle, V, I, rpm, torque [N m], thrust [g]
BENCH = [
    ("T-Motor AT2312 KV1150", "AT2312_1150", "E9x6", [(0.5, 11.42, 5.32, 5867, 0.068, 418),
                                                       (0.7, 11.32, 9.55, 7258, 0.103, 677),
                                                       (1.0, 11.12, 18.03, 8960, 0.160, 1029)]),
    ("T-Motor AT2312 KV1150", "AT2312_1150", "SF10x4.7", [(0.5, 11.27, 6.17, 5147, 0.085, 548),
                                                           (1.0, 10.78, 22.63, 7656, 0.199, 1244)]),
    # SunnySky X2212 KV980 with 1047 prop at 11.1 V (flymod.net / buddyrc listing); UIUC APC 10x4.7SF used as proxy
    ("SunnySky X2212 KV980", "X2212_980", "SF10x4.7", [(1.0, 11.1, 13.2, 7100, None, 870)]),
]


def validate(props, rho_test=1.20, verbose=True):
    """(a) prop-only: thrust & torque from UIUC CT0/CP0 at the MEASURED rpm (ratio meas/UIUC);
       (b) full model at 100 % throttle and bench voltage (calibrated KV_EFF, R_ESC_EFF)."""
    props = dict(props)
    props.setdefault("SF10x4.7", Prop("apcsf_10x4.7", "APC 10x4.7SF", 10, 4.7, False, 15, 0))
    rows = []
    for mname, mk, pk, data in BENCH:
        p, m = props[pk], MOTORS[mk]
        for thr, V, I, rpm, Q, Tg in data:
            n = rpm / 60
            T_u = p.CT0 * rho_test * n ** 2 * p.D ** 4 / G * 1000
            Q_u = p.CP0 * rho_test * n ** 2 * p.D ** 5 / (2 * math.pi)
            row = dict(case=f"{mname} / {p.label}", throttle=thr, rpm_meas=rpm, T_meas_g=Tg, I_meas=I,
                       T_ratio=Tg / T_u, Q_ratio=(Q / Q_u) if Q else None)
            if thr == 1.0:
                c = Combo(m, p, Battery(1, 1000, 0, 100, R_cell=0.0, V_cell=V), ESC("bench", 60, 0),
                          I_avionics=0.0)
                op = operating_point(c, 0.0, 1.0, rho_test)
                row.update(rpm_model=op["rpm"], I_model=op["I_motor"], T_model_g=op["T"] / G * 1000)
            rows.append(row)
    if verbose:
        print(f"\nVALIDATION (rho_test={rho_test:.2f}; KV_EFF={KV_EFF}, R_ESC_EFF={R_ESC_EFF} ohm)")
        for r in rows:
            s = (f"  {r['case']:<40} thr {r['throttle']:.1f} rpm {r['rpm_meas']:>5} T {r['T_meas_g']:>5.0f} g "
                 f"meas/UIUC: T x{r['T_ratio']:.2f}" + (f" Q x{r['Q_ratio']:.2f}" if r['Q_ratio'] else ""))
            if "rpm_model" in r:
                s += (f" | model@100%: {r['rpm_model']:.0f} rpm, {r['I_model']:.1f} A, {r['T_model_g']:.0f} g "
                      f"(meas {r['rpm_meas']}, {r['I_meas']:.1f} A, {r['T_meas_g']:.0f} g)")
            print(s)
    return rows


# ============================================================================
# 8. PLOTS
# ============================================================================
PAL = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#6250d6"]


def _style(ax, title, xl, yl):
    ax.set_title(title, loc="left", fontsize=11, color="#0b0b0b")
    ax.set_xlabel(xl, color="#52514e"); ax.set_ylabel(yl, color="#52514e")
    ax.grid(True, color="#e6e5e0", lw=0.6); ax.set_axisbelow(True)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color("#b7b6b0")
    ax.tick_params(colors="#52514e")


def make_plots(designs, combo, ac_main, outdir=HERE):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    # (1) power required vs V for representative designs
    fig, ax = plt.subplots(figsize=(7.5, 4.6))
    for i, (name, ac) in enumerate(designs.items()):
        V = np.linspace(ac.v_stall(), 16, 120)
        P = [ac.p_req(v) for v in V]
        ax.plot(V, P, lw=2, color=PAL[i], label=name)
        ax.plot(ac.v_cruise(), ac.p_req(ac.v_cruise()), "o", ms=8, color=PAL[i], mec="white", mew=2)
    _style(ax, "Aerodynamic power required P = D·V (curves start at stall; dots = cruise, CL 0.6)",
           "Airspeed V [m/s]", "P_req [W]")
    ax.legend(frameon=False, fontsize=8, loc="upper left")
    ax.set_xlim(left=4); ax.set_ylim(bottom=0)
    fig.tight_layout(); fig.savefig(os.path.join(outdir, "power_required.png"), dpi=150); plt.close(fig)

    # (2) thrust available (full & part throttle) vs thrust required, main design
    fig, ax = plt.subplots(figsize=(7.5, 4.6))
    V = np.linspace(0, 20, 60)
    for i, thr in enumerate([1.0, 0.6, 0.4]):
        T = [operating_point(combo, v, thr)["T"] for v in V]
        ax.plot(V, T, lw=2, color=PAL[i], label=f"thrust available, {int(thr*100)}% throttle")
    Vd = np.linspace(ac_main.v_stall(), 20, 60)
    ax.plot(Vd, [ac_main.drag(v) for v in Vd], lw=2, color="#52514e", ls="--", label="drag (level, n=1)")
    tr = turn_for_radius(ac_main, ac_main.v_cruise(), 7.0)
    ax.plot(Vd, [ac_main.drag(v, tr["n"]) for v in Vd], lw=1.5, color="#8a8984", ls=":",
            label=f"drag in turn (n={tr['n']:.2f})")
    ax.axvline(ac_main.v_cruise(), color="#b7b6b0", lw=1)
    ax.text(ac_main.v_cruise() + 0.2, ax.get_ylim()[1] * 0.92 if ax.get_ylim()[1] else 4, "cruise", fontsize=8,
            color="#52514e")
    _style(ax, f"Thrust available vs required — {combo.label}", "Airspeed V [m/s]", "Force [N]")
    ax.legend(frameon=False, fontsize=8); ax.set_ylim(bottom=0)
    fig.tight_layout(); fig.savefig(os.path.join(outdir, "thrust_vs_drag.png"), dpi=150); plt.close(fig)

    # (3) rate of climb vs V
    fig, ax = plt.subplots(figsize=(7.5, 4.2))
    V = np.linspace(ac_main.v_stall() * 1.05, 18, 60)
    for i, thr in enumerate([1.0, 0.7, 0.5]):
        roc = [climb_full(ac_main, operating_point(combo, v, thr)["T"], v)["roc"] for v in V]
        ax.plot(V, roc, lw=2, color=PAL[i], label=f"{int(thr*100)}% throttle")
    ax.axhline(3.0, color=PAL[1], lw=1.2, ls="--", label="3 m/s requirement")
    _style(ax, f"Rate of climb — {ac_main.mass:.2f} kg, {combo.label}", "Airspeed V [m/s]", "ROC [m/s]")
    ax.legend(frameon=False, fontsize=8)
    fig.tight_layout(); fig.savefig(os.path.join(outdir, "rate_of_climb.png"), dpi=150); plt.close(fig)


# ============================================================================
# 9. MAIN
# ============================================================================
def representative_designs():
    return {
        "D1 light 0.40 kg, b1.3 AR10": airframe_from_span(0.40, 1.3, 10),
        "D2 base  0.50 kg, b1.5 AR10": airframe_from_span(0.50, 1.5, 10),
        "D3 0.55 kg, b1.6 AR11": airframe_from_span(0.55, 1.6, 11),
        "D4 heavy 0.65 kg, b1.6 AR10, CD0 .03 e .75": airframe_from_span(0.65, 1.6, 10, CD0=0.030, e=0.75),
    }


def _fmt(x, f="{:.2f}"):
    return "-" if x is None else f.format(x)


def detail_table(c: Combo, ac: Airframe, rho=RHO):
    """Throttle sweep: static and at cruise speed."""
    rows = []
    for thr in (0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0):
        o0 = operating_point(c, 0.0, thr, rho)
        oc = operating_point(c, ac.v_cruise(), thr, rho)
        rows.append(dict(throttle=thr, rpm0=o0["rpm"], T0_g=o0["T"] / G * 1000, I0=o0["I_batt"], P0=o0["P_elec"],
                         gW=(o0["T"] / G * 1000) / max(o0["P_elec"], 1e-6),
                         Tc_g=oc["T"] / G * 1000, Ic=oc["I_batt"], eta_p=oc["eta_prop"], eta_m=oc["eta_motor"]))
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mass", type=float, default=0.50)
    ap.add_argument("--span", type=float, default=1.50)
    ap.add_argument("--AR", type=float, default=10.0)
    ap.add_argument("--CD0", type=float, default=0.025)
    ap.add_argument("--e", type=float, default=0.85)
    ap.add_argument("--CLmax", type=float, default=1.10)
    ap.add_argument("--CLcruise", type=float, default=0.60)
    ap.add_argument("--rho", type=float, default=RHO)
    ap.add_argument("--noplots", action="store_true")
    a = ap.parse_args()

    props = load_props()
    print("Propeller data (UIUC): static CT0/CP0 and zero-thrust J")
    for k, p in props.items():
        print(f"  {k:<6} {p.label:<17} CT0={p.CT0:.3f} CP0={p.CP0:.4f} J(T=0)={p.J_T0:.2f}")
    val = validate(props)

    print("\nREQUIREMENTS (hardware independent), rho=%.2f" % RHO)
    designs = representative_designs()
    req_rows = {}
    for name, ac in designs.items():
        r = requirements(ac); req_rows[name] = r
        print(f"  {name:<44} S={r['S']:.3f} W/S={r['WS']:.1f} L/Dmax={r['LDmax']:.1f} Vs={r['V_stall']:.1f} "
              f"VminP={r['V_minP']:.1f} Vc={r['V_cruise']:.1f} | D_c={r['D_cruise']:.2f} N P_aero={r['P_aero_cruise']:.1f} W "
              f"| turn R7m @Vc: {r['turn_bank_deg']:.0f}deg n={r['turn_n']:.2f} | T_climb(3.5 m/s)={r['T_climb']:.2f} N "
              f"P_aero_climb={r['P_aero_climb']:.1f} W | T0_req={r['T_static_required_g']:.0f} g (T/W {r['TW_static_required']:.2f})"
              f" | pitch speed >= {r['pitch_speed_min']:.0f}-{r['pitch_speed_target']:.0f} m/s")

    print("\nTURN-IN-FIELD guidance (max wing loading so a level 180-deg turn fits, flown at 1.25 x turn stall speed, CLmax 1.1)")
    for R in (7.0, 8.0):
        for phi in (30.0, 35.0, 40.0):
            t = max_wing_loading_for_turn(R, phi)
            print(f"  R={R:.0f} m bank {phi:.0f} deg: V_turn={t['V_turn']:.1f} m/s, n={t['n']:.2f}, "
                  f"Vs(1g)<={t['Vs_1g_max']:.1f} m/s -> W/S <= {t['WS_max']:.1f} N/m^2")

    ac = Airframe(mass=a.mass, S=a.span ** 2 / a.AR, AR=a.AR, CD0=a.CD0, e=a.e, CLmax=a.CLmax,
                  CL_cruise=a.CLcruise, rho=a.rho)
    print(f"\nCOMBO EVALUATION on airframe: m={ac.mass} kg, b={ac.span:.2f} m, S={ac.S:.3f} m2, AR={ac.AR}, "
          f"W/S={ac.WS:.1f}, CD0={ac.CD0}, e={ac.e}, Vc={ac.v_cruise():.1f} m/s, rho={ac.rho}")
    res = suggest_combo(ac, props)
    hdr = ("mass_g", "T0_g", "T/W", "I0_A", "rpm0", "Vpitch", "thr_c", "I_c", "P_c", "ROCmax", "thr25", "I_25",
           "x20m", "a_lnch", "Ilvl", "end_min", "I_beg", "endBeg", "feas")
    print(f"  {'key':>28} " + " ".join(f"{h:>7}" for h in hdr))
    for r in res:
        vals = (r["combo_mass_g"], r["T_static_g"], r["TW_static"], r["I_static"], r["rpm_static"],
                r["pitch_speed"], r["thr_cruise"], r["I_cruise"], r["P_cruise"], r["roc_max"], r["thr_climb25"],
                r["I_climb25"], r["x_climb_20m"], r["a_launch"], r["I_level_avg"], r["endurance_min"],
                r["I_beginner_avg"], r["endurance_beginner_min"], r["feasible"])
        print(f"  {r['key'][:28]:>28} " + " ".join(
            (f"{v:>7.2f}" if abs(v) < 100 else f"{v:>7.0f}") if isinstance(v, float) else f"{str(v):>7}" for v in vals))
    best = res[0]
    print(f"\nSUGGESTED (lightest feasible near T/W 1.2): {best['combo']}")

    combos = build_combos(props)
    main_c = combos["P1_A2212-1000_F9x5_3S1000"]
    print(f"\nDETAIL: {main_c.label}  (battery {main_c.batt.Voc:.1f} V mid-discharge)")
    print("  thr   rpm0   T0[g]  Ibat0[A] Pel0[W]  g/W | T@Vc[g] I@Vc[A] eta_prop eta_motor")
    det = detail_table(main_c, ac)
    for d in det:
        print(f"  {d['throttle']:.1f} {d['rpm0']:>6.0f} {d['T0_g']:>7.0f} {d['I0']:>8.2f} {d['P0']:>7.1f} {d['gW']:>5.2f} |"
              f" {d['Tc_g']:>7.0f} {d['Ic']:>7.2f} {d['eta_p']:>8.2f} {d['eta_m']:>9.2f}")

    print("\nSENSITIVITY of recommended combo (P1):")
    sens = {}
    cases = {
        "nominal": (ac, RHO, 3.8),
        "hot day rho=1.06": (ac, 1.06, 3.8),
        "end of pack 3.6 V/cell": (ac, RHO, 3.6),
        "full pack 4.1 V/cell": (ac, RHO, 4.1),
        "draggy build CD0 .035 e .75": (Airframe(mass=a.mass, S=ac.S, AR=ac.AR, CD0=0.035, e=0.75), RHO, 3.8),
        "L/D~10 build CD0 .05 e .75": (Airframe(mass=a.mass, S=ac.S, AR=ac.AR, CD0=0.050, e=0.75), RHO, 3.8),
        "low W/S 0.45 kg b1.7 AR10": (airframe_from_span(0.45, 1.7, 10), RHO, 3.8),
        "heavy 0.65 kg": (Airframe(mass=0.65, S=ac.S, AR=ac.AR, CD0=ac.CD0, e=ac.e), RHO, 3.8),
    }
    for name, (acx, rho, vcell) in cases.items():
        c = Combo(main_c.motor, main_c.prop, Battery(3, 1000, 85, 30, V_cell=vcell), main_c.esc)
        r = evaluate(acx, c, rho=rho)
        sens[name] = r
        print(f"  {name:<30} T0={r['T_static_g']:.0f} g T/W={r['TW_static']:.2f} I0={r['I_static']:.1f} A "
              f"Vc={r['V_cruise']:.1f} I_cruise={r['I_cruise']:.2f} A I_level(avg incl turns)={r['I_level_avg']:.2f} A "
              f"ROCmax={r['roc_max']:.1f} m/s endurance(80%): physics {r['endurance_min']:.0f} min, "
              f"beginner profile {r['endurance_beginner_min']:.0f} min")

    out = dict(airframe=asdict(ac), calibration=dict(KV_EFF=KV_EFF, R_ESC_EFF=R_ESC_EFF), validation=val,
               requirements=req_rows, combos=res, detail_P1=det, sensitivity_P1=sens)
    with open(os.path.join(HERE, "propulsion_results.json"), "w") as f:
        json.dump(out, f, indent=1, default=str)
    print("\nresults written: propulsion_results.json")
    if not a.noplots:
        make_plots(designs, main_c, ac)
        print("plots written: power_required.png, thrust_vs_drag.png, rate_of_climb.png")


if __name__ == "__main__":
    main()
