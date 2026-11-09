"""
common.py -- shared geometry, mass/CG, aero helpers for CONCEPT B ("High-AR efficiency").

Everything (sizing, aero, dynamics, performance) imports this file so that the
geometry the report quotes is the geometry that was analysed.

Units: SI, angles in degrees unless a name ends in _rad.
Axes: x aft from the wing ROOT LEADING EDGE, y right (starboard), z up.
      (XFLR5 uses the same x/z convention with the wing root LE at the origin.)

Stage-1 code is IMPORTED read-only (sys.dont_write_bytecode so nothing is written
into the stage-1 folders):
    02_construction/mass_model.py   -> structural masses (shell, spar, pod, boom, tails, glue)
    04_propulsion/propulsion_model.py -> motor/prop/battery operating points (UIUC prop data)
"""
from __future__ import annotations

import sys
sys.dont_write_bytecode = True

import json
import math
import os
from copy import deepcopy

import numpy as np
import aerosandbox as asb
import aerosandbox.numpy as anp  # noqa: F401  (kept for users of this module)

HERE = os.path.dirname(os.path.abspath(__file__))
DESIGN_DIR = os.path.abspath(os.path.join(HERE, "..", ".."))
STAGE1_DAT = os.path.join(DESIGN_DIR, "03_airfoil", "dat")
RES = os.path.join(HERE, "results")
PLOTS = os.path.join(HERE, "plots")
os.makedirs(RES, exist_ok=True)
os.makedirs(PLOTS, exist_ok=True)

sys.path.insert(0, os.path.join(DESIGN_DIR, "02_construction"))
sys.path.insert(0, os.path.join(DESIGN_DIR, "04_propulsion"))
import mass_model as MM          # noqa: E402  (stage-1, read only)
import propulsion_model as PM    # noqa: E402  (stage-1, read only)

# ----------------------------------------------------------------------------- air
RHO = 1.09          # kg/m^3 Hyderabad ~540 m, 30 C (project constant)
RHO_HOT = 1.06
NU = 1.7e-5         # m^2/s
G = 9.81


class SiteAtm(asb.Atmosphere):
    """Atmosphere with the project's rho and nu (ISA would give a different nu)."""

    def __init__(self, rho=RHO):
        super().__init__(altitude=0.0)
        self._rho = rho

    def density(self):
        return self._rho

    def dynamic_viscosity(self):
        return self._rho * NU


def op(V, alpha=0.0, beta=0.0, p=0.0, q=0.0, r=0.0, rho=RHO):
    return asb.OperatingPoint(atmosphere=SiteAtm(rho), velocity=V, alpha=alpha, beta=beta, p=p, q=q, r=r)


def Re(V, c):
    return V * c / NU


# ----------------------------------------------------------------------------- plotting style
C1, C2, C3, C4, C5, C6 = "#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#6250d6"
INK, INK2, GRID = "#222222", "#5f5e58", "#d9d8d2"


def style():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({
        "figure.dpi": 130, "axes.edgecolor": INK2, "axes.labelcolor": INK,
        "xtick.color": INK2, "ytick.color": INK2, "axes.grid": True,
        "grid.color": GRID, "grid.linewidth": 0.6, "axes.spines.top": False,
        "axes.spines.right": False, "font.size": 9, "lines.linewidth": 1.8,
        "legend.frameon": False,
    })
    return plt


# ----------------------------------------------------------------------------- airfoils
TE_MM = 0.8   # printed vase-mode trailing edge thickness (two merged 0.4 mm walls), construction_recommendation 2.1(6)


def load_foil(name: str, chord: float | None = None, te_mm: float = TE_MM) -> asb.Airfoil:
    """Wing airfoil from the stage-1 .dat (sharp) or the AeroSandbox UIUC database,
    re-panelled, with a blunt TE of te_mm at this chord (printed wing)."""
    p = os.path.join(STAGE1_DAT, f"{name}.dat")
    af = asb.Airfoil(name=name, coordinates=p) if os.path.exists(p) else asb.Airfoil(name)
    af = af.repanel(n_points_per_side=100)
    if chord is not None and te_mm > 0:
        af = af.set_TE_thickness(te_mm * 1e-3 / chord)
        af.name = f"{name}_te{te_mm*1e-3/chord*100:.2f}pct"
    return af


def plate_foil(chord: float, t_mm: float = 3.0) -> asb.Airfoil:
    """3 mm Depron flat plate with rounded LE and sanded TE: stage-1 3 % plate scaled to t/c = t/chord."""
    base = asb.Airfoil(name="fp", coordinates=os.path.join(STAGE1_DAT, "flatplate-3pct.dat"))
    xy = base.coordinates.copy()
    xy[:, 1] *= (t_mm * 1e-3 / chord) / 0.03
    return asb.Airfoil(name=f"plate_{t_mm:.0f}mm_c{chord*1000:.0f}", coordinates=xy).repanel(n_points_per_side=80)


# ----------------------------------------------------------------------------- baseline geometry
# Wing: 3 panels per side whose breaks coincide with the printed-segment joints (3 x 233 mm per half).
# Quarter-chord line straight & unswept (straight spar at ~25 % c).  Panel 3 = polyhedral tip.
BASE = dict(
    name="B_highAR",
    b=1.40,
    y=[0.0, 0.2333, 0.4667, 0.700],          # panel breaks (projected span), m
    c=[0.135, 0.128, 0.108, 0.072],          # chords, m
    twist=[0.0, 0.0, -1.0, -2.5],            # deg, washout (relative to root)
    dihedral=[0.0, 0.0, 8.0],                # deg per panel (inner, mid, tip)
    i_w=2.0,                                 # wing root incidence vs fuselage x axis, deg
    foil="sd7037",
    # ailerons on mid panel (35-66 % semispan): spanwise from/to (y) and chord fraction
    ail_y=(0.245, 0.4667), ail_cf=0.27,
    # horizontal tail (Depron plate, boom-mounted, cruciform-low)
    lt=0.68,                                 # wing root c/4 -> htail root c/4, m
    Sh=0.0180, ARh=4.2, taper_h=0.75, zh=0.015, elev_cf=0.40,
    i_t=-1.0,                                # tail incidence (nominal, trimmed later), deg
    # vertical tail
    Sv=0.0110, ARv=1.5, taper_v=0.65, rud_cf=0.40,
    # fuselage: pod + boom
    pod_d=0.046, pod_len=0.40, pod_zc=-0.026, boom_od=0.008, boom_x0=0.12,
    x_nose=-0.25,                            # firewall/nose x (solved by the CG loop)
    # ballast (g) if the CG loop cannot reach the target with the battery bay
    ballast=0.0,
)


def panel_le(d, i):
    """x of LE at xsec i so that the c/4 line is straight and unswept."""
    return 0.25 * (d["c"][0] - d["c"][i])


def wing_geom_numbers(d):
    y, c = np.array(d["y"]), np.array(d["c"])
    S_half = np.sum((c[1:] + c[:-1]) / 2 * np.diff(y))
    S = 2 * S_half
    # MAC (exact for piecewise linear chord)
    mac_int, ymac_int = 0.0, 0.0
    for i in range(len(y) - 1):
        yy = np.linspace(y[i], y[i + 1], 201)
        cc = np.interp(yy, y, c)
        mac_int += np.trapezoid(cc ** 2, yy)
        ymac_int += np.trapezoid(cc * yy, yy)
    mac = 2 * mac_int / S
    y_mac = 2 * ymac_int / S                    # spanwise centroid of area (= y of MAC for piecewise)
    x_le_mac = 0.25 * (c[0] - mac)              # straight unswept c/4 line
    return dict(S=S, b=2 * y[-1], AR=(2 * y[-1]) ** 2 / S, mac=mac, y_mac=y_mac, x_le_mac=x_le_mac,
                x_qc_mac=x_le_mac + 0.25 * mac, taper=c[-1] / c[0])


def tail_dims(d):
    bh = math.sqrt(d["Sh"] * d["ARh"])
    ch_r = 2 * d["Sh"] / (bh * (1 + d["taper_h"]))
    ch_t = ch_r * d["taper_h"]
    hv = math.sqrt(d["Sv"] * d["ARv"])
    cv_r = 2 * d["Sv"] / (hv * (1 + d["taper_v"]))
    cv_t = cv_r * d["taper_v"]
    x_h_qc = 0.25 * d["c"][0] + d["lt"]        # htail root c/4
    x_h_le = x_h_qc - 0.25 * ch_r
    # fin: TE flush with the htail root TE (rudder clears elevator because the elevator is split around the fin)
    x_v_le = x_h_le + ch_r - cv_r
    mac_h = 2 / 3 * ch_r * (1 + d["taper_h"] + d["taper_h"] ** 2) / (1 + d["taper_h"])
    mac_v = 2 / 3 * cv_r * (1 + d["taper_v"] + d["taper_v"] ** 2) / (1 + d["taper_v"])
    return dict(bh=bh, ch_r=ch_r, ch_t=ch_t, hv=hv, cv_r=cv_r, cv_t=cv_t, x_h_le=x_h_le, x_h_qc=x_h_qc,
                x_v_le=x_v_le, mac_h=mac_h, mac_v=mac_v,
                x_h_ac=x_h_le + 0.25 * mac_h + (ch_r - mac_h) * 0.25,  # approx (unswept c/4 line at 25 %)
                x_v_ac=x_v_le + (cv_r - cv_t) * 0.25 * 0 + 0.25 * mac_v + (cv_r - mac_v) * 0.25,
                tail_end=x_h_le + ch_r)


def wing_xsecs(d, deflect_ail=0.0, flaperon=0.0):
    """WingXSecs (right half) incl. extra xsecs at the aileron ends so control surfaces sit exactly there."""
    y = list(d["y"])
    c = list(d["c"])
    tw = list(d["twist"])
    # insert aileron start/end stations
    ys = sorted(set(y + list(d["ail_y"])))
    cs = np.interp(ys, y, c)
    tws = np.interp(ys, y, tw)
    # z from per-panel dihedral (piecewise along original panels)
    zs = []
    for yy in ys:
        z = 0.0
        for i in range(len(y) - 1):
            seg = max(0.0, min(yy, y[i + 1]) - y[i])
            z += seg * math.tan(math.radians(d["dihedral"][i]))
        zs.append(z)
    xsecs = []
    for k, yy in enumerate(ys):
        cs_k = []
        if d["ail_y"][0] - 1e-9 <= yy < d["ail_y"][1] - 1e-9:
            cs_k.append(asb.ControlSurface(name="aileron", symmetric=False, deflection=deflect_ail,
                                           hinge_point=1 - d["ail_cf"]))
            if flaperon:
                cs_k.append(asb.ControlSurface(name="flaperon", symmetric=True, deflection=flaperon,
                                               hinge_point=1 - d["ail_cf"]))
        xle = 0.25 * (c[0] - cs[k])
        xsecs.append(asb.WingXSec(xyz_le=[xle, yy, zs[k]], chord=float(cs[k]), twist=float(tws[k]),
                                  airfoil=load_foil(d["foil"], float(cs[k])), control_surfaces=cs_k))
    return xsecs


def build_airplane(d, i_t=None, fuselage=True, xyz_ref=(0, 0, 0), deflect_ail=0.0, flaperon=0.0,
                   elevator=0.0, rudder=0.0, tails=True, wing=True):
    """AeroSandbox airplane.  The whole wing is rotated by i_w about the root LE (twist adds i_w)."""
    i_t = d["i_t"] if i_t is None else i_t
    wg = wing_geom_numbers(d)
    td = tail_dims(d)
    xs = wing_xsecs(d, deflect_ail, flaperon)
    for x in xs:
        x.twist = x.twist + d["i_w"]
    wings = []
    if wing:
        wings.append(asb.Wing(name="Main wing", symmetric=True, xsecs=xs))
    if tails:
        fh = plate_foil(td["ch_r"])
        el = [asb.ControlSurface(name="elevator", symmetric=True, deflection=elevator, hinge_point=1 - d["elev_cf"])]
        wings.append(asb.Wing(name="Elevator", symmetric=True, xsecs=[
            asb.WingXSec(xyz_le=[td["x_h_le"], 0.0, d["zh"]], chord=td["ch_r"], twist=i_t, airfoil=fh,
                         control_surfaces=el),
            asb.WingXSec(xyz_le=[td["x_h_le"] + 0.25 * (td["ch_r"] - td["ch_t"]), td["bh"] / 2, d["zh"]],
                         chord=td["ch_t"], twist=i_t, airfoil=plate_foil(td["ch_t"]))]))
        fv = plate_foil(td["cv_r"])
        rd = [asb.ControlSurface(name="rudder", symmetric=True, deflection=rudder, hinge_point=1 - d["rud_cf"])]
        wings.append(asb.Wing(name="Fin", symmetric=False, xsecs=[
            asb.WingXSec(xyz_le=[td["x_v_le"], 0.0, d["zh"]], chord=td["cv_r"], twist=0.0, airfoil=fv,
                         control_surfaces=rd),
            asb.WingXSec(xyz_le=[td["x_v_le"] + (td["cv_r"] - td["cv_t"]), 0.0, d["zh"] + td["hv"]],
                         chord=td["cv_t"], twist=0.0, airfoil=plate_foil(td["cv_t"]))]))
    fus = []
    if fuselage:
        fus = fuselages(d)
    return asb.Airplane(name=d["name"], xyz_ref=list(xyz_ref), wings=wings, fuselages=fus,
                        s_ref=wg["S"], c_ref=wg["mac"], b_ref=wg["b"])


def fuselages(d):
    """Printed pod (spinner -> wing TE, super-elliptic section) + 8 mm carbon boom."""
    x0 = d["x_nose"] - 0.025               # spinner tip ahead of firewall
    L = d["pod_len"] + 0.025
    R = d["pod_d"] / 2
    zc = d["pod_zc"]
    xs_n = np.array([0.0, 0.03, 0.08, 0.15, 0.55, 0.80, 1.0])
    rs_n = np.array([0.30, 0.78, 0.97, 1.0, 1.0, 0.62, 0.22])
    pod = asb.Fuselage(name="Pod", xsecs=[asb.FuselageXSec(xyz_c=[x0 + L * a, 0, zc], radius=R * r)
                                          for a, r in zip(xs_n, rs_n)])
    td = tail_dims(d)
    boom = asb.Fuselage(name="Boom", xsecs=[asb.FuselageXSec(xyz_c=[d["boom_x0"], 0, zc + 0.012], radius=d["boom_od"] / 2),
                                            asb.FuselageXSec(xyz_c=[td["tail_end"], 0, d["zh"] - d["boom_od"] / 2],
                                                             radius=d["boom_od"] / 2)])
    return [pod, boom]


# ----------------------------------------------------------------------------- mass model & CG
# Avionics actually chosen for concept B (lightest viable set).  Sources in report.md section 2.
AVIONICS_B = dict(
    fc=("Matek F405-WMN (ArduPilot MatekF405-TE target), 10 g + 2 g leads/mount", 0.012),
    rx=("FlySky FS-A8S iBUS, ~2 g incl. antennas", 0.002),
    gps=("Matek M10Q-5883 GPS+compass", 0.008),
    aspd=("Matek ASPD-4525 + pitot + silicone tube", 0.007),
    servo_each=("EMAX ES9051 4.1 g (x4)", 0.0041),
    esc=("12 A BLHeli_S/Skywalker-12A class ESC, 10 g", 0.010),
)

PROP_SETS = {
    # motor (name, kg), prop+spinner kg, battery (name, kg, mAh, cells), propulsion-model keys
    "X2204": dict(motor=("SunnySky X2204 1480KV", 0.0200), prop_spinner=0.016,
                  batt=("3S 450 mAh 75C (Tattu)", 0.042, 450, 3)),
    "X2204_650": dict(motor=("SunnySky X2204 1480KV", 0.0200), prop_spinner=0.016,
                      batt=("3S 650 mAh 45C", 0.056, 650, 3)),
    "X2208": dict(motor=("SunnySky X2208-II 1100KV", 0.044), prop_spinner=0.016,
                  batt=("3S 450 mAh 75C (Tattu)", 0.042, 450, 3)),
}


def mass_items(d, density_gcc=0.75, prop_set="X2204", contingency=0.05, verbose=False):
    """Component list [(name, kg, x, y, z)] for CG and inertia.
    Structural masses come from the stage-1 parametric model with PLA-Aero density overridden.
    Distributed items (wing shell, spar, boom) are split into many point masses for the inertia."""
    rho_kgm3 = density_gcc * 1000.0
    key = f"B_{density_gcc:.2f}"
    MM.CONSTRUCTIONS[key] = dict(MM.CONSTRUCTIONS["recommended"], label=f"A1 PLA Aero {density_gcc:.2f} g/cc",
                                 wing_rho=rho_kgm3, pod_material_areal=0.8e-3 * rho_kgm3)
    wg = wing_geom_numbers(d)
    td = tail_dims(d)
    ps = PROP_SETS[prop_set]
    fus_len = td["tail_end"] - d["x_nose"]
    m = MM.mass_breakdown(span_m=wg["b"], wing_area_m2=wg["S"], htail_area_m2=d["Sh"], vtail_area_m2=d["Sv"],
                          fuselage_length_m=fus_len, battery_mass_kg=ps["batt"][1], motor_mass_kg=ps["motor"][1],
                          construction=key, airfoil_tc=0.092, taper_ratio=wg["taper"], n_servos=4,
                          servo="servo_emax_es9051", fc="fc_none", rx="rx_none_integrated", gps="gps_none",
                          airspeed="airspeed_none", esc="esc_12a_light", prop="folding",
                          contingency_frac=0.0)
    # --- override: concept B avionics (sourced list) and prop/spinner
    avi = {k: v[1] for k, v in AVIONICS_B.items()}
    items = []
    zc = d["pod_zc"]
    xn = d["x_nose"]
    # propulsion & avionics in the pod
    items += [("prop+spinner (folding 9x5, 30 mm)", ps["prop_spinner"], xn - 0.022, 0, zc),
              (f"motor {ps['motor'][0]}", ps["motor"][1], xn - 0.010, 0, zc),
              ("motor mount (PETG) & screws", 0.005, xn + 0.003, 0, zc),
              ("ESC", avi["esc"], xn + 0.035, 0, zc - 0.008),
              (f"battery {ps['batt'][0]}", ps["batt"][1], xn + 0.095, 0, zc - 0.004),
              ("receiver FS-A8S", avi["rx"], -0.030, 0, zc + 0.010),
              ("flight controller F405-WMN", avi["fc"], -0.010, 0, zc),
              ("airspeed sensor + pitot (wing root / LE y=0.20)", avi["aspd"], 0.0, 0, 0.0),
              ("GPS M10Q-5883 (on wing centre, top)", avi["gps"], 0.070, 0, 0.018),
              ("elevator servo (pod)", avi["servo_each"], 0.105, 0, zc),
              ("rudder servo (pod)", avi["servo_each"], 0.105, 0, zc),
              ("wiring, XT30, extensions", m["wiring_connectors"], xn + 0.12, 0, zc),
              ("pushrods/horns tail (2)", 0.003, 0.45, 0, zc + 0.015),
              ]
    if d.get("ballast", 0) > 0:
        items.append(("nose ballast", d["ballast"] / 1000, xn + 0.005, 0, zc))
    # aileron servos & horns in the wing at mid panel root
    ys = 0.262
    cs = float(np.interp(ys, d["y"], d["c"]))
    xs = 0.25 * (d["c"][0] - cs) + 0.40 * cs
    for sgn in (1, -1):
        items.append(("aileron servo + horn", avi["servo_each"] + 0.0015, xs, sgn * ys, 0.0))
    # pod shell (stage-1 model) distributed over pod length; pod fixed parts (saddle, hatch, boom socket)
    n = 8
    for k in range(n):
        items.append(("pod shell (printed)", m["fuselage_pod"] - 0.019 + 0.005, xn + (k + 0.5) / n * d["pod_len"], 0, zc))
    items[-n:] = [(a, (m["fuselage_pod"] - 0.019) / n, x, y, z) for (a, _, x, y, z) in items[-n:]]
    items.append(("pod fixed parts (saddle, hatch, boom socket)", 0.019 - 0.005, 0.03, 0, zc))
    # boom
    L_boom = td["tail_end"] - d["boom_x0"]
    lin = MM.cf_linear_density(8.0, 6.0)
    for k in range(n):
        items.append(("tail boom CF 8x6", lin * L_boom / n, d["boom_x0"] + (k + 0.5) / n * L_boom, 0, zc + 0.012))
    items.append(("tail mounts (printed clamp)", m["tail_mounts"], td["x_h_le"] + 0.3 * td["ch_r"], 0, d["zh"]))
    items.append(("H-tail (3 mm Depron + CF strip)", m["htail"], td["x_h_le"] + 0.45 * td["ch_r"], 0, d["zh"]))
    items.append(("V-tail/fin (3 mm Depron)", m["vtail"], td["x_v_le"] + 0.5 * td["cv_r"], 0, d["zh"] + 0.42 * td["hv"]))
    # wing: shell + joints by area strips, spars by span
    y_edges = np.linspace(-wg["b"] / 2, wg["b"] / 2, 29)
    yc = 0.5 * (y_edges[1:] + y_edges[:-1])
    cc = np.interp(np.abs(yc), d["y"], d["c"])
    zz = np.array([_z_at(d, abs(v)) for v in yc])
    area_w = cc * np.diff(y_edges)
    area_w /= area_w.sum()
    shell = m["wing_shell"] + m["wing_joints"]
    for k in range(len(yc)):
        xle = 0.25 * (d["c"][0] - cc[k])
        items.append(("wing shell + segment joints", shell * area_w[k], xle + 0.42 * cc[k], yc[k], zz[k]))
        items.append(("wing spars (main CF + 2 mm rear rod)", (m["wing_main_spar"] + m["wing_rear_spar"]) / len(yc),
                      xle + 0.33 * cc[k], yc[k], zz[k]))
    items.append(("wing hardware (bolts/magnets, servo covers)", m["wing_hardware"], 0.05, 0, 0.0))
    # glue: proportional to structure, at structure centroid (computed below)
    struct = [it for it in items if any(s in it[0] for s in ("pod", "boom", "tail", "H-tail", "fin", "wing"))]
    ms = sum(it[1] for it in struct)
    xg = sum(it[1] * it[2] for it in struct) / ms
    items.append(("glue (4 % of structure)", MM.CONSTRUCTIONS[key]["glue_frac"] * ms, xg, 0, 0))
    # contingency on everything except battery and motor, at the CG of those items
    if contingency:
        base = [it for it in items if not it[0].startswith("battery") and not it[0].startswith("motor ")]
        mb = sum(it[1] for it in base)
        xb = sum(it[1] * it[2] for it in base) / mb
        zb = sum(it[1] * it[4] for it in base) / mb
        items.append((f"contingency {contingency*100:.0f} %", contingency * mb, xb, 0, zb))
    info = dict(stage1=m["info"], spar=m["info"]["main_spar"], wing_shell=m["wing_shell"],
                wing_main_spar=m["wing_main_spar"], wing_rear_spar=m["wing_rear_spar"],
                wing_joints=m["wing_joints"])
    return items, info


def _z_at(d, yy):
    y = d["y"]
    z = 0.0
    for i in range(len(y) - 1):
        seg = max(0.0, min(yy, y[i + 1]) - y[i])
        z += seg * math.tan(math.radians(d["dihedral"][i]))
    return z


def mass_props(items):
    m = sum(it[1] for it in items)
    x = sum(it[1] * it[2] for it in items) / m
    y = sum(it[1] * it[3] for it in items) / m
    z = sum(it[1] * it[4] for it in items) / m
    Ixx = sum(it[1] * ((it[3] - y) ** 2 + (it[4] - z) ** 2) for it in items)
    Iyy = sum(it[1] * ((it[2] - x) ** 2 + (it[4] - z) ** 2) for it in items)
    Izz = sum(it[1] * ((it[2] - x) ** 2 + (it[3] - y) ** 2) for it in items)
    Ixz = sum(it[1] * (it[2] - x) * (it[4] - z) for it in items)
    return dict(m=m, x=x, y=y, z=z, Ixx=Ixx, Iyy=Iyy, Izz=Izz, Ixz=Ixz)


def grouped_breakdown(items):
    """Merge distributed items for the report table: name -> (mass, x_cg, z_cg)."""
    out = {}
    for n, mm, x, y, z in items:
        if n not in out:
            out[n] = [0.0, 0.0, 0.0]
        out[n][0] += mm
        out[n][1] += mm * x
        out[n][2] += mm * z
    return {n: (v[0], v[1] / v[0] if v[0] else 0, v[2] / v[0] if v[0] else 0) for n, v in out.items()}


# ----------------------------------------------------------------------------- VLM helpers
VLM_KW = dict(spanwise_resolution=8, chordwise_resolution=8)


def vlm(airplane, V, alpha, beta=0.0, p=0.0, q=0.0, r=0.0, xyz_ref=None, **kw):
    kk = dict(VLM_KW)
    kk.update(kw)
    v = asb.VortexLatticeMethod(airplane=airplane, op_point=op(V, alpha, beta, p, q, r),
                                xyz_ref=list(airplane.xyz_ref if xyz_ref is None else xyz_ref), **kk)
    res = v.run()
    return v, res


def vlm_strips(v, airplane, wing_index=0):
    """Spanwise strips of one wing from a solved VLM: y_mid, chord, width, local Cl (normal-force based)."""
    nc = v.chordwise_resolution
    counts = []
    for w in airplane.wings:
        ww = w.subdivide_sections(ratio=v.spanwise_resolution, spacing_function=v.spanwise_spacing_function) \
            if v.spanwise_resolution > 1 else w
        pts, faces = ww.mesh_thin_surface(method="quad", chordwise_resolution=nc,
                                          chordwise_spacing_function=v.chordwise_spacing_function, add_camber=True)
        counts.append(len(faces))
    i0 = int(sum(counts[:wing_index]))
    i1 = i0 + counts[wing_index]
    F = v.forces_geometry[i0:i1].reshape(-1, nc, 3).sum(axis=1)
    fl = v.front_left_vertices[i0:i1].reshape(-1, nc, 3)
    fr = v.front_right_vertices[i0:i1].reshape(-1, nc, 3)
    bl = v.back_left_vertices[i0:i1].reshape(-1, nc, 3)
    br = v.back_right_vertices[i0:i1].reshape(-1, nc, 3)
    y_mid = 0.25 * (fl[:, 0, 1] + fr[:, 0, 1] + bl[:, -1, 1] + br[:, -1, 1])
    width = np.hypot(fr[:, 0, 1] - fl[:, 0, 1], fr[:, 0, 2] - fl[:, 0, 2])
    chord = 0.5 * ((bl[:, -1, 0] - fl[:, 0, 0]) + (br[:, -1, 0] - fr[:, 0, 0]))
    # lift direction (wind axes, geometry frame): perpendicular to freestream in x-z plane
    a = math.radians(v.op_point.alpha)
    # geometry axes: x aft, z up; freestream comes from -x... asb geometry: x aft => freestream direction (+cos a, 0, +sin a)?
    fs = v.op_point.compute_freestream_direction_geometry_axes()
    fs = np.array([float(fs[0]), float(fs[1]), float(fs[2])])
    lift_dir = np.cross(fs, np.array([0.0, 1.0, 0.0]))
    lift_dir = lift_dir / np.linalg.norm(lift_dir)
    if lift_dir[2] < 0:
        lift_dir = -lift_dir
    q = v.op_point.dynamic_pressure()
    lprime = F @ lift_dir
    cl = lprime / (q * chord * width)
    o = np.argsort(y_mid)
    return dict(y=y_mid[o], c=chord[o], dy=width[o], cl=cl[o], a=a)


# ----------------------------------------------------------------------------- NeuralFoil section tables
def section_table(foil: asb.Airfoil, Re_val: float, n_crit: float = 9.0, alphas=None, model_size="large"):
    alphas = np.linspace(-8, 18, 105) if alphas is None else alphas
    r = foil.get_aero_from_neuralfoil(alpha=alphas, Re=Re_val * np.ones_like(alphas), n_crit=n_crit,
                                      model_size=model_size)
    return dict(alpha=alphas, cl=np.asarray(r["CL"]), cd=np.asarray(r["CD"]), cm=np.asarray(r["CM"]),
                conf=np.asarray(r["analysis_confidence"]))


def clmax_of(t):
    i = int(np.argmax(t["cl"]))
    return float(t["cl"][i]), float(t["alpha"][i])


def cd_at_cl(t, cl):
    """Cd at a given Cl on the pre-stall branch (cl beyond Clmax -> returns Cd at Clmax and flag)."""
    i = int(np.argmax(t["cl"]))
    a_cl = t["cl"][: i + 1]
    a_cd = t["cd"][: i + 1]
    # keep monotonic part
    j0 = int(np.argmin(a_cl))
    a_cl, a_cd = a_cl[j0:], a_cd[j0:]
    cl_c = np.clip(cl, a_cl[0], a_cl[-1])
    return np.interp(cl_c, a_cl, a_cd), cl > a_cl[-1]


def save_json(name, obj):
    def conv(o):
        if isinstance(o, (np.floating, np.integer)):
            return o.item()
        if isinstance(o, np.ndarray):
            return o.tolist()
        return str(o)
    with open(os.path.join(RES, name), "w") as f:
        json.dump(obj, f, indent=1, default=conv)


def load_json(name):
    with open(os.path.join(RES, name)) as f:
        return json.load(f)


def load_design():
    """Converged design written by a2_sizing.py (falls back to BASE)."""
    p = os.path.join(RES, "design.json")
    if os.path.exists(p):
        with open(p) as f:
            dd = json.load(f)
        d = deepcopy(BASE)
        d.update(dd["geometry"])
        return d, dd
    return deepcopy(BASE), None
