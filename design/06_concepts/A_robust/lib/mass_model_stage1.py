"""
Parametric mass model for the small hand-launched powered glider (EC4.402 project).

Usage
-----
    from mass_model import mass_breakdown
    m = mass_breakdown(span_m=1.4, wing_area_m2=0.178, htail_area_m2=0.030,
                       vtail_area_m2=0.016, fuselage_length_m=1.0,
                       battery_mass_kg=0.042, motor_mass_kg=0.030)
    print(m["total"])

Run this file directly to print the example tables (spans 1.0-1.6 m, AR 10-12)
and the construction comparison, and to write example_breakdowns.md / .png.

All masses in kg.  Areal densities are per m^2 of PLANFORM (projected) area unless noted.
Sources for every constant are listed in construction_recommendation.md (section 5/6);
values marked "ASSUMED" there are engineering estimates, not sourced numbers.
"""
from __future__ import annotations

import math
from copy import deepcopy

G = 9.81

# ----------------------------------------------------------------------------
# 1. Material constants
# ----------------------------------------------------------------------------
# Printed-shell effective densities (g/cm^3 -> kg/m^3 below)
RHO_PLA = 1240.0            # solid PLA
RHO_PLA_AERO_TUNED = 650.0  # Bambu PLA Aero, foamed. Bambu: 0.55-1.05 g/cm3 depending on temp/flow
RHO_LW_PLA = 480.0          # colorFabb LW-PLA near max foaming: 0.40-0.48 g/cm3 (colorFabb / CNC Kitchen)
RHO_CF = 1550.0             # pultruded carbon tube/rod (typical 1.5-1.6 g/cm3, ASSUMED)
RHO_DEPRON_3MM = 40.0       # Depron 3 mm datasheet ~40 kg/m3 (6 mm ~33 kg/m3)
RHO_XPS = 32.0              # Indian XPS insulation board, 28-35 kg/m3 (IndiaMART listings)
RHO_SUNBOARD = 450.0        # Indian PVC "sunboard", 0.35-0.55 g/cm3 (IndiaMART) -> REJECTED material

# Carbon tubes commonly sold in India (1 mm wall, 1000 mm long; Robu / IndianRoboStore)
# (name, OD mm, ID mm)
CF_TUBES = [
    ("CF tube 4x2", 4.0, 2.0),
    ("CF tube 5x3", 5.0, 3.0),
    ("CF tube 6x4", 6.0, 4.0),
    ("CF tube 8x6", 8.0, 6.0),
    ("CF tube 10x8", 10.0, 8.0),
    ("CF tube 12x10", 12.0, 10.0),
]
# Solid rods used only as telescoping outboard spars / rear spar (fit inside 4x2 / 5x3 tubes)
CF_RODS = [
    ("CF rod 2 mm", 2.0, 0.0),
    ("CF rod 3 mm", 3.0, 0.0),
]


def cf_linear_density(od_mm: float, id_mm: float = 0.0) -> float:
    """kg/m of a carbon tube / rod."""
    a = math.pi / 4.0 * (od_mm ** 2 - id_mm ** 2) * 1e-6
    return a * RHO_CF


def cf_I(od_mm: float, id_mm: float) -> float:
    """Second moment of area, m^4."""
    return math.pi / 64.0 * ((od_mm * 1e-3) ** 4 - (id_mm * 1e-3) ** 4)


# ----------------------------------------------------------------------------
# 2. Fixed (avionics / propulsion accessory) masses, kg  -- sourced, see .md
# ----------------------------------------------------------------------------
AVIONICS = {
    # Flight controllers
    "fc_matek_f405_wte": 0.022,      # 22 g incl. USB/buzzer adapter, integrated ELRS 2.4 GHz RX (mateksys.com)
    "fc_matek_f411_wte": 0.010,      # 10 g (mateksys.com) - INAV only, no integrated RX
    "fc_matek_f405_minite": 0.005,   # 5 g bare 20x20 board (team-blacksheep / defiancerc) - no wing PDB
    "fc_speedybee_f405_wing_mini": 0.023,
    "fc_none": 0.0,  # 23 g FC+PDB stack (oscarliang.com review)
    # Receivers
    "rx_none_integrated": 0.0,
    "rx_radiomaster_rp1": 0.0022,    # 2.2 g incl. antenna (radiomasterrc.com)
    "rx_betafpv_elrs_lite": 0.0005,  # 0.46-0.53 g bare (betafpv retailers)
    # Servos (each)
    "servo_emax_es9051": 0.0041,     # 4.1 g, 0.8 kg.cm, plastic gear (retailers)
    "servo_emax_es9052md": 0.0055,   # 5.5 g, metal gear digital (getfpv / racedayquads)
    "servo_towerpro_sg90": 0.009,    # 9 g, 1.8 kg.cm; many counterfeits (TowerPro warning)
    "servo_emax_es08ma2": 0.012,     # 12 g, 1.6-2.0 kg.cm metal gear analog
    "servo_corona_ds929mg": 0.0125,  # 12.5 g metal gear digital (servodatabase / hobbyking)
    # GPS
    "gps_none": 0.0,
    "gps_beitian_bn220": 0.0053,     # 5.3 g
    "gps_matek_m10q_5883": 0.008,    # ~8 g incl. compass (7.5-8 g, retailers)
    # ESC
    "esc_12a_light": 0.010,          # Hobbywing Skywalker 12AE: 10 g
    "esc_20a_skywalker": 0.019,      # Hobbywing Skywalker 20A: 19 g
    "esc_generic_12_20a": 0.012,     # ASSUMED mid value for a light 12-20 A (no-BEC) ESC incl. wires
    # Prop + spinner
    "spinner_30mm_folding": 0.012,   # Hyperflight MFZ30 30 mm, 12 g (10-14.5 g range for 30 mm spinners)
    "folding_blades_pair": 0.004,    # ASSUMED 7-8" folding blade pair
    "fixed_prop_adapter": 0.006,     # ASSUMED small fixed prop + prop-saver (calibration cases only)
    # Airspeed sensor (optional, recommended for XFLR5-vs-flight comparison)
    "airspeed_none": 0.0,
    "airspeed_ms4525_pitot": 0.006,  # ASSUMED (sensor board + pitot + tubing)
}

# Reference batteries / motors (for example tables), kg
BATTERIES = {
    "2S 450 mAh (Tattu 75C)": 0.029,
    "3S 450 mAh (Tattu 75C)": 0.042,
    "2S 1000 mAh (Orange 20C, Robu)": 0.051,
    "3S 1000 mAh (Orange 30C/40C, Robu)": 0.085,
}
MOTORS = {
    "2204 (SunnySky X2204)": 0.021,
    "2204 (EMAX MT2204)": 0.025,
    "2205 (EMAX RS2205)": 0.030,
    "2208 (SunnySky X2208)": 0.044,
}

# ----------------------------------------------------------------------------
# 3. Construction presets
# ----------------------------------------------------------------------------
# Printed shells: areal density per planform = perimeter_ratio * wall_t * rho * internal_factor
#   perimeter_ratio ~2.04 for a 8-10 % airfoil (wetted / planform)
#   internal_factor accounts for vase-mode internal rib lattice / diagonal webs (+25..45 %)
# Calibration check: 3DLabPrint Bungee Prop (LW-PLA, 21.2 dm2 wing, 216 g total print):
#   2.04*0.45mm*0.48*1.35 = 5.9 g/dm2 -> wing ~126 g, leaving ~90 g for pod+boom+tails: consistent.

_BASE = dict(
    label="",
    wing_type="printed",           # printed | dbox_hybrid | hotwire | foamboard | sunboard
    wing_wall_t=0.45e-3,           # single-wall vase line width (0.4 mm nozzle)
    wing_rho=RHO_PLA_AERO_TUNED,
    wing_perim=2.04,
    wing_internal=1.35,
    seg_len=0.24,                  # printable segment length (P1S 256 mm Z, keep 240 mm)
    seg_joint_mass=0.0015,         # per segment joint: thicker root rib/key + glue (ASSUMED)
    tail_type="depron",            # depron | printed
    tail_printed_internal=1.15,
    tail_depron_extra=0.06,        # kg/m2 (=0.6 g/dm2): 1-2 mm CF strip LE/stiffener + hinge tape (ASSUMED)
    pod_material_areal=0.8e-3 * 750.0,  # 2-wall PLA Aero pod, 0.8 mm at 0.75 g/cm3 -> 0.6 kg/m2
    pod_fixed=0.019,               # motor mount (PLA/PETG), wing saddle+nylon bolts, hatch magnets,
                                   # boom socket, bulkheads (ASSUMED)
    pod_diam=0.048,                # fits FC 44x29 mm and 3S 450-850 packs
    tail_mount=0.008,              # printed H/V tail fittings on boom (ASSUMED)
    glue_frac=0.04,                # CA/epoxy/foam-safe glue as fraction of structure (ASSUMED)
)

CONSTRUCTIONS = {
    # Option A-1 (RECOMMENDED): printed PLA Aero wing+pod, CF spar and boom, 3 mm Depron/XPS tails
    "A1_printed_aero_foam_tail": dict(_BASE, label="A1: PLA Aero printed wing+pod, CF spar/boom, 3 mm Depron tails"),
    # Option A-2: everything printed in PLA Aero incl. tails (simplest, slightly heavier)
    "A2_printed_aero_all": dict(_BASE, label="A2: All printed PLA Aero (incl. tails)", tail_type="printed"),
    # Option A-3: same as A1 but colorFabb/Polymaker LW-PLA (imported, lighter)
    "A3_printed_lwpla_foam_tail": dict(_BASE, label="A3: LW-PLA printed wing+pod, Depron tails",
                                       wing_rho=RHO_LW_PLA, pod_material_areal=0.8e-3 * 560.0),
    # Option A-0: PLA Aero printed but NOT tuned (default foaming, ~0.85 g/cm3) - pessimistic
    "A3_printed_lwpla_all": dict(_BASE, label="A3b: LW-PLA everything printed (calibration)",
                                 wing_rho=RHO_LW_PLA, pod_material_areal=0.45e-3 * 560.0, tail_type="printed",
                                 pod_fixed=0.012),
    "A0_printed_aero_untuned": dict(_BASE, label="A0: PLA Aero untuned (~0.85 g/cm3), all printed",
                                    wing_rho=850.0, tail_type="printed",
                                    pod_material_areal=0.8e-3 * 900.0),
    # Option C: Planeprint-style hybrid: printed PLA Aero D-box (0-30 % c) + 3 mm Depron skins aft + printed ribs
    "C_dbox_hybrid": dict(_BASE, label="C: Printed D-box LE + 3 mm Depron aft skins + CF spar", wing_type="dbox_hybrid"),
    # Option B-1: hot-wire cut XPS core wing, packing-tape/lamination covering, printed pod
    "B1_hotwire_xps": dict(_BASE, label="B1: Hot-wire XPS core + tape covering, printed pod", wing_type="hotwire"),
    # Option B-2: folded paper-faced 5 mm foam board (FliteBoard/DTFB-style) wing
    "B2_foamboard_folded": dict(_BASE, label="B2: Folded 5 mm paper-faced XPS foamboard wing", wing_type="foamboard"),
    # Rejected: Indian PVC Sunboard 5 mm (for illustration only)
    "X_sunboard_5mm": dict(_BASE, label="X: Indian 5 mm PVC Sunboard (REJECTED)", wing_type="sunboard"),
}
CONSTRUCTIONS["recommended"] = dict(CONSTRUCTIONS["A1_printed_aero_foam_tail"])


def wing_areal_density(c: dict, mean_chord: float, airfoil_tc: float) -> float:
    """kg per m^2 planform for the wing SHELL (without spars/joints/hardware)."""
    t = c["wing_type"]
    if t == "printed":
        return c["wing_perim"] * c["wing_wall_t"] * c["wing_rho"] * c["wing_internal"]
    if t == "dbox_hybrid":
        # D-box: front 30 % of chord wrapped (perimeter ~0.68 c), printed single wall + light internal webs
        dbox = 0.68 * c["wing_wall_t"] * c["wing_rho"] * 1.2
        # aft 70 % upper+lower 3 mm Depron skins (~1.44 c of surface)
        foam = 1.44 * 3e-3 * RHO_DEPRON_3MM
        ribs = 0.25  # printed ribs every ~80 mm + TE strip (ASSUMED) kg/m2
        return dbox + foam + ribs
    if t == "hotwire":
        core = 0.68 * airfoil_tc * mean_chord * RHO_XPS        # airfoil area ~0.68 t c
        tape = 2.04 * 0.050                                       # ~50 g/m2 packing tape / laminate film (ASSUMED)
        le_te = 0.30                                              # LE/TE tape reinforcement, servo pockets (ASSUMED)
        return core + tape + le_te
    if t == "foamboard":
        # FT-style folded wing ~2.3 layers of 5 mm paper-faced foamboard @ ~3.6 g/dm2 (ASSUMED DTFB-class)
        return 2.3 * 0.36
    if t == "sunboard":
        return 2.3 * 5e-3 * RHO_SUNBOARD
    raise ValueError(t)


def tail_areal_density(c: dict, tail_tc: float = 0.07) -> float:
    if c["tail_type"] == "depron":
        return 3e-3 * RHO_DEPRON_3MM + c["tail_depron_extra"]
    # printed symmetric section, thinner -> perimeter ~2.02
    return 2.02 * c["wing_wall_t"] * c["wing_rho"] * c["tail_printed_internal"]


def elliptic_moment_fraction(eta: float, n: int = 400) -> float:
    """Bending moment at spanwise station eta (0=root,1=tip) for elliptic lift,
    as a fraction of (half-wing lift * semi-span)."""
    import numpy as np
    y = np.linspace(eta, 1.0, n)
    l = np.sqrt(np.clip(1 - y ** 2, 0, None))
    half_lift = math.pi / 4.0
    return float(np.trapezoid(l * (y - eta), y) / half_lift)


def _tube_ok(od, idd, M, sigma_allow, t_avail, fit_frac):
    sigma = M * (od * 1e-3 / 2.0) / cf_I(od, idd)
    return sigma <= sigma_allow and od * 1e-3 <= fit_frac * t_avail, sigma


def _size_main_spar(W_total_N, span, t_root, t_tip, n_ult, sigma_allow, fit_frac=0.80, overlap=0.06):
    """Stepped, telescoping 1 mm-wall CF spar (e.g. 6x4 centre tube with 4x2 outer tubes sliding inside).
    Inner tube spans +/- eta_step of the semi-span (one piece through the fuselage = wing joiner),
    outer tubes run from eta_step-overlap to 97 % semi-span.
    Returns dict(mass, inner, outer, eta_step, sigma_root, sigma_step, M_root)."""
    s = span / 2.0
    M0 = n_ult * (W_total_N / 2.0) * s * elliptic_moment_fraction(0.0)
    inner = None
    for name, od, idd in CF_TUBES:
        ok, sig = _tube_ok(od, idd, M0, sigma_allow, t_root, fit_frac)
        if ok:
            inner = (name, od, idd, sig)
            break
    flag = ""
    if inner is None:
        name, od, idd = CF_TUBES[-1]
        inner = (name, od, idd, M0 * (od * 1e-3 / 2) / cf_I(od, idd))
        flag = " (OVERSTRESSED/does not fit: raise t/c or chord)"
    lin_in = cf_linear_density(inner[1], inner[2])
    # outer tube: the CF tube whose OD equals inner ID (telescoping), else no step
    outer_c = [t for t in CF_TUBES + CF_RODS if abs(t[1] - inner[2]) < 1e-6]
    best = dict(mass=0.97 * span * lin_in, inner=inner[0] + flag, outer=None, eta_step=0.97,
                sigma_root=inner[3], sigma_step=None, M_root=M0)
    if outer_c:
        oname, ood, oid = outer_c[0]
        lin_out = cf_linear_density(ood, oid)
        for eta in (0.35, 0.40, 0.45, 0.50, 0.55, 0.60, 0.65, 0.70):
            if eta * span > 1.0:      # inner tube must come from one 1 m stock length
                break
            M = n_ult * (W_total_N / 2.0) * s * elliptic_moment_fraction(eta)
            t_here = t_root + (t_tip - t_root) * eta
            ok, sig = _tube_ok(ood, oid, M, sigma_allow, t_here, 1.0)
            if ok:
                mass = eta * span * lin_in + 2 * ((0.97 - eta) * s + overlap) * lin_out
                if mass < best["mass"]:
                    best = dict(mass=mass, inner=inner[0] + flag, outer=oname, eta_step=eta,
                                sigma_root=inner[3], sigma_step=sig, M_root=M0)
                break
    if best["outer"] is None and 0.97 * span > 1.0:
        # no step possible: two half-span tubes + 0.20 m sleeve joiner of next size up
        bigger = [t for t in CF_TUBES if t[1] > inner[1]]
        j = bigger[0] if bigger else (inner[0], inner[1], inner[2])
        best["mass"] += 0.20 * cf_linear_density(j[1], j[2])
    return best


def mass_breakdown(span_m: float,
                   wing_area_m2: float,
                   htail_area_m2: float,
                   vtail_area_m2: float,
                   fuselage_length_m: float,
                   battery_mass_kg: float,
                   motor_mass_kg: float,
                   construction: str = "recommended",
                   *,
                   airfoil_tc: float = 0.09,
                   taper_ratio: float = 0.6,
                   n_servos: int = 4,
                   servo: str = "servo_emax_es9052md",
                   fc: str = "fc_matek_f405_wte",
                   rx: str = "rx_none_integrated",
                   gps: str = "gps_matek_m10q_5883",
                   airspeed: str = "airspeed_none",
                   esc: str = "esc_generic_12_20a",
                   prop: str = "folding",
                   n_ult: float = 6.0,
                   sigma_allow_pa: float = 400e6,
                   contingency_frac: float = 0.05,
                   verbose: bool = False) -> dict:
    """
    Component mass breakdown (kg) for the powered glider.

    Parameters (SI): span, wing planform area, horizontal & vertical tail planform areas,
    overall fuselage length (nose to tail-boom end), battery and motor masses.
    construction : key of CONSTRUCTIONS ('recommended' == 'A1_printed_aero_foam_tail').

    Returns dict with component masses (kg), 'total', and an 'info' sub-dict
    (wing loading, spar choice, segment count, areal densities).
    """
    c = deepcopy(CONSTRUCTIONS[construction])
    b, S = span_m, wing_area_m2
    c_mean = S / b
    c_root = 2.0 * c_mean / (1.0 + taper_ratio)
    t_root = airfoil_tc * c_root
    t_tip = airfoil_tc * c_root * taper_ratio

    out = {}
    # ---------------- avionics & propulsion (fixed) ----------------
    out["flight_controller"] = AVIONICS[fc]
    out["receiver"] = AVIONICS[rx]
    out["servos"] = n_servos * AVIONICS[servo]
    out["gps"] = AVIONICS[gps]
    out["airspeed_sensor"] = AVIONICS[airspeed]
    out["esc"] = AVIONICS[esc]
    out["motor"] = motor_mass_kg
    out["prop_spinner"] = (AVIONICS["spinner_30mm_folding"] + AVIONICS["folding_blades_pair"]
                           if prop == "folding" else AVIONICS["fixed_prop_adapter"])
    # wiring: XT30 pair, power lead, servo extensions (span-dependent), velcro strap, heat-shrink (ASSUMED)
    out["wiring_connectors"] = 0.008 + 0.005 * b
    out["control_hardware"] = n_servos * 0.0015  # horns, 1 mm steel/CF pushrods, clevises (ASSUMED)
    out["battery"] = battery_mass_kg

    # ---------------- fuselage ----------------
    L = fuselage_length_m
    L_pod = min(max(0.38 * L, 0.28), 0.42)
    pod_wetted = math.pi * c["pod_diam"] * L_pod * 0.8
    out["fuselage_pod"] = pod_wetted * c["pod_material_areal"] + c["pod_fixed"]
    boom_len = max(L - 0.8 * L_pod, 0.2)
    boom_tube = ("CF tube 8x6", 8.0, 6.0) if boom_len > 0.50 else ("CF tube 6x4", 6.0, 4.0)
    out["tail_boom"] = boom_len * cf_linear_density(boom_tube[1], boom_tube[2])
    out["tail_mounts"] = c["tail_mount"]

    # ---------------- tails ----------------
    rho_tail = tail_areal_density(c)
    out["htail"] = htail_area_m2 * rho_tail + 0.001
    out["vtail"] = vtail_area_m2 * rho_tail + 0.001

    # ---------------- wing ----------------
    rho_wing = wing_areal_density(c, c_mean, airfoil_tc)
    out["wing_shell"] = rho_wing * S
    n_seg = 2 * math.ceil((b / 2.0) / c["seg_len"]) if c["wing_type"] in ("printed", "dbox_hybrid") else 2
    out["wing_joints"] = n_seg * c["seg_joint_mass"] if c["wing_type"] in ("printed", "dbox_hybrid") else 0.004
    # secondary 2 mm CF rod (anti-twist / segment alignment) full span
    out["wing_rear_spar"] = 0.97 * b * cf_linear_density(2.0)
    out["wing_hardware"] = 0.004  # 2x nylon wing bolts / magnets, servo covers (ASSUMED)

    # iterate main spar sizing on total weight
    out["wing_main_spar"] = 0.0
    out["glue"] = 0.0
    out["contingency"] = 0.0
    spar = None
    for _ in range(6):
        structure = sum(out[k] for k in ("fuselage_pod", "tail_boom", "tail_mounts", "htail", "vtail",
                                         "wing_shell", "wing_joints", "wing_rear_spar", "wing_main_spar"))
        out["glue"] = c["glue_frac"] * structure
        non_batt = sum(v for k, v in out.items() if k not in ("battery", "contingency", "motor"))
        out["contingency"] = contingency_frac * non_batt
        W = sum(out.values()) * G
        spar = _size_main_spar(W, b, t_root, t_tip, n_ult, sigma_allow_pa)
        out["wing_main_spar"] = spar["mass"]

    total = sum(out.values())
    wing_total = sum(out[k] for k in ("wing_shell", "wing_joints", "wing_rear_spar", "wing_main_spar", "wing_hardware"))
    out["total"] = total
    out["info"] = dict(
        construction=c["label"],
        mean_chord_m=c_mean,
        root_chord_m=c_root,
        aspect_ratio=b ** 2 / S,
        wing_shell_areal_density_g_dm2=rho_wing * 10.0,     # kg/m2 -> g/dm2  (x1000/100)
        wing_total_areal_density_g_dm2=wing_total / S * 10.0,
        tail_areal_density_g_dm2=rho_tail * 10.0,
        n_wing_segments=n_seg,
        main_spar=spar["inner"] + (f" + {spar['outer'].replace('CF tube ','')} outboard of {spar['eta_step']:.0%} semi-span"
                                   if spar["outer"] else ""),
        main_spar_sigma_MPa=spar["sigma_root"] / 1e6,
        root_moment_ult_Nm=spar["M_root"],
        tail_boom=boom_tube[0],
        boom_len_m=boom_len,
        pod_len_m=L_pod,
        wing_mass_kg=wing_total,
        wing_loading_g_dm2=total * 1000.0 / (S * 100.0),
        wing_loading_N_m2=total * G / S,
    )
    if verbose:
        print_breakdown(out)
    return out


def performance_hint(m: dict, rho: float = 1.09, CLmax: float = 1.2, bank_deg: float = 45.0) -> dict:
    """Quick look at stall speed and turn radius implied by the wing loading (for screening only)."""
    WS = m["info"]["wing_loading_N_m2"]
    Vs = math.sqrt(2 * WS / (rho * CLmax))
    n = 1 / math.cos(math.radians(bank_deg))
    V = 1.3 * Vs * math.sqrt(n)            # 30 % margin over the banked stall speed
    R = V ** 2 / (G * math.tan(math.radians(bank_deg)))
    return dict(Vs=Vs, V_turn=V, R_turn=R)


def print_breakdown(m: dict):
    print(f"  {m['info']['construction']}")
    for k, v in m.items():
        if k in ("info", "total"):
            continue
        print(f"    {k:22s} {v*1000:7.1f} g")
    print(f"    {'TOTAL':22s} {m['total']*1000:7.1f} g   WL = {m['info']['wing_loading_g_dm2']:.1f} g/dm2")


# ----------------------------------------------------------------------------
# 4. Example geometry generator (for tables only - final geometry comes from XFLR5 work)
# ----------------------------------------------------------------------------
def example_geometry(span, AR, Vh=0.45, Vv=0.028, fus_over_span=0.70, tail_arm_over_fus=0.62):
    S = span ** 2 / AR
    c = S / span
    L = fus_over_span * span
    lt = tail_arm_over_fus * L
    Sh = Vh * S * c / lt
    Sv = Vv * S * span / lt
    return dict(span=span, S=S, c=c, L=L, lt=lt, Sh=Sh, Sv=Sv)


ORDER = ["wing_shell", "wing_main_spar", "wing_rear_spar", "wing_joints", "wing_hardware",
         "fuselage_pod", "tail_boom", "tail_mounts", "htail", "vtail", "glue",
         "flight_controller", "receiver", "gps", "airspeed_sensor", "servos", "control_hardware",
         "esc", "motor", "prop_spinner", "wiring_connectors", "battery", "contingency"]



def calibration_check():
    """Re-create published LW-PLA aircraft with their (simpler) equipment and compare with
    the published take-off weights. Equipment for the references is ASSUMED (not published)."""
    rows = []
    # 3DLabPrint Bungee Prop: b 1.18 m, S 21.2 dm2, L 1.07 m, print 216 g LW-PLA, TOW 320 g w/ 2S 350
    m = mass_breakdown(1.18, 0.212, 0.030, 0.015, 1.07, 0.024, 0.021, construction="A3_printed_lwpla_all",
                       n_servos=2, servo="servo_emax_es9051", fc="fc_none", rx="rx_radiomaster_rp1",
                       gps="gps_none", esc="esc_12a_light", contingency_frac=0.0, prop="fixed_prop")
    rows.append(("3DLabPrint Bungee Prop (1.18 m, 21.2 dm2)", 320, m))
    # Planeprint Rise nano: b 1.02 m, WL 19 g/dm2 -> S ~12.6 dm2, 240 g, 2S, 3 ch (no ailerons)
    m = mass_breakdown(1.02, 0.126, 0.015, 0.010, 0.75, 0.020, 0.018, construction="A3_printed_lwpla_all",
                       n_servos=2, servo="servo_emax_es9051", fc="fc_none", rx="rx_radiomaster_rp1",
                       gps="gps_none", esc="esc_12a_light", contingency_frac=0.0, prop="fixed_prop")
    rows.append(("Planeprint Rise nano (1.02 m, ~12.6 dm2)", 240, m))
    return rows


def _examples_markdown():
    lines = []
    lines.append("# Example mass breakdowns (generated by mass_model.py)\n")
    lines.append("Assumptions for these examples: taper 0.6, airfoil t/c 9 %, fuselage length = 0.70 b, "
                 "tail arm = 0.62 L, V_h = 0.45, V_v = 0.028, 4 servos EMAX ES9052MD (5.5 g), "
                 "Matek F405-WTE (integrated ELRS), Matek M10Q GPS, 12-20 A ESC 12 g, 30 mm folding spinner, "
                 "motor 30 g (2205/2206 class) and battery 3S 450 mAh 42 g unless stated. "
                 "Spar sized at n_ult = 6 (4 g limit x 1.5), sigma_allow = 400 MPa. "
                 "Contingency 5 % of non-battery, non-motor mass. "
                 "Screening performance: rho = 1.09 kg/m3, CLmax = 1.2, turn at 45 deg bank with V = 1.3 Vs(banked).\n")
    # --- Table 1: recommended construction, spans x AR
    for AR in (10, 11, 12):
        lines.append(f"\n## Recommended construction (A1), AR = {AR}\n")
        spans = (1.0, 1.2, 1.4, 1.6)
        res = []
        for b in spans:
            g = example_geometry(b, AR)
            m = mass_breakdown(b, g["S"], g["Sh"], g["Sv"], g["L"], 0.042, 0.030)
            res.append((g, m, performance_hint(m)))
        hdr = "| Component (g) | " + " | ".join(f"b = {b:.1f} m" for b in spans) + " |"
        lines.append(hdr)
        lines.append("|---|" + "---|" * len(spans))
        geo_rows = [("Wing area S (dm2)", lambda g, m: f"{g['S']*100:.1f}"),
                    ("Mean chord (mm)", lambda g, m: f"{g['c']*1000:.0f}"),
                    ("Fuselage length (m)", lambda g, m: f"{g['L']:.2f}"),
                    ("H-tail / V-tail (dm2)", lambda g, m: f"{g['Sh']*100:.1f} / {g['Sv']*100:.1f}"),
                    ("Wing segments (printed)", lambda g, m: f"{m['info']['n_wing_segments']}"),
                    ("Main spar", lambda g, m: m['info']['main_spar'].replace('CF tube ', '') +
                     f" ({m['info']['main_spar_sigma_MPa']:.0f} MPa)")]
        for name, f in geo_rows:
            lines.append(f"| {name} | " + " | ".join(f(g, m) for g, m, _ in res) + " |")
        for k in ORDER:
            vals = [m[k] * 1000 for _, m, _ in res]
            if all(abs(v) < 1e-9 for v in vals):
                continue
            lines.append(f"| {k} | " + " | ".join(f"{v:.1f}" for v in vals) + " |")
        lines.append("| **Wing total** | " + " | ".join(f"**{m['info']['wing_mass_kg']*1000:.0f}**" for _, m, _ in res) + " |")
        lines.append("| **AUW** | " + " | ".join(f"**{m['total']*1000:.0f}**" for _, m, _ in res) + " |")
        lines.append("| Wing loading (g/dm2) | " + " | ".join(f"{m['info']['wing_loading_g_dm2']:.1f}" for _, m, _ in res) + " |")
        lines.append("| Vs @CLmax 1.2 (m/s) | " + " | ".join(f"{p['Vs']:.2f}" for *_, p in res) + " |")
        lines.append("| Turn radius @45 deg (m) | " + " | ".join(f"{p['R_turn']:.1f}" for *_, p in res) + " |")

    # --- Table 2: construction comparison at b = 1.4 m, AR 11
    lines.append("\n## Construction comparison at b = 1.4 m, AR = 11 (same avionics, 3S 450, 30 g motor)\n")
    g = example_geometry(1.4, 11)
    lines.append("| Construction | Wing shell g/dm2 | Wing total g/dm2 | Wing (g) | Tails (g) | AUW (g) | WL (g/dm2) | Vs (m/s) | R45 (m) |")
    lines.append("|---|---|---|---|---|---|---|---|---|")
    for key in ["A1_printed_aero_foam_tail", "A2_printed_aero_all", "A0_printed_aero_untuned",
                "A3_printed_lwpla_foam_tail", "C_dbox_hybrid", "B1_hotwire_xps", "B2_foamboard_folded",
                "X_sunboard_5mm"]:
        m = mass_breakdown(1.4, g["S"], g["Sh"], g["Sv"], g["L"], 0.042, 0.030, construction=key)
        p = performance_hint(m)
        i = m["info"]
        lines.append(f"| {i['construction']} | {i['wing_shell_areal_density_g_dm2']:.1f} | "
                     f"{i['wing_total_areal_density_g_dm2']:.1f} | {i['wing_mass_kg']*1000:.0f} | "
                     f"{(m['htail']+m['vtail'])*1000:.0f} | {m['total']*1000:.0f} | "
                     f"{i['wing_loading_g_dm2']:.1f} | {p['Vs']:.2f} | {p['R_turn']:.1f} |")

    # --- Calibration
    lines.append("\n## Calibration against published 3D-printed LW-PLA aircraft (reference equipment ASSUMED: "
                 "2 servos, RX only, no FC/GPS, fixed prop, no contingency)\n")
    lines.append("| Reference | Published TOW (g) | Model (g) | Model printed+spar structure (g) | Error |")
    lines.append("|---|---|---|---|---|")
    for name, ref, m in calibration_check():
        struct = sum(m[k] for k in ("wing_shell", "wing_joints", "wing_rear_spar", "wing_main_spar",
                                    "wing_hardware", "fuselage_pod", "tail_mounts", "htail", "vtail"))
        lines.append(f"| {name} | {ref} | {m['total']*1000:.0f} | {struct*1000:.0f} | "
                     f"{(m['total']*1000-ref)/ref*100:+.0f} % |")

    # --- Table 3: avionics sensitivity (light vs standard) at 1.4 m AR 11
    lines.append("\n## Avionics / battery sensitivity (recommended construction, b = 1.4 m, AR = 11)\n")
    lines.append("| Variant | AUW (g) | WL (g/dm2) | R45 (m) |")
    lines.append("|---|---|---|---|")
    variants = [
        ("Baseline: F405-WTE, M10Q GPS, 4x ES9052MD, 3S 450, 30 g motor", {}, 0.042, 0.030),
        ("Light: F411-WTE + RP1, no GPS, 4x ES9051, 2S 450, 21 g X2204", dict(fc="fc_matek_f411_wte", rx="rx_radiomaster_rp1", gps="gps_none", servo="servo_emax_es9051", esc="esc_12a_light"), 0.029, 0.021),
        ("+ airspeed sensor (for XFLR5-vs-flight)", dict(airspeed="airspeed_ms4525_pitot"), 0.042, 0.030),
        ("Heavy servos: 4x SG90 9 g", dict(servo="servo_towerpro_sg90"), 0.042, 0.030),
        ("Bigger pack: 3S 1000 mAh (85 g)", {}, 0.085, 0.030),
        ("Bigger motor: 2208 (44 g)", {}, 0.042, 0.044),
        ("3 servos instead of 4", dict(n_servos=3), 0.042, 0.030),
    ]
    for name, kw, batt, mot in variants:
        m = mass_breakdown(1.4, g["S"], g["Sh"], g["Sv"], g["L"], batt, mot, **kw)
        p = performance_hint(m)
        lines.append(f"| {name} | {m['total']*1000:.0f} | {m['info']['wing_loading_g_dm2']:.1f} | {p['R_turn']:.1f} |")
    return "\n".join(lines) + "\n"


def _plot(path_png):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    spans = np.linspace(0.9, 1.8, 19)
    keys = [("A1_printed_aero_foam_tail", "A1 PLA Aero printed + Depron tails (rec.)"),
            ("A2_printed_aero_all", "A2 all PLA Aero printed"),
            ("A0_printed_aero_untuned", "A0 PLA Aero untuned"),
            ("A3_printed_lwpla_foam_tail", "A3 LW-PLA printed"),
            ("C_dbox_hybrid", "C printed D-box + Depron"),
            ("B1_hotwire_xps", "B1 hot-wire XPS")]
    fig, ax = plt.subplots(1, 2, figsize=(11, 4.2))
    for k, lab in keys:
        auw, wl = [], []
        for b in spans:
            g = example_geometry(b, 11)
            m = mass_breakdown(b, g["S"], g["Sh"], g["Sv"], g["L"], 0.042, 0.030, construction=k)
            auw.append(m["total"] * 1000)
            wl.append(m["info"]["wing_loading_g_dm2"])
        ax[0].plot(spans, auw, label=lab)
        ax[1].plot(spans, wl, label=lab)
    ax[0].set_xlabel("Span (m), AR = 11")
    ax[0].set_ylabel("AUW (g)")
    ax[0].grid(alpha=0.3)
    ax[1].set_xlabel("Span (m), AR = 11")
    ax[1].set_ylabel("Wing loading (g/dm$^2$)")
    ax[1].axhspan(15, 22, color="green", alpha=0.08)
    ax[1].text(0.92, 15.5, "desirable: <= ~22 g/dm2 (45 deg turn radius <= ~8 m)", fontsize=8, color="green")
    ax[1].grid(alpha=0.3)
    ax[0].legend(fontsize=7)
    ax[0].set_title("All-up weight vs span")
    ax[1].set_title("Wing loading vs span")
    fig.tight_layout()
    fig.savefig(path_png, dpi=130)


if __name__ == "__main__":
    import os
    here = os.path.dirname(os.path.abspath(__file__))
    md = _examples_markdown()
    with open(os.path.join(here, "example_breakdowns.md"), "w") as f:
        f.write(md)
    print(md)
    _plot(os.path.join(here, "auw_wingloading_vs_span.png"))
    print("wrote example_breakdowns.md and auw_wingloading_vs_span.png")
