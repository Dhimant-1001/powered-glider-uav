"""
c_common.py -- shared model for Concept C "Compact & light" (EC4.402 powered glider).

Coordinate system (same as XFLR5 body axes): origin at the wing ROOT leading edge,
x aft, y right, z up.  All SI units.

Contents
  * Air / constants (rho 1.09, nu 1.7e-5)
  * PARAMS: design parameters (geometry, tail, construction, propulsion)
  * geometry(p): derived planform (chords, MAC, x_mac, tail sizes from V_H / V_v)
  * airplane(p, ...): AeroSandbox Airplane (wing panels, H-tail, fin, pod, boom, control surfaces)
  * mass_model(p): component list with (m, x, y, z) -> AUW, CG, inertias. Re-uses the stage-1
    areal-density formulas (stage1_copies/mass_model.py) and extends them with
    (a) component positions, (b) a printed-rib + film option, (c) explicit print density.
"""
from __future__ import annotations
import os, sys, math, copy
import numpy as np
import aerosandbox as asb

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "stage1_copies"))
import mass_model as s1m          # stage-1 mass model (copied, unmodified)

RHO = 1.09
RHO_HOT = 1.06
NU = 1.7e-5
MU = RHO * NU
G = 9.81


class SiteAtmosphere(asb.Atmosphere):
    """Hyderabad hot-day site air: rho = 1.09 kg/m^3, nu = 1.7e-5 m^2/s (mu = rho*nu)."""
    def __init__(self, rho=RHO, nu=NU):
        super().__init__(altitude=0.0, method="isa")
        self._rho, self._mu = rho, rho * nu

    def density(self):
        return self._rho

    def dynamic_viscosity(self):
        return self._mu


ATM = SiteAtmosphere()


def op(V, alpha=0.0, beta=0.0, p=0.0, q=0.0, r=0.0, rho=RHO):
    atm = ATM if rho == RHO else SiteAtmosphere(rho=rho)
    return asb.OperatingPoint(atmosphere=atm, velocity=V, alpha=alpha, beta=beta, p=p, q=q, r=r)


def foil(name):
    return asb.Airfoil(name=name, coordinates=os.path.join(HERE, "dat", name + ".dat"))


WING_FOIL = foil("sd7037_te07")
TAIL_FOIL = foil("flatplate-3pct")

# ----------------------------------------------------------------------------------------
# Design parameters (final values are set after the trade studies; see report.md)
# ----------------------------------------------------------------------------------------
PARAMS = dict(
    # --- wing ---
    b=1.35, AR=9.0,
    y_break=0.225,          # end of rectangular centre panel = first printed segment / polyhedral joint
    taper_outer=0.70,       # c_tip / c_root (outer trapezoid, straight quarter-chord line)
    dih_inner=0.0,          # deg, centre panel
    dih_outer=7.0,          # deg, outer panels (polyhedral)
    twist_root=0.0,         # deg relative to wing incidence
    twist_break=0.0,
    twist_tip=-2.0,         # washout
    i_w=2.0,                # wing incidence relative to the body x-axis (deg)
    ail_y0=0.250, ail_y1=0.650, ail_cf=0.25,
    # --- tail (conventional, flat 3 mm Depron) ---
    x_h_le=0.640,           # H-stab root LE x
    z_h=-0.012,
    VH=0.55, ARh=4.0, taper_h=0.80,
    i_t=-1.0,               # tail incidence relative to body x-axis (deg)
    elev_cf=0.40,
    Vv=0.030, ARv=1.5, taper_v=0.60,  # fin aspect ratio h^2/S_v
    rud_cf=0.40,
    # --- fuselage ---
    x_nose=-0.235, x_pod_end=0.15, pod_w=0.042, pod_h=0.050, z_pod=-0.030,
    boom_od=0.008, boom_id=0.006, z_boom=-0.012,
    # --- CG design target ---
    SM_target=0.11,
    # --- construction / materials ---
    construction="C2_dbox_depron",   # C1_full_shell | C2_dbox_depron | C3_dbox_film
    rho_print=750.0,                 # kg/m^3 printed PLA Aero (design 0.75 g/cm^3)
    # --- propulsion / avionics masses (kg) and positions (filled from s02 propulsion trade) ---
    m_motor=0.050, m_prop=0.018, m_esc=0.010, m_batt=0.059, batt_name="Tattu 3S 650 mAh 75C",
    batt_len=0.058,
    m_fc=0.019, m_rx=0.002, m_gps=0.008, m_aspd=0.007, m_servo=0.0055, n_servo=4,
    x_batt=None,             # battery CG; None = centre of the bay. Bay = [x_nose+0.075, +0.110]
    m_ballast=0.0, x_ballast=-0.215,
    contingency=0.05,
)


def wing_planform(p):
    b, AR = p["b"], p["AR"]
    S = b * b / AR
    s = b / 2
    yb = p["y_break"]
    lam = p["taper_outer"]
    cr = (S / 2) / (yb + (s - yb) * (1 + lam) / 2)
    ct = lam * cr
    # MAC by integration (L11 eq. 8)
    y = np.linspace(0, s, 4001)
    c = np.where(y <= yb, cr, cr + (ct - cr) * (y - yb) / (s - yb))
    xle = np.where(y <= yb, 0.0, (cr - c) / 4)        # straight quarter-chord line on the outer panel
    mac = 2 / S * np.trapezoid(c ** 2, y)
    ymac = 2 / S * np.trapezoid(c * y, y)
    xle_mac = 2 / S * np.trapezoid(c * xle, y)
    # chord-weighted LE x at the MAC station (equivalent wing)
    return dict(S=S, s=s, cr=cr, ct=ct, mac=mac, y_mac=ymac, x_le_mac=xle_mac,
                x_ac_est=xle_mac + 0.25 * mac)


def tail_sizes(p, x_cg):
    """Size H-tail and fin from tail volume coefficients (L14 eq. 11 and 20)."""
    w = wing_planform(p)
    # first guess of the tail chord to locate its quarter chord
    Sh = p.get("Sh_fixed") or 0.03
    for _ in range(20):
        bh = math.sqrt(p["ARh"] * Sh)
        crh = 2 * Sh / (bh * (1 + p["taper_h"]))
        x_ac_h = p["x_h_le"] + 0.25 * crh
        lt = x_ac_h - x_cg
        Sh_new = p["VH"] * w["S"] * w["mac"] / lt if not p.get("Sh_fixed") else p["Sh_fixed"]
        if abs(Sh_new - Sh) < 1e-7:
            break
        Sh = Sh_new
    Sv = p.get("Sv_fixed") or 0.012
    for _ in range(20):
        hv = math.sqrt(p["ARv"] * Sv)
        crv = 2 * Sv / (hv * (1 + p["taper_v"]))
        x_ac_v = p["x_h_le"] + 0.02 + 0.25 * crv + 0.4 * hv * 0.15   # small LE sweep of the fin (see airplane())
        lv = x_ac_v - x_cg
        Sv_new = p["Vv"] * w["S"] * p["b"] / lv if not p.get("Sv_fixed") else p["Sv_fixed"]
        if abs(Sv_new - Sv) < 1e-7:
            break
        Sv = Sv_new
    return dict(Sh=Sh, bh=bh, crh=crh, cth=crh * p["taper_h"], x_ac_h=x_ac_h, lt=lt,
                Sv=Sv, hv=hv, crv=crv, ctv=crv * p["taper_v"], x_ac_v=x_ac_v, lv=lv)


def geometry(p, x_cg=None):
    w = wing_planform(p)
    if x_cg is None:
        x_cg = w["x_le_mac"] + (0.25 - p["SM_target"] + 0.10) * w["mac"]   # rough first guess
    t = tail_sizes(p, x_cg)
    g = dict(w); g.update(t); g["x_cg_used"] = x_cg
    g["VH"] = t["Sh"] * t["lt"] / (w["S"] * w["mac"])
    g["Vv"] = t["Sv"] * t["lv"] / (w["S"] * p["b"])
    g["x_tail_end"] = p["x_h_le"] + max(t["crh"], t["crv"] + 0.02 + 0.15 * t["hv"])
    g["L_fus"] = g["x_tail_end"] - p["x_nose"]
    g["boom_len"] = g["x_tail_end"] - (p["x_pod_end"] - 0.03)   # 30 mm inside the pod socket
    return g


# ----------------------------------------------------------------------------------------
# AeroSandbox airplane
# ----------------------------------------------------------------------------------------
def _wing_station(p, g, y):
    """LE position, chord, twist at spanwise station y (outer panel straight)."""
    s, yb = g["s"], p["y_break"]
    if y <= yb + 1e-9:
        c = g["cr"]; xle = 0.0
        z = y * math.tan(math.radians(p["dih_inner"]))
        tw = p["twist_root"] + (p["twist_break"] - p["twist_root"]) * y / yb
    else:
        f = (y - yb) / (s - yb)
        c = g["cr"] + (g["ct"] - g["cr"]) * f
        xle = (g["cr"] - c) / 4
        z = yb * math.tan(math.radians(p["dih_inner"])) + (y - yb) * math.tan(math.radians(p["dih_outer"]))
        tw = p["twist_break"] + (p["twist_tip"] - p["twist_break"]) * f
    return xle, z, c, tw


def wing_stations(p, g):
    return [0.0, p["y_break"], p["ail_y0"], p["ail_y1"], g["s"]]


def airplane(p, g=None, delta_e=0.0, delta_a=0.0, delta_r=0.0, spoileron=0.0, x_ref=None,
             fuselage=True, i_t=None, tail="conventional", name="ConceptC"):
    """delta_a: right-aileron-down positive (asb antisymmetric convention), degrees.
    spoileron: symmetric aileron deflection (negative = trailing edge up)."""
    if g is None:
        g = geometry(p)
    i_t = p["i_t"] if i_t is None else i_t
    xs = []
    ys = sorted(set([0.0, p["y_break"], p["ail_y0"], p["ail_y1"], g["s"]]))
    for k, y in enumerate(ys):
        xle, z, c, tw = _wing_station(p, g, y)
        cs = []
        if abs(y - p["ail_y0"]) < 1e-9:
            if delta_a != 0.0:
                cs.append(asb.ControlSurface(name="aileron", symmetric=False, deflection=delta_a,
                                             hinge_point=1 - p["ail_cf"]))
            if spoileron != 0.0:
                cs.append(asb.ControlSurface(name="spoileron", symmetric=True, deflection=spoileron,
                                             hinge_point=1 - p["ail_cf"]))
        # rotate the section about its quarter chord: XFLR5 rotates about LE? -> asb twists about LE.
        xs.append(asb.WingXSec(xyz_le=[xle, y, z], chord=c, twist=p["i_w"] + tw, airfoil=WING_FOIL,
                               control_surfaces=cs))
    wing = asb.Wing(name="Main wing", symmetric=True, xsecs=xs)

    wings = [wing]
    if tail == "conventional":
        hx = []
        for y, c in ((0.0, g["crh"]), (g["bh"] / 2, g["cth"])):
            hx.append(asb.WingXSec(xyz_le=[p["x_h_le"] + (g["crh"] - c) / 4, y, p["z_h"]], chord=c, twist=i_t,
                                   airfoil=TAIL_FOIL,
                                   control_surfaces=[asb.ControlSurface(name="elevator", symmetric=True,
                                                                        deflection=delta_e,
                                                                        hinge_point=1 - p["elev_cf"])] if y == 0.0 else []))
        wings.append(asb.Wing(name="Elevator", symmetric=True, xsecs=hx))
        vx = []
        x0 = p["x_h_le"] + 0.02
        for zz, c in ((0.0, g["crv"]), (g["hv"], g["ctv"])):
            vx.append(asb.WingXSec(xyz_le=[x0 + 0.15 * zz + (g["crv"] - c) * 0.0, 0.0, p["z_h"] + 0.004 + zz], chord=c,
                                   twist=0.0, airfoil=TAIL_FOIL,
                                   control_surfaces=[asb.ControlSurface(name="rudder", symmetric=True,
                                                                        deflection=delta_r,
                                                                        hinge_point=1 - p["rud_cf"])] if zz == 0.0 else []))
        wings.append(asb.Wing(name="Fin", symmetric=False, xsecs=vx).subdivide_sections(2))
    elif tail == "vtail":
        # equal total planform area, dihedral angle chosen by the V-tail trade (s03)
        Sv_tot = p["S_vtail"]; G_v = math.radians(p["vtail_dih"])
        bv = math.sqrt(p["ARh"] * Sv_tot)            # total developed span of both panels
        cr = 2 * Sv_tot / (bv * (1 + p["taper_h"])); ct = cr * p["taper_h"]
        half = bv / 2
        vx = []
        for f, c in ((0.0, cr), (1.0, ct)):
            vx.append(asb.WingXSec(xyz_le=[p["x_h_le"] + (cr - c) / 4, f * half * math.cos(G_v),
                                           p["z_h"] + f * half * math.sin(G_v)], chord=c, twist=i_t,
                                   airfoil=TAIL_FOIL,
                                   control_surfaces=[asb.ControlSurface(name="ruddervator", symmetric=True,
                                                                        deflection=delta_e,
                                                                        hinge_point=1 - p["elev_cf"])] if f == 0.0 else []))
        wings.append(asb.Wing(name="V-tail", symmetric=True, xsecs=vx))

    fus = []
    if fuselage:
        # pod: rounded nose (spinner 32 mm), max section 42 x 50 mm, tapering into the boom
        xn, xe, W, H, zc = p["x_nose"], p["x_pod_end"], p["pod_w"], p["pod_h"], p["z_pod"]
        stations = [(xn, 0.016, 0.016), (xn + 0.03, 0.034, 0.038), (xn + 0.08, W, H), (0.085, W, H),
                    (xe - 0.02, 0.024, 0.026), (xe, 0.012, 0.012)]
        fx = [asb.FuselageXSec(xyz_c=[x, 0, zc + (p["z_boom"] - zc) * max(0, (x - 0.085) / (xe - 0.085))],
                               width=wd, height=ht, shape=2.5) for x, wd, ht in stations]
        fus.append(asb.Fuselage(name="Pod", xsecs=fx))
        bx = [asb.FuselageXSec(xyz_c=[x, 0, p["z_boom"]], radius=p["boom_od"] / 2)
              for x in (xe, g["x_tail_end"])]
        fus.append(asb.Fuselage(name="Boom", xsecs=bx))
    if x_ref is None:
        x_ref = g["x_cg_used"]
    return asb.Airplane(name=name, xyz_ref=[x_ref, 0, 0], wings=wings, fuselages=fus,
                        s_ref=g["S"], c_ref=g["mac"], b_ref=p["b"])


# ----------------------------------------------------------------------------------------
# Mass model with positions
# ----------------------------------------------------------------------------------------
RHO_DEPRON = 40.0     # kg/m^3, 3 mm Depron datasheet (stage-1 source)
FILM_AREAL = 0.043    # kg/m^2, Oracover Oralight opaque white incl. adhesive (Modelemax/Digitec); transparent 0.036


def wing_shell_areal(p, c_mean, tc=0.092):
    """kg per m^2 planform. Stage-1 formulas for C1/C2 with explicit print density;
    C2 ribs and C3 are computed from rib geometry instead of the stage-1 0.25 kg/m^2 lump."""
    rho = p["rho_print"]
    wall = 0.45e-3
    con = p["construction"]
    # printed rib: 1.2 mm thick plate, 40 % solid after lightening holes, aft 75 % of section area
    rib_mass = 0.40 * 0.75 * 0.68 * tc * c_mean ** 2 * 1.2e-3 * rho
    if con == "C1_full_shell":
        return 2.04 * wall * rho * 1.35                         # stage-1 A1 (vase shell + internal webs)
    if con == "C2_dbox_depron":
        dbox = 0.68 * wall * rho * 1.2                          # stage-1 D-box (0-30 % c)
        skins = 1.44 * 3e-3 * RHO_DEPRON                        # stage-1: 3 mm Depron aft skins
        ribs = rib_mass / (c_mean * 0.060)                      # ribs every 60 mm
        te = 1.0e-3 * 8e-3 * rho * 2 / c_mean                   # printed TE strip 8 x 1 mm (both sides of hinge)
        return dbox + skins + ribs + te
    if con == "C3_dbox_film":
        dbox = 0.68 * wall * rho * 1.2
        ribs = rib_mass / (c_mean * 0.045)                      # closer ribs (film sags) every 45 mm
        caps = 2 * 0.75 * 2.0e-3 * 1.0e-3 * rho / 0.045 * 0     # (rib caps included in rib plate)
        film = 1.50 * FILM_AREAL                                # aft 75 % top + bottom
        te = 1.0e-3 * 10e-3 * rho * 2 / c_mean
        return dbox + ribs + film + te + caps
    raise ValueError(con)


def batt_bay(p):
    """Battery CG travel limits (pack fully inside the bay)."""
    return (p["x_nose"] + 0.075 + p["batt_len"] / 2, 0.110 - p["batt_len"] / 2)


def batt_x(p):
    lo, hi = batt_bay(p)
    return 0.5 * (lo + hi) if p.get("x_batt") is None else p["x_batt"]


def _spar(p, g, W_N):
    w = wing_planform(p)
    t_root = 0.092 * w["cr"]; t_tip = 0.092 * w["ct"]
    return s1m._size_main_spar(W_N, p["b"], t_root, t_tip, 6.0, 400e6)


def mass_model(p, g=None):
    """Returns dict(items=[(name, m, x, y, z, kind)], m, cg, I) ; y used for inertia only
    (mirrored items listed per side)."""
    if g is None:
        g = geometry(p)
    rho = p["rho_print"]
    w = wing_planform(p)
    items = []

    def add(n, m, x, y=0.0, z=0.0, kind="fixed"):
        items.append([n, float(m), float(x), float(y), float(z), kind])

    # ---- propulsion / avionics ----
    add("motor", p["m_motor"], p["x_nose"] + 0.012, 0, p["z_pod"], "prop")
    add("prop+spinner (folding)", p["m_prop"], p["x_nose"] - 0.005, 0, p["z_pod"], "prop")
    add("ESC", p["m_esc"], p["x_nose"] + 0.055, 0, p["z_pod"], "avionics")
    add("battery " + p["batt_name"], p["m_batt"], batt_x(p), 0, p["z_pod"] - 0.005, "battery")
    add("flight controller + PDB (shelf above battery)", p["m_fc"], p["x_nose"] + 0.105, 0, p["z_pod"] + 0.012, "avionics")
    add("receiver FS-A8S + antenna", p["m_rx"], 0.095, 0, p["z_pod"] + 0.01, "avionics")
    add("GPS M10Q-5883 (cradle on boom)", p["m_gps"], 0.200, 0, p["z_boom"] + 0.012, "avionics")
    add("airspeed ASPD-4525 + pitot", p["m_aspd"], 0.000, 0, -0.01, "avionics")
    for side in (-1, 1):
        add("aileron servo", p["m_servo"], 0.055, side * 0.27, 0.225 * 0 + (0.27 - p["y_break"]) * math.tan(math.radians(p["dih_outer"])), "avionics")
    add("elevator servo (tail, on boom)", p["m_servo"], p["x_h_le"] - 0.06, 0.007, p["z_boom"] + 0.01, "avionics")
    add("rudder servo (tail, on boom)", p["m_servo"], p["x_h_le"] - 0.06, -0.007, p["z_boom"] + 0.01, "avionics")
    add("tail servo leads along boom (2 x 0.55 m, 2.5 g/m assumed)", 2 * 0.55 * 0.0025, 0.38, 0, p["z_boom"], "avionics")
    add("control hardware (ailerons)", 2 * 0.0015, 0.10, 0, 0.0, "avionics")
    add("control hardware (tail: horns + 80 mm pushrods)", 2 * 0.0015 + 2 * 0.08 * 1.2e-3, p["x_h_le"] + 0.01, 0, p["z_boom"], "avionics")
    add("wiring, XT30, servo extensions", 0.008 + 0.005 * p["b"], -0.05, 0, p["z_pod"], "avionics")

    # ---- fuselage ----
    L_pod = p["x_pod_end"] - p["x_nose"]
    pod_wet = math.pi * 0.5 * (p["pod_w"] + p["pod_h"]) * L_pod * 0.8
    add("pod shell (PLA Aero 0.8 mm)", pod_wet * 0.8e-3 * rho, p["x_nose"] + 0.45 * L_pod, 0, p["z_pod"], "structure")
    add("motor mount (PETG)", 0.006, p["x_nose"] + 0.022, 0, p["z_pod"], "structure")
    add("pod fixed: saddle, bolts, hatch magnets, boom socket", 0.013, 0.02, 0, p["z_pod"], "structure")
    lin = s1m.cf_linear_density(p["boom_od"] * 1e3, p["boom_id"] * 1e3)
    x0b = p["x_pod_end"] - 0.03
    add(f"tail boom CF {p['boom_od']*1e3:.0f}x{p['boom_id']*1e3:.0f}", lin * g["boom_len"], x0b + g["boom_len"] / 2, 0, p["z_boom"], "structure")
    add("tail mounts (printed)", 0.008, p["x_h_le"] + 0.03, 0, p["z_boom"], "structure")
    tail_areal = 3e-3 * RHO_DEPRON + 0.06                      # stage-1: Depron + CF strip/tape
    add("H-stab 3 mm Depron", g["Sh"] * tail_areal + 0.001, p["x_h_le"] + 0.42 * g["crh"], 0, p["z_h"], "structure")
    add("fin 3 mm Depron", g["Sv"] * tail_areal + 0.001, p["x_h_le"] + 0.02 + 0.42 * g["crv"] + 0.06 * g["hv"], 0,
        p["z_h"] + 0.4 * g["hv"], "structure")

    # ---- wing (distributed: 12 strips per side) ----
    c_mean = w["S"] / p["b"]
    areal = wing_shell_areal(p, c_mean)
    ny = 12
    edges = np.linspace(0, w["s"], ny + 1)
    shell_tot = 0.0
    for side in (-1, 1):
        for a, b_ in zip(edges[:-1], edges[1:]):
            ym = 0.5 * (a + b_)
            xle, z, c, tw = _wing_station(p, g, ym)
            m = areal * c * (b_ - a)
            shell_tot += m
            add("wing shell strip", m, xle + 0.42 * c, side * ym, z, "wingshell")
    n_seg = 2 * math.ceil(w["s"] / 0.240)
    add("wing segment joints/keys", n_seg * 0.0015, 0.30 * w["cr"], 0, 0.0, "structure")
    add("polyhedral joints (PETG keys, 2x)", 0.006, 0.30 * w["cr"], 0, 0.0, "structure")
    add("rear CF rod 2 mm", 0.97 * p["b"] * s1m.cf_linear_density(2.0), 0.65 * c_mean, 0, 0.01, "structure")
    add("wing hardware (bands/bolts, servo covers)", 0.004, 0.3 * w["cr"], 0, 0.0, "structure")
    # main spar sized at n_ult 6 on the converged weight (iterate)
    spar_idx = len(items)
    add("main spar (stage-1 stepped CF sizing)", 0.02, 0.27 * c_mean, 0, 0.012, "structure")
    add("glue (4 % structure)", 0.0, 0.0, 0, 0, "glue")
    add("contingency (5 % non-battery non-motor)", 0.0, 0.0, 0, 0, "cont")
    add("nose ballast", p["m_ballast"], p["x_nose"] + 0.03, 0, p["z_pod"], "ballast")
    spar = None
    for _ in range(8):
        struct = sum(it[1] for it in items if it[5] in ("structure", "wingshell"))
        items[spar_idx + 1][1] = 0.04 * struct
        non = sum(it[1] for it in items if it[5] not in ("battery", "prop", "cont", "ballast"))
        items[spar_idx + 2][1] = p["contingency"] * (non + p["m_esc"] * 0)
        Wt = sum(it[1] for it in items) * G
        spar = _spar(p, g, Wt)
        items[spar_idx][1] = spar["mass"]
    # glue & contingency located at the structure / airframe CG
    def cg_of(kinds):
        sel = [it for it in items if it[5] in kinds]
        m = sum(it[1] for it in sel)
        return sum(it[1] * it[2] for it in sel) / m, sum(it[1] * it[4] for it in sel) / m
    items[spar_idx + 1][2], items[spar_idx + 1][4] = cg_of(("structure", "wingshell"))
    items[spar_idx + 2][2], items[spar_idx + 2][4] = cg_of(("structure", "wingshell", "avionics"))

    M = sum(it[1] for it in items)
    cg = np.array([sum(it[1] * it[k] for it in items) / M for k in (2, 3, 4)])
    Ixx = Iyy = Izz = Ixz = 0.0
    for n, m, x, y, z, kind in items:
        dx, dy, dz = x - cg[0], y - cg[1], z - cg[2]
        Ixx += m * (dy * dy + dz * dz); Iyy += m * (dx * dx + dz * dz); Izz += m * (dx * dx + dy * dy); Ixz += m * dx * dz
    # distributed-strip self inertia (strip chord) and boom rod self-inertia
    for n, m, x, y, z, kind in items:
        if kind == "wingshell":
            Iyy += m * (c_mean ** 2) / 12; Izz += m * (c_mean ** 2) / 12
        if n.startswith("tail boom"):
            Iyy += m * g["boom_len"] ** 2 / 12; Izz += m * g["boom_len"] ** 2 / 12
        if n.startswith("main spar") or n.startswith("rear CF"):
            Ixx += m * p["b"] ** 2 / 12; Izz += m * p["b"] ** 2 / 12
    wing_total = shell_tot + sum(it[1] for it in items if it[0].startswith(("wing segment", "polyhedral", "rear CF", "wing hardware", "main spar")))
    return dict(items=items, m=M, cg=cg, Ixx=Ixx, Iyy=Iyy, Izz=Izz, Ixz=Ixz, spar=spar,
                wing_areal=areal, wing_mass=wing_total, WS=M * G / w["S"], n_seg=n_seg)


def group_masses(mm):
    """Collapse the item list into a readable breakdown (g)."""
    out = {}
    for n, m, x, y, z, k in mm["items"]:
        key = "wing shell" if n == "wing shell strip" else ("aileron servos" if n == "aileron servo" else n)
        out.setdefault(key, [0.0, 0.0])
        out[key][0] += m; out[key][1] += m * x
    return {k: (v[0] * 1000, (v[1] / v[0] if v[0] > 0 else 0.0)) for k, v in out.items()}
