"""
final_analysis.py -- FINAL design of the 1.40 m hand-launched powered glider (EC4.402).
One script: sizing loop -> mass/CG -> VLM neutral point & trim -> AeroBuildup drag polar ->
critical-section CLmax -> operating points -> turn/climb/glide/roll -> lateral checks ->
spar + print segments -> propulsion/endurance -> plots + design_spec.md.

Reuses: 06_concepts/C_compact_light/c_common.py (geometry, asb airplane, positioned mass model,
which itself wraps 02_construction/mass_model.py) and 04_propulsion/propulsion_model.py (UIUC-calibrated
motor/prop model). Axes: origin at wing ROOT LE, x aft, y right, z up. SI units.
Run: /opt/anaconda3/bin/python3 final_analysis.py
"""
import os, sys, math, json, copy, warnings
import numpy as np
warnings.filterwarnings("ignore")
HERE = os.path.dirname(os.path.abspath(__file__))
D = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(D, "06_concepts", "C_compact_light"))
sys.path.insert(0, os.path.join(D, "04_propulsion"))
import aerosandbox as asb
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
import c_common as cc
import propulsion_model as pm

RHO, NU, G = 1.09, 1.7e-5, 9.81
DAT = os.path.join(D, "03_airfoil", "dat")
cc.WING_FOIL = asb.Airfoil(name="sd7037_te05", coordinates=os.path.join(DAT, "sd7037_te05.dat"))
cc.TAIL_FOIL = asb.Airfoil(name="flatplate-3pct", coordinates=os.path.join(DAT, "flatplate-3pct.dat"))
VLM_KW = dict(spanwise_resolution=6, chordwise_resolution=8)
CD_MISC = 0.003      # excrescences not in AeroBuildup: pitot, servo horns/linkages, hinge gaps, skid, seams
OUT = {}

# ------------------------------------------------------------------ design parameters
P = copy.deepcopy(cc.PARAMS)
P.update(
    b=1.40, AR=10.0, y_break=0.240, taper_outer=0.72, dih_inner=0.0, dih_outer=6.0,
    twist_root=0.0, twist_break=0.0, twist_tip=-2.0, i_w=2.0,
    ail_y0=0.250, ail_y1=0.660, ail_cf=0.25,
    x_h_le=0.700, z_h=-0.012, VH=0.55, ARh=4.0, taper_h=0.80, i_t=-1.0, elev_cf=0.40,
    Vv=0.030, ARv=1.5, taper_v=0.60, rud_cf=0.40,
    x_nose=-0.195, x_pod_end=0.15, pod_w=0.042, pod_h=0.050, z_pod=-0.030,
    boom_od=0.008, boom_id=0.006, z_boom=-0.012,
    SM_target=0.12, construction="C1_full_shell", rho_print=750.0,
    m_motor=0.050, m_prop=0.018, m_esc=0.019,
    m_fc=0.023, m_rx=0.002, m_gps=0.008, m_aspd=0.007, m_servo=0.005, n_servo=4,
    m_ballast=0.0, contingency=0.05,
)
def spar_polyhedral(p, g, W_N, n_ult=6.0):
    """Main spar matched to the polyhedral break (a straight tube cannot cross it):
    6x4 CF centre tube y = +-y_break (one piece through the pod), 5x3 CF outer tubes y_break..0.97 s,
    joined by 3 mm music-wire joiners pre-bent to the polyhedral angle (100 mm; 50 mm into a 4x3 brass
    sleeve bonded in the 6x4 end, 50 mm into the 5x3 bore)."""
    m1 = cc.s1m
    s, yb = p["b"] / 2, p["y_break"]
    M0 = n_ult * (W_N / 2) * s * m1.elliptic_moment_fraction(0.0)
    Mb = n_ult * (W_N / 2) * s * m1.elliptic_moment_fraction(yb / s)
    sig = lambda M, od, idd: M * od / 2e3 / m1.cf_I(od, idd)
    L_out = 0.97 * s - yb
    mass = 2 * yb * m1.cf_linear_density(6, 4) + 2 * L_out * m1.cf_linear_density(5, 3) \
        + 2 * 0.10 * math.pi / 4 * 9e-6 * 7850 + 2 * 0.05 * math.pi / 4 * 7e-6 * 8500
    return dict(mass=mass, inner="CF tube 6x4", outer="CF tube 5x3", L_in=2 * yb, L_out=L_out, eta_step=yb / s,
                sigma_root=sig(M0, 6, 4), sigma_step=sig(Mb, 5, 3), sigma_wire=Mb * 1.5e-3 / (math.pi / 64 * 3e-3 ** 4),
                M_root=M0, M_break=Mb)
cc._spar = spar_polyhedral

BATTS = {  # name: (mass kg, length m, pm.Battery)
    "3S 650 mAh": (0.059, 0.058, pm.Battery(3, 650, 59, 45, source="3S 650 mAh 45-75C, ~55-60 g")),
    "3S 850 mAh": (0.072, 0.065, pm.Battery(3, 850, 72, 30, source="Orange 3S 850 30C ~70-75 g")),
}


def set_batt(p, name):
    p["m_batt"], p["batt_len"], _ = BATTS[name]; p["batt_name"] = name


def plane(p, g, **kw):
    return cc.airplane(p, g, **kw)


def vlm(ap, V, a, beta=0.0, p_=0.0, r_=0.0, stab=False):
    v = asb.VortexLatticeMethod(airplane=ap, op_point=cc.op(V, a, beta, p_, 0.0, r_),
                                xyz_ref=list(ap.xyz_ref), **VLM_KW)
    return (v.run_with_stability_derivatives() if stab else v.run()), v


def ab(ap, V, a, stab=False):
    o = asb.AeroBuildup(airplane=ap, op_point=cc.op(V, a), xyz_ref=list(ap.xyz_ref))
    return o.run_with_stability_derivatives(alpha=True, beta=True, p=True, q=False, r=True) if stab else o.run()


def neutral_point(p, g, V=9.0):
    ap = plane(p, g, fuselage=False, x_ref=0.0)
    r1, _ = vlm(ap, V, 2.0); r2, _ = vlm(ap, V, 4.0)
    CLa = (r2["CL"] - r1["CL"]) / 2.0; Cma = (r2["Cm"] - r1["Cm"]) / 2.0
    x_np_vlm = -Cma / CLa * g["mac"]
    # fuselage (pod+boom) destabilising increment from AeroBuildup (with - without fuselage)
    d = []
    for fus in (True, False):
        a1 = ab(plane(p, g, fuselage=fus, x_ref=0.0), V, 2.0); a2 = ab(plane(p, g, fuselage=fus, x_ref=0.0), V, 4.0)
        d.append(float(np.ravel(a2["Cm"] - a1["Cm"])[0]) / 2.0)
    dCma_fus = d[0] - d[1]
    return dict(x_np=x_np_vlm - dCma_fus / CLa * g["mac"], x_np_vlm=x_np_vlm, CLa_deg=CLa, dCma_fus=dCma_fus)


def size(p, verbose=True):
    """Sizing loop: tails from V_H/V_v at current CG -> masses -> NP -> battery x for SM target."""
    x_cg = None
    for it in range(6):
        g = cc.geometry(p, x_cg)
        mm = cc.mass_model(p, g)
        npd = neutral_point(p, g)
        x_tgt = npd["x_np"] - p["SM_target"] * g["mac"]
        xb = cc.batt_x(p) + (x_tgt - mm["cg"][0]) * mm["m"] / p["m_batt"]
        lo, hi = cc.batt_bay(p)
        p["x_batt"] = float(np.clip(xb, lo, hi))
        mm = cc.mass_model(p, g)
        if x_cg is not None and abs(mm["cg"][0] - x_cg) < 2e-4:
            break
        x_cg = mm["cg"][0]
    SM = (npd["x_np"] - mm["cg"][0]) / g["mac"]
    if verbose:
        print(f"  sized: AUW {mm['m']*1e3:.0f} g  x_cg {mm['cg'][0]*1e3:.1f} mm  x_np {npd['x_np']*1e3:.1f} mm  "
              f"SM {SM*100:.1f}%  batt x {p['x_batt']*1e3:.0f} (bay {lo*1e3:.0f}..{hi*1e3:.0f})  Sh {g['Sh']*1e4:.0f} cm2  Sv {g['Sv']*1e4:.0f} cm2")
    return g, mm, npd, SM


# ================================================================== 1. sizing for both batteries
print("== 1. sizing loop")
designs = {}
for bn in BATTS:
    p = copy.deepcopy(P); set_batt(p, bn); p["x_batt"] = None
    print(" battery", bn)
    designs[bn] = (p,) + size(p)

# ================================================================== 2. aerodynamics (650 geometry; geometry identical for 850)
BN0 = "3S 650 mAh"
p, g, mm, npd, SM = designs[BN0]
S, b, mac = g["S"], p["b"], g["mac"]
W = mm["m"] * G
x_cg = mm["cg"][0]


def vsp(CL, Wt=None):
    return math.sqrt(2 * (Wt or W) / (RHO * S * CL))


def strips(v, ap):
    nc = v.chordwise_resolution
    w0 = ap.wings[0].subdivide_sections(ratio=v.spanwise_resolution, spacing_function=v.spanwise_spacing_function)
    _, faces = w0.mesh_thin_surface(method="quad", chordwise_resolution=nc,
                                    chordwise_spacing_function=v.chordwise_spacing_function, add_camber=True)
    n = len(faces)
    F = np.asarray(v.forces_geometry)[:n].reshape(-1, nc, 3).sum(axis=1)
    fl = np.asarray(v.front_left_vertices)[:n].reshape(-1, nc, 3); fr = np.asarray(v.front_right_vertices)[:n].reshape(-1, nc, 3)
    bl = np.asarray(v.back_left_vertices)[:n].reshape(-1, nc, 3); br = np.asarray(v.back_right_vertices)[:n].reshape(-1, nc, 3)
    y = 0.25 * (fl[:, 0, 1] + fr[:, 0, 1] + bl[:, -1, 1] + br[:, -1, 1])
    dy = np.hypot(fr[:, 0, 1] - fl[:, 0, 1], fr[:, 0, 2] - fl[:, 0, 2])
    c = 0.5 * ((bl[:, -1, 0] - fl[:, 0, 0]) + (br[:, -1, 0] - fr[:, 0, 0]))
    a = math.radians(v.op_point.alpha)
    ld = np.array([-math.sin(a), 0.0, math.cos(a)])
    cl = (F @ ld) / (v.op_point.dynamic_pressure() * c * dy)
    k = y > 0
    o = np.argsort(y[k])
    return y[k][o], c[k][o], cl[k][o]


print("== 2. aero")
# --- tail incidence: zero elevator trim at the chosen operating CL (iterated once below)
CL_OP_GUESS = 0.70
TAU_E = 0.65   # elevator effectiveness, 40 % chord flat plate: thin-airfoil 0.75 x 0.87 (gap, low Re); AeroBuildup gives 0.84
def trim_map(p, g):
    ap = plane(p, g, fuselage=False, x_ref=x_cg)
    al = np.arange(-4, 13.01, 1.0)
    CL0, Cm0 = [], []
    for a in al:
        r, _ = vlm(ap, 9.0, a); CL0.append(r["CL"]); Cm0.append(r["Cm"])
    r0, _ = vlm(ap, 9.0, 4.0)
    # asb 4.2.10 VLM ignores control-surface deflections -> elevator = tau_e x tail-incidence change
    rd, _ = vlm(plane(p, g, fuselage=False, x_ref=x_cg, i_t=p["i_t"] + 1.0), 9.0, 4.0)
    Cmde = TAU_E * (rd["Cm"] - r0["Cm"]); CLde = TAU_E * (rd["CL"] - r0["CL"])
    CL0, Cm0 = np.array(CL0), np.array(Cm0)
    de = -Cm0 / Cmde
    return dict(alpha=al, CL0=CL0, Cm0=Cm0, de=de, CLtrim=CL0 + CLde * de, Cmde=Cmde, CLde=CLde)

for _ in range(2):
    T = trim_map(p, g)
    de_op = float(np.interp(CL_OP_GUESS, T["CLtrim"], T["de"]))
    p["i_t"] = round(p["i_t"] + de_op * TAU_E, 1)
T = trim_map(p, g)
print(f"  tail incidence i_t = {p['i_t']:.1f} deg (body axis), wing i_w = {p['i_w']} deg; Cm_de {T['Cmde']:.4f}/deg")

# --- section Clmax vs Re (NeuralFoil, n_crit 9)
Re_g = np.array([30e3, 40e3, 50e3, 60e3, 80e3, 100e3, 130e3])
al_s = np.linspace(-4, 18, 89)
clmax_re = []
for Re in Re_g:
    r = cc.WING_FOIL.get_aero_from_neuralfoil(alpha=al_s, Re=Re * np.ones_like(al_s), n_crit=9, model_size="large")
    clmax_re.append(float(np.max(r["CL"])))
clmax_re = np.array(clmax_re)
clmax_of = lambda Re: np.interp(Re, Re_g, clmax_re)

# --- critical-section CLmax: VLM cl_i = A_i + B_i * CL_wing
apv = plane(p, g, fuselage=False, x_ref=x_cg)
rA, vA = vlm(apv, 9.0, 0.0); yA, cA, clA = strips(vA, apv)
rB, vB = vlm(apv, 9.0, 8.0); yB, cB, clB = strips(vB, apv)
Bi = (clB - clA) / (rB["CL"] - rA["CL"]); Ai = clA - Bi * rA["CL"]
Vs = 7.5
for _ in range(4):
    clm_i = clmax_of(Vs * cA / NU)
    CLs_i = (clm_i - Ai) / Bi
    ok = yA < 0.97 * g["s"]
    j = int(np.argmin(np.where(ok, CLs_i, 9)))
    CLmax_wing = float(CLs_i[j])
    al_st = float(np.interp(CLmax_wing, T["CL0"], T["alpha"]))
    CLmax = float(np.interp(al_st, T["alpha"], T["CLtrim"]))      # trimmed aircraft CLmax
    Vs = vsp(CLmax)
y_stall = float(yA[j])
print(f"  CLmax: wing {CLmax_wing:.3f}, trimmed aircraft {CLmax:.3f} at alpha {al_st:.1f} deg; first stall at y = {y_stall*1e3:.0f} mm "
      f"({y_stall/g['s']*100:.0f}% semispan); Vs = {Vs:.2f} m/s")

# --- AeroBuildup polar at the self-consistent flight speed for each alpha
apf = plane(p, g, fuselage=True, x_ref=x_cg)
al = np.arange(-3, 13.01, 0.5)
CLv = np.interp(al, T["alpha"], T["CL0"])
Vv = np.array([vsp(max(c, 0.12)) for c in CLv]); Vv = np.minimum(Vv, 25)
o = asb.AeroBuildup(airplane=apf, op_point=asb.OperatingPoint(atmosphere=cc.ATM, velocity=Vv, alpha=al), xyz_ref=[x_cg, 0, 0]).run()
CLab, CDab, Cmab = (np.ravel(o[k]) for k in ("CL", "CD", "Cm"))
CDt = CDab + CD_MISC
m_ok = (CLab > 0.1) & (CLab <= CLmax) & (al <= al[int(np.argmax(CLab))])
CLp, CDp, alp = CLab[m_ok], CDt[m_ok], al[m_ok]
cd_of = lambda CL: float(np.interp(CL, CLp, CDp))
al_of = lambda CL: float(np.interp(CL, CLp, alp))
de_of = lambda CL: float(np.interp(CL, T["CLtrim"], T["de"]))
fit = np.polyfit(CLp[(CLp > 0.2) & (CLp < 0.9)] ** 2, CDp[(CLp > 0.2) & (CLp < 0.9)], 1)
k_ind, CD0 = fit; e_osw = 1 / (math.pi * p["AR"] * k_ind)
CLfine = np.linspace(CLp[0], CLmax, 400)
CDfine = np.interp(CLfine, CLp, CDp)
i_ld = int(np.argmax(CLfine / CDfine)); i_ms = int(np.argmax(CLfine ** 1.5 / CDfine))
print(f"  polar fit CD0 {CD0:.4f}, e {e_osw:.3f}; L/D max {CLfine[i_ld]/CDfine[i_ld]:.1f} at CL {CLfine[i_ld]:.2f}; "
      f"min-sink CL {CLfine[i_ms]:.2f}")
OUT["aero"] = dict(CLmax=CLmax, CLmax_wing=CLmax_wing, alpha_stall=al_st, y_stall=y_stall, CD0=CD0, e=e_osw,
                   CLa_deg=npd["CLa_deg"], x_np=npd["x_np"], x_np_vlm=npd["x_np_vlm"], i_t=p["i_t"],
                   Cmde=T["Cmde"], CLde=T["CLde"], clmax_re=dict(zip(Re_g.tolist(), clmax_re.tolist())))

# ================================================================== 3. propulsion combos, battery choice, operating points
print("== 3. propulsion / operating points")
PROPS = pm.load_props()
def combo(bn):
    return pm.Combo(pm.MOTORS["A2212_1000"], PROPS["F9x5"], BATTS[bn][2], pm.ESCS["SKY20"])

V_ld = vsp(CLfine[i_ld]); V_ms = vsp(CLfine[i_ms])
V_op = max(V_ms, 1.30 * Vs)            # chosen low-speed operating point (>= 1.3 Vs gust/turn margin)
CL_op = 2 * W / (RHO * S * V_op ** 2)
CL_cr = 0.40; V_cr = vsp(CL_cr)
endur = {}
for bn in BATTS:
    pb, gb, mb = designs[bn][0], designs[bn][1], designs[bn][2]
    ac = pm.Airframe(mass=mb["m"], S=S, AR=p["AR"], CD0=CD0, e=e_osw, CLmax=CLmax, CL_cruise=CL_op, rho=RHO)
    R35 = V_op ** 2 / (G * math.tan(math.radians(35)))
    ev = pm.evaluate(ac, combo(bn), R_turn=R35)
    Wb = mb["m"] * G; Vb = vsp(CL_op, Wb); Db = 0.5 * RHO * Vb ** 2 * S * cd_of(CL_op)
    ob = pm.operating_point(combo(bn), Vb, pm.throttle_for_thrust(combo(bn), Vb, Db))
    endur[bn] = dict(AUW=mb["m"], I_level=ob["I_batt"], t_level=0.8 * BATTS[bn][2].cap_mAh * 3.6 / ob["I_batt"] / 60,
                     t_mission=ev["endurance_min"], t_beginner=ev["endurance_beginner_min"], ev=ev)
    print(f"  {bn}: AUW {mb['m']*1e3:.0f} g, level I {ob['I_batt']:.2f} A, level-only {endur[bn]['t_level']:.1f} min, "
          f"mission {ev['endurance_min']:.1f} min, beginner (25% climb) {ev['endurance_beginner_min']:.1f} min, static T {ev['T_static_g']:.0f} g")
# choose the lighter pack if it meets >= 10 min on the conservative beginner profile (80 % usable)
BN = BN0 if endur[BN0]["t_beginner"] >= 10.0 else "3S 850 mAh"
p, g, mm, npd, SM = designs[BN]
p["i_t"] = OUT["aero"]["i_t"]
W = mm["m"] * G; x_cg = mm["cg"][0]; C = combo(BN)
Vs = vsp(CLmax); V_ld = vsp(CLfine[i_ld]); V_ms = vsp(CLfine[i_ms]); V_op = max(V_ms, 1.30 * Vs)
CL_op = 2 * W / (RHO * S * V_op ** 2); V_cr = vsp(CL_cr)
print(f"  chosen battery: {BN}")


def point(name, CL=None, V=None):
    if V is None: V = vsp(CL)
    if CL is None: CL = 2 * W / (RHO * S * V * V)
    CLc = min(CL, CLmax); CD = cd_of(CLc); Dg = 0.5 * RHO * V * V * S * CD
    th = pm.throttle_for_thrust(C, V, Dg); opp = pm.operating_point(C, V, th) if th else None
    return dict(name=name, V=V, CL=CL, alpha=al_of(CLc), LD=CLc / CD, sink=V * CD / CLc, P_req=Dg * V,
                thr=th, I=opp["I_batt"] if opp else float("nan"), P_el=opp["P_elec"] if opp else float("nan"),
                de=de_of(CLc))

OPS = [point("Stall (1g, trimmed)", CL=CLmax), point("Hand launch", V=7.0), point("Minimum sink", CL=CLfine[i_ms]),
       point("Best L/D", CL=CLfine[i_ld]), point("Operating point (low-speed loiter)", V=V_op), point("Cruise", CL=CL_cr)]
for o_ in OPS:
    print(f"  {o_['name']:<36} V {o_['V']:5.2f} CL {o_['CL']:.3f} a {o_['alpha']:5.1f} L/D {o_['LD']:5.1f} sink {o_['sink']:.2f} "
          f"Preq {o_['P_req']:4.1f} W  I {o_['I']:.2f} A  de {o_['de']:+.1f}")

# ================================================================== 4. performance
print("== 4. performance")
TURN = []
for ph in (30, 35, 40, 45):
    n = 1 / math.cos(math.radians(ph)); t = math.tan(math.radians(ph))
    CLt = n * CL_op; w_op = 2 * V_op ** 2 / (G * t)
    Vmin = 1.2 * Vs * math.sqrt(n); w_min = 2 * Vmin ** 2 / (G * t)
    Dt = 0.5 * RHO * V_op ** 2 * S * cd_of(min(CLt, CLmax))
    th = pm.throttle_for_thrust(C, V_op, Dt); It = pm.operating_point(C, V_op, th)["I_batt"] if th else float("nan")
    TURN.append(dict(bank=ph, n=n, width_op=w_op, margin_op=CLmax / CLt, V_min=Vmin, width_min=w_min, I=It))
    print(f"  bank {ph}: width @V_op {w_op:.1f} m (CLmax/CL {CLmax/CLt:.2f}, Vs_turn {Vs*math.sqrt(n):.2f}), "
          f"width @1.2Vs_turn {w_min:.1f} m, I {It:.2f} A")
best = None; ROC = []
for V in np.linspace(1.15 * Vs, 16, 60):
    o_ = pm.operating_point(C, V, 1.0); CLv_ = min(2 * W / (RHO * S * V * V), CLmax)
    Dv = 0.5 * RHO * V * V * S * cd_of(CLv_); sg = min((o_["T"] - Dv) / W, 0.99); ROC.append((V, V * sg, o_["T"], o_["I_batt"]))
    if best is None or V * sg > best[1]: best = (V, V * sg, sg, o_)
V_cl, roc, sg, ocl = best; gam = math.degrees(math.asin(sg))
t20 = (20 - 1.8) / roc; x20 = t20 * V_cl * math.cos(math.radians(gam))
st = pm.operating_point(C, 0.0, 1.0); ol = pm.operating_point(C, 7.0, 1.0)
a_l = (ol["T"] - 0.5 * RHO * 49 * S * cd_of(min(2 * W / (RHO * S * 49), CLmax))) / mm["m"]
x_gl_ld = 10 * CLfine[i_ld] / CDfine[i_ld]; x_gl_op = 10 * OPS[4]["LD"]
# steep approach: 1.6 V_ld (CL lower) glide ratio
CL_st = CLfine[i_ld] / 1.6 ** 2; x_gl_fast = 10 * CL_st / cd_of(CL_st)
# spoilerons (both ailerons TE up 25 deg) for a steep approach: AeroBuildup at V_op
asp = np.arange(0, 10.01, 1.0)
osp = asb.AeroBuildup(airplane=plane(p, g, fuselage=True, x_ref=x_cg, spoileron=-25.0),
                      op_point=asb.OperatingPoint(atmosphere=cc.ATM, velocity=V_op * np.ones_like(asp), alpha=asp), xyz_ref=[x_cg, 0, 0]).run()
CLs_, CDs_ = np.ravel(osp["CL"]), np.ravel(osp["CD"]) + CD_MISC
LD_sp = CL_op / float(np.interp(CL_op, CLs_, CDs_)); x_gl_sp = 10 * LD_sp
PERF = dict(LD_spoileron=LD_sp, x_glide_spoileron=x_gl_sp, V_climb=V_cl, ROC=roc, gamma=gam, t20=t20, x20=x20, I_climb=ocl["I_batt"], T_static_g=st["T"] / G * 1e3,
            I_static=st["I_batt"], rpm_static=st["rpm"], TW=st["T"] / W, T7_g=ol["T"] / G * 1e3, a_launch=a_l,
            x_glide_LD=x_gl_ld, x_glide_op=x_gl_op, x_glide_fast=x_gl_fast, V_fast=vsp(CL_st))
print(f"  static T {PERF['T_static_g']:.0f} g (T/W {PERF['TW']:.2f}), I {st['I_batt']:.1f} A; launch 7 m/s T {PERF['T7_g']:.0f} g a {a_l:.1f} m/s2")
print(f"  best climb {roc:.2f} m/s at {V_cl:.1f} m/s ({gam:.0f} deg), I {ocl['I_batt']:.1f} A; to 20 m: {t20:.1f} s, {x20:.0f} m ground")
print(f"  glide from 10 m: {x_gl_ld:.0f} m at best L/D, {x_gl_op:.0f} m at V_op, {x_gl_fast:.0f} m at {PERF['V_fast']:.1f} m/s, "
      f"{x_gl_sp:.0f} m with -25 deg spoilerons at V_op (L/D {LD_sp:.1f})")

# roll: AeroBuildup aileron derivative (x0.8 knock-down), VLM roll damping
alo = OPS[4]["alpha"]
r0 = ab(plane(p, g, fuselage=False, x_ref=x_cg), V_op, alo); ra = ab(plane(p, g, fuselage=False, x_ref=x_cg, delta_a=10.0), V_op, alo)
Clda = 0.8 * abs(float(np.ravel(ra["Cl"])[0] - np.ravel(r0["Cl"])[0])) / 10.0
SV, _ = vlm(plane(p, g, fuselage=False, x_ref=x_cg), V_op, alo, stab=True)
SA = ab(plane(p, g, fuselage=True, x_ref=x_cg), V_op, alo, stab=True)
SA0 = ab(plane(p, g, fuselage=False, x_ref=x_cg), V_op, alo, stab=True)
f_ = lambda r, k: float(np.ravel(r[k])[0])
der = {k: f_(SV, k) for k in ("Clb", "Cnb", "CYb", "Clp", "Cnp", "Clr", "Cnr")}
der["Cnb"] += f_(SA, "Cnb") - f_(SA0, "Cnb"); der["Clb"] += f_(SA, "Clb") - f_(SA0, "Clb")   # pod+boom increments
pb2V_deg = Clda / -der["Clp"]; da_req = 0.08 / pb2V_deg
p_rate = 0.08 * 2 * V_op / b
qd = 0.5 * RHO * V_op ** 2
E_sp = der["Clb"] * der["Cnr"] - der["Cnb"] * der["Clr"]
lam_sp = -(G / V_op) * E_sp / (der["Clb"] * der["Cnp"] - der["Clp"] * der["Cnb"])
wn_dr = math.sqrt(qd * S * b * der["Cnb"] / mm["Izz"])
z_dr = -der["Cnr"] * (b / (2 * V_op)) * qd * S * b / mm["Izz"] / (2 * wn_dr)
LAT = dict(der=der, Clda=Clda, pb2V_per_deg=pb2V_deg, da_req=da_req, roll_rate_dps=math.degrees(p_rate),
           E_spiral=E_sp, lam_spiral=lam_sp, t2_spiral=math.log(2) / abs(lam_sp) if lam_sp else float("inf"),
           wn_dr=wn_dr, zeta_dr=z_dr, Ixx=mm["Ixx"], Iyy=mm["Iyy"], Izz=mm["Izz"])
print(f"  roll: Cl_da {Clda:.5f}/deg, Clp {der['Clp']:.3f}; pb/2V = {pb2V_deg:.4f}/deg -> 0.08 needs {da_req:.1f} deg aileron "
      f"(p = {math.degrees(p_rate):.0f} deg/s at V_op)")
print(f"  lateral: Clb {der['Clb']:.4f}, Cnb {der['Cnb']:.4f}, Clr {der['Clr']:.3f}, Cnr {der['Cnr']:.4f}; spiral E {E_sp:+.5f} "
      f"({'stable' if E_sp > 0 else 'divergent'}, lambda {lam_sp:+.3f} 1/s, t2 {LAT['t2_spiral']:.0f} s); DR wn {wn_dr:.1f} rad/s zeta {z_dr:.2f}")

# ================================================================== 5. density sensitivity, structure, segments
DENS = {}
for rho_p in (650.0, 750.0, 850.0):
    q_ = copy.deepcopy(p); q_["rho_print"] = rho_p; m_ = cc.mass_model(q_, g)
    sm_ = (npd["x_np"] - m_["cg"][0]) / mac
    xb_need = q_["x_batt"] + (npd["x_np"] - p["SM_target"] * mac - m_["cg"][0]) * m_["m"] / q_["m_batt"]
    DENS[rho_p] = dict(AUW=m_["m"], WS=m_["m"] * G / S, x_cg=m_["cg"][0], SM=sm_, Vs=vsp(CLmax, m_["m"] * G), xb=xb_need,
                       wing=m_["wing_mass"])
    print(f"  print density {rho_p/1000:.2f}: AUW {m_['m']*1e3:.0f} g, wing {m_['wing_mass']*1e3:.0f} g, W/S {m_['m']*G/S:.1f} N/m2, "
          f"SM {sm_*100:.1f}% (battery to {xb_need*1e3:.0f} mm for 12%), Vs {DENS[rho_p]['Vs']:.2f}")
sp = mm["spar"]
seg_y = [0.0, 0.240, 0.470, 0.700]
SEG = []
for i in range(3):
    c0 = cc._wing_station(p, g, seg_y[i])[2]; c1 = cc._wing_station(p, g, seg_y[i + 1])[2]
    SEG.append(dict(n=i + 1, y0=seg_y[i], y1=seg_y[i + 1], c0=c0, c1=c1, t=0.092 * c0,
                    content=["root: 6x4 tube bore, wing bolt/band seat, brass sleeve + wire joiner at outer end",
                             "aileron servo bay + aileron 250-470 mm", "aileron 470-660 mm, tip"][i]))
print(f"  spar: {sp}")

# ================================================================== 6. plots
fig, ax = plt.subplots(2, 3, figsize=(15, 8.5))
ax[0, 0].plot(al, CLab, label="AeroBuildup (de=0)"); ax[0, 0].plot(T["alpha"], T["CLtrim"], "--", label="VLM trimmed")
ax[0, 0].axhline(CLmax, color="r", lw=0.8, label=f"CLmax crit. section {CLmax:.2f}"); ax[0, 0].set(xlabel="alpha [deg]", ylabel="CL", title="CL-alpha")
ax[0, 1].plot(CDt, CLab); ax[0, 1].set(xlabel="CD", ylabel="CL", title=f"Drag polar (AB + {CD_MISC} misc)")
ax[0, 2].plot(CLfine, CLfine / CDfine, label="L/D"); ax[0, 2].plot(CLfine, CLfine ** 1.5 / CDfine, label="CL^1.5/CD")
ax[0, 2].set(xlabel="CL", title="L/D and endurance parameter")
ax[1, 0].plot(al, Cmab, label="AeroBuildup (no downwash)"); ax[1, 0].plot(T["alpha"], T["Cm0"], label="VLM (de=0)")
ax[1, 0].axhline(0, color="k", lw=0.5); ax[1, 0].set(xlabel="alpha [deg]", ylabel="Cm about CG", title=f"Cm-alpha, CG {x_cg*1e3:.0f} mm, SM {SM*100:.0f}%")
CLw = CLmax_wing
ax[1, 1].plot(yA, Ai + Bi * CLw, label="cl at aircraft stall"); ax[1, 1].plot(yA, clmax_of(Vs * cA / NU), "r--", label="section clmax(Re)")
ax[1, 1].plot(yA, (Ai + Bi * CL_op), label=f"cl at V_op (CL {CL_op:.2f})"); ax[1, 1].set(xlabel="y [m]", ylabel="cl", title="Lift distribution / critical section")
Vg = np.linspace(Vs, 16, 60); Pr = [0.5 * RHO * v ** 3 * S * cd_of(min(2 * W / (RHO * S * v * v), CLmax)) for v in Vg]
ax[1, 2].plot(Vg, Pr, label="P required"); ax[1, 2].plot([r[0] for r in ROC], [r[1] * W for r in ROC], label="excess power (full thr)")
ax[1, 2].set(xlabel="V [m/s]", ylabel="W", title="Power")
for a_ in ax.flat: a_.grid(alpha=0.3); a_.legend(fontsize=7) if a_.get_legend_handles_labels()[0] else None
fig.tight_layout(); fig.savefig(os.path.join(HERE, "aero_performance.png"), dpi=130); plt.close(fig)

# three-view
fig = plt.figure(figsize=(14, 10)); gs = fig.add_gridspec(2, 2, width_ratios=[1.25, 1])
axT, axS, axF = fig.add_subplot(gs[:, 0]), fig.add_subplot(gs[0, 1]), fig.add_subplot(gs[1, 1])
ys = np.array([0, p["y_break"], g["s"]]); st_ = [cc._wing_station(p, g, y) for y in ys]
xle = np.array([s_[0] for s_ in st_]); ch = np.array([s_[2] for s_ in st_]); zz = np.array([s_[1] for s_ in st_])
mmm = 1e3
for sgn in (1, -1):
    axT.plot(sgn * ys * mmm, -xle * mmm, "k"); axT.plot(sgn * ys * mmm, -(xle + ch) * mmm, "k")
    axT.plot([sgn * g["s"] * mmm] * 2, [-xle[-1] * mmm, -(xle[-1] + ch[-1]) * mmm], "k")
    ya = np.array([p["ail_y0"], p["ail_y1"]]); ca = np.interp(ya, ys, ch); xa = np.interp(ya, ys, xle)
    axT.fill_between(sgn * ya * mmm, -(xa + ca * 0.75) * mmm, -(xa + ca) * mmm, color="tab:blue", alpha=0.4)
    for y_ in seg_y[1:3]: axT.plot([sgn * y_ * mmm] * 2, [-np.interp(y_, ys, xle) * mmm, -(np.interp(y_, ys, xle) + np.interp(y_, ys, ch)) * mmm], "k:", lw=0.7)
    xh = p["x_h_le"]; yh = g["bh"] / 2
    axT.plot([0, sgn * yh * mmm, sgn * yh * mmm, 0], [-xh * mmm, -(xh + (g["crh"] - g["cth"]) / 4) * mmm, -(xh + (g["crh"] - g["cth"]) / 4 + g["cth"]) * mmm, -(xh + g["crh"]) * mmm], "k")
    axT.fill_between([0, sgn * yh * mmm], [-(xh + 0.6 * g["crh"]) * mmm, -(xh + (g["crh"] - g["cth"]) / 4 + 0.6 * g["cth"]) * mmm],
                     [-(xh + g["crh"]) * mmm, -(xh + (g["crh"] - g["cth"]) / 4 + g["cth"]) * mmm], color="tab:blue", alpha=0.4)
axT.add_patch(plt.Rectangle((-p["pod_w"] / 2 * mmm, -p["x_pod_end"] * mmm), p["pod_w"] * mmm, (p["x_pod_end"] - p["x_nose"]) * mmm, fc="0.85", ec="k"))
axT.plot([0, 0], [-p["x_pod_end"] * mmm, -g["x_tail_end"] * mmm], "k", lw=3)
axT.plot([-114, 114], [-(p["x_nose"] - 0.01) * mmm] * 2, "0.4", lw=2)
axT.plot(0, -x_cg * mmm, "o", color="tab:red", ms=8); axT.plot(0, -npd["x_np"] * mmm, "^", color="tab:green", ms=8)
axT.text(15, -x_cg * mmm, f"CG {x_cg*mmm:.0f}", color="tab:red", fontsize=8); axT.text(15, -npd["x_np"] * mmm - 12, f"NP {npd['x_np']*mmm:.0f}", color="tab:green", fontsize=8)
axT.annotate("", (-700, 60), (700, 60), arrowprops=dict(arrowstyle="<->")); axT.text(0, 68, f"span {b*mmm:.0f}", ha="center")
axT.text(-700, -(xle[-1] + ch[-1]) * mmm - 20, f"tip c {ch[-1]*mmm:.0f}", fontsize=8)
axT.text(-p["y_break"] * mmm, -ch[0] * mmm - 20, f"root c {ch[0]*mmm:.0f} (const. to y={p['y_break']*mmm:.0f})", fontsize=8, ha="center")
axT.text(-500, -(p["x_h_le"] + g["crh"]) * mmm - 25, f"H-stab {g['bh']*mmm:.0f} x {g['crh']*mmm:.0f}/{g['cth']*mmm:.0f}, elevator 40%", fontsize=8)
axT.text(330, 40, "segment joints at y=240, 470", fontsize=8)
axT.annotate("", (760, -p["x_nose"] * mmm), (760, -g["x_tail_end"] * mmm), arrowprops=dict(arrowstyle="<->"))
axT.text(770, -300, f"length {g['L_fus']*mmm:.0f}\n(nose {p['x_nose']*mmm:.0f}\n to tail {g['x_tail_end']*mmm:.0f})", fontsize=8)
axT.set_aspect("equal"); axT.set_title("Top view [mm] (origin: wing root LE)"); axT.grid(alpha=0.2)
# side
axS.add_patch(plt.Rectangle((p["x_nose"] * mmm, (p["z_pod"] - p["pod_h"] / 2) * mmm), (p["x_pod_end"] - p["x_nose"]) * mmm, p["pod_h"] * mmm, fc="0.85", ec="k"))
axS.plot([p["x_pod_end"] * mmm, g["x_tail_end"] * mmm], [p["z_boom"] * mmm] * 2, "k", lw=3)
ar_ = math.radians(p["i_w"]); axS.plot([0, ch[0] * math.cos(ar_) * mmm], [0, -ch[0] * math.sin(ar_) * mmm], "k", lw=3)
x0 = p["x_h_le"] + 0.02; zf = p["z_h"] + 0.004
axS.plot(np.array([x0, x0 + 0.15 * g["hv"], x0 + 0.15 * g["hv"] + g["ctv"], x0 + g["crv"], x0]) * mmm,
         np.array([zf, zf + g["hv"], zf + g["hv"], zf, zf]) * mmm, "k")
axS.plot(np.array([p["x_h_le"], p["x_h_le"] + g["crh"]]) * mmm, [p["z_h"] * mmm] * 2, "k", lw=2)
axS.plot([(p["x_nose"] - 0.01) * mmm] * 2, [(p["z_pod"] - 0.114) * mmm, (p["z_pod"] + 0.114) * mmm], "0.4", lw=2)
axS.plot(x_cg * mmm, mm["cg"][2] * mmm, "o", color="tab:red")
axS.text(x0 * mmm, (zf + g["hv"]) * mmm + 10, f"fin h {g['hv']*mmm:.0f}, c {g['crv']*mmm:.0f}/{g['ctv']*mmm:.0f}, rudder 40%", fontsize=8)
axS.text(-150, 60, f"wing i_w {p['i_w']:+.1f} deg, tail i_t {p['i_t']:+.1f} deg\nprop 9x5 folding, D 229", fontsize=8)
axS.set_aspect("equal"); axS.set_title("Side view [mm]"); axS.grid(alpha=0.2)
# front
for sgn in (1, -1):
    axF.plot(sgn * ys * mmm, zz * mmm, "k", lw=2)
    axF.plot([0, sgn * g["bh"] / 2 * mmm], [p["z_h"] * mmm] * 2, "k")
axF.plot([0, 0], [zf * mmm, (zf + g["hv"]) * mmm], "k", lw=2)
axF.add_patch(plt.Circle((0, p["z_pod"] * mmm), 114, fill=False, ls="--", ec="0.5"))
axF.add_patch(plt.Rectangle((-p["pod_w"] / 2 * mmm, (p["z_pod"] - p["pod_h"] / 2) * mmm), p["pod_w"] * mmm, p["pod_h"] * mmm, fc="0.85", ec="k"))
axF.text(300, zz[-1] * mmm + 15, f"polyhedral {p['dih_outer']:.0f} deg outboard of y={p['y_break']*mmm:.0f}\ntip rise {zz[-1]*mmm:.0f} mm, washout {abs(p['twist_tip']):.0f} deg", fontsize=8)
axF.set_aspect("equal"); axF.set_title("Front view [mm]"); axF.grid(alpha=0.2)
fig.tight_layout(); fig.savefig(os.path.join(HERE, "three_view.png"), dpi=140); plt.close(fig)

# ================================================================== 7. save numbers
R = dict(battery=BN, endur={k: {kk: vv for kk, vv in v.items() if kk != "ev"} for k, v in endur.items()},
         geom={k: float(v) for k, v in g.items() if np.isscalar(v)}, params={k: v for k, v in p.items() if np.isscalar(v) or v is None},
         mass_items=[it[:5] for it in mm["items"]], AUW=mm["m"], cg=mm["cg"].tolist(), SM=SM, np=npd, ops=OPS, turn=TURN, perf=PERF,
         lat=LAT, dens={str(k): v for k, v in DENS.items()}, spar={k: (float(v) if isinstance(v, (float, np.floating)) else v) for k, v in sp.items()},
         seg=SEG, aero=OUT["aero"], V=dict(Vs=Vs, V_ms=V_ms, V_ld=V_ld, V_op=V_op, V_cr=V_cr, CL_op=CL_op),
         ev={k: v["ev"] for k, v in endur.items()})
json.dump(R, open(os.path.join(HERE, "final_results.json"), "w"), indent=1, default=float)
print("saved final_results.json, aero_performance.png, three_view.png")

# ================================================================== 8. design_spec.md
f1 = lambda v, d=1: f"{v:.{d}f}"
mmv = lambda v: f"{v*1e3:.0f}"
pct = lambda x: (x - g["x_le_mac"]) / mac * 100
ev = endur[BN]["ev"]
L = []
A = L.append
A(f"# Final design specification -- 1.40 m hand-launched powered glider\n")
A(f"Single source of truth, generated by `final_analysis.py` (aerosandbox 4.2.10: VLM for NP / trim / lift distribution, "
  f"AeroBuildup + NeuralFoil for drag and control derivatives; UIUC-calibrated propulsion model). Air rho {RHO} kg/m3, nu {NU} m2/s. "
  f"Axes: origin = wing root leading edge, x aft, z up; all positions in mm. Design print density 0.75 g/cm3.\n")
A("## 1. Geometry\n| Item | Value |\n|---|---|")
for k, v in [("Span b", f"{b*1e3:.0f} mm"), ("Aspect ratio", f"{p['AR']:.1f}"), ("Wing area S", f"{S*1e4:.0f} cm2 ({S:.4f} m2)"),
             ("Planform", f"double trapezoid: constant chord to y = {mmv(p['y_break'])}, then straight taper (lambda {p['taper_outer']}) with straight quarter-chord line"),
             ("Root / tip chord", f"{mmv(g['cr'])} / {mmv(g['ct'])} mm"), ("MAC / y_MAC / x_LE,MAC", f"{mmv(mac)} / {mmv(g['y_mac'])} / {f1(g['x_le_mac']*1e3)} mm"),
             ("Wing airfoil", "SD7037 with 0.5 mm blunt TE (`03_airfoil/dat/sd7037_te05.dat`), t/c 9.2 %"),
             ("Dihedral", f"0 deg centre panel, {p['dih_outer']:.0f} deg polyhedral outboard of y = {mmv(p['y_break'])} (tip rise {cc._wing_station(p, g, g['s'])[1]*1e3:.0f} mm)"),
             ("Incidence / twist", f"wing root +{p['i_w']:.1f} deg to pod axis, linear washout to {p['i_w']+p['twist_tip']:+.1f} deg at tip (-2 deg); H-stab {p['i_t']:+.1f} deg (decalage {p['i_w']-p['i_t']:.1f} deg)"),
             ("Tip Re at V_op / Vs", f"{V_op*g['ct']/NU/1e3:.0f}k / {Vs*g['ct']/NU/1e3:.0f}k (root {V_op*g['cr']/NU/1e3:.0f}k)"),
             ("H-stab (3 mm Depron flat plate)", f"S_h {g['Sh']*1e4:.0f} cm2, span {mmv(g['bh'])}, chord {mmv(g['crh'])}/{mmv(g['cth'])} mm, AR {p['ARh']}, LE at x = {mmv(p['x_h_le'])}, V_H {g['VH']:.2f}, l_t {mmv(g['lt'])}"),
             ("Fin (3 mm Depron)", f"S_v {g['Sv']*1e4:.0f} cm2, height {mmv(g['hv'])}, chord {mmv(g['crv'])}/{mmv(g['ctv'])} mm, V_v {g['Vv']:.3f}, l_v {mmv(g['lv'])}"),
             ("Pod", f"printed PLA Aero, x {mmv(p['x_nose'])} to {mmv(p['x_pod_end'])} ({(p['x_pod_end']-p['x_nose'])*1e3:.0f} mm), {mmv(p['pod_w'])} x {mmv(p['pod_h'])} mm max section"),
             ("Boom", f"CF tube 8x6, {mmv(g['boom_len'])} mm (30 mm inside pod socket), tail end at x = {mmv(g['x_tail_end'])}"),
             ("Overall length", f"{mmv(g['L_fus'])} mm incl. spinner")]:
    A(f"| {k} | {v} |")
A("\nThree-view: `three_view.png`. Aero/performance plots: `aero_performance.png`.\n")
A("## 2. Control surfaces\n| Surface | Size | Throw (recommended) | Notes |\n|---|---|---|---|")
A(f"| Ailerons (2 servos) | 25 % chord, y = {mmv(p['ail_y0'])}..{mmv(p['ail_y1'])} mm each side | +15 up / -10 down (differential); spoileron mix -25 (both up) | pb/2V = {LAT['pb2V_per_deg']:.4f}/deg, 0.08 reached at {LAT['da_req']:.1f} deg |")
A(f"| Elevator | 40 % chord, full H-stab span | +-12 deg (expo 30 %) | trim range {OPS[0]['de']:+.1f} (stall) to {OPS[5]['de']:+.1f} (cruise) deg, TE-down positive, tau_e {TAU_E} |")
A(f"| Rudder | 40 % fin chord, full height | +-20 deg | coordinated turns / ground handling |")
A("\n## 3. Mass breakdown (3S 650 design, print density 0.75)\n| Component | Mass g | x mm |\n|---|---|---|")
grp = cc.group_masses(mm)
for k, (m_g, x_) in grp.items():
    if m_g > 0: A(f"| {k} | {m_g:.1f} | {x_*1e3:.0f} |")
A(f"| **AUW** | **{mm['m']*1e3:.0f}** | **CG {x_cg*1e3:.1f}** |")
A(f"\nWing (shell + spars + joints) {mm['wing_mass']*1e3:.0f} g; wing shell areal density {mm['wing_areal']*1e3:.0f} g/m2. "
  f"Inertia about CG: Ixx {mm['Ixx']*1e3:.1f}, Iyy {mm['Iyy']*1e3:.1f}, Izz {mm['Izz']*1e3:.1f} g m2.\n")
A("| Print density | AUW g | wing g | W/S N/m2 | Vs m/s | SM (battery unchanged) | battery x for SM 12 % |\n|---|---|---|---|---|---|---|")
for k_, d_ in DENS.items():
    A(f"| {k_/1000:.2f} g/cm3 | {d_['AUW']*1e3:.0f} | {d_['wing']*1e3:.0f} | {d_['WS']:.1f} | {d_['Vs']:.2f} | {d_['SM']*100:.1f} % | {d_['xb']*1e3:.0f} mm |")
A(f"\n3S 850 alternative: AUW {endur['3S 850 mAh']['AUW']*1e3:.0f} g (battery at x = {designs['3S 850 mAh'][0]['x_batt']*1e3:.0f} mm).\n")
A("## 4. Balance and longitudinal stability\n| Item | Value |\n|---|---|")
for k, v in [("CG", f"x = {x_cg*1e3:.1f} mm behind wing root LE = {pct(x_cg):.1f} % MAC (z = {mm['cg'][2]*1e3:.0f} mm)"),
             ("Neutral point", f"x = {npd['x_np']*1e3:.1f} mm = {pct(npd['x_np']):.1f} % MAC (VLM wing+tail {npd['x_np_vlm']*1e3:.1f} mm, pod/boom shift {(npd['x_np']-npd['x_np_vlm'])*1e3:+.1f} mm)"),
             ("Static margin", f"{SM*100:.1f} % MAC (target 10-15 %: CG window x = {(npd['x_np']-0.15*mac)*1e3:.0f} to {(npd['x_np']-0.10*mac)*1e3:.0f} mm)"),
             ("Battery position", f"pack centre x = {p['x_batt']*1e3:.0f} mm (bay {cc.batt_bay(p)[0]*1e3:.0f}..{cc.batt_bay(p)[1]*1e3:.0f}); each 10 mm of battery moves CG {p['m_batt']/mm['m']*10:.1f} mm"),
             ("CL_alpha (VLM)", f"{npd['CLa_deg']:.4f} /deg ({npd['CLa_deg']*180/math.pi:.2f} /rad)"),
             ("Cm_delta_e", f"{T['Cmde']:.4f} /deg"),
             ("CLmax (critical section)", f"wing {CLmax_wing:.2f}, trimmed aircraft {CLmax:.2f}; first section to stall at y = {y_stall*1e3:.0f} mm ({y_stall/g['s']*100:.0f} % semispan, inboard of ailerons)"),
             ("Drag polar fit", f"CD = {CD0:.4f} + {k_ind:.4f} CL^2 (e_eff {e_osw:.2f}) for CL 0.2-0.9; includes {CD_MISC} excrescence allowance")]:
    A(f"| {k} | {v} |")
A("\n## 5. Operating points (AUW %.0f g, W/S %.1f N/m2)\n| Point | V m/s | CL | alpha deg (pod) | L/D | sink m/s | P_req W | I_batt A (level) | elevator deg |\n|---|---|---|---|---|---|---|---|---|" % (mm['m']*1e3, W/S))
for o_ in OPS:
    A(f"| {o_['name']} | {o_['V']:.2f} | {o_['CL']:.3f} | {o_['alpha']:.1f} | {o_['LD']:.1f} | {o_['sink']:.2f} | {o_['P_req']:.1f} | {o_['I']:.2f} | {o_['de']:+.1f} |")
A(f"\nV_op = max(V_min-sink, 1.3 Vs). Hand launch CL {OPS[1]['CL']:.2f} = {OPS[1]['CL']/CLmax*100:.0f} % of CLmax -> throw firmly (>= 8 m/s) at full throttle.\n")
A("## 6. Performance\n| Bank | n | 180 deg turn width at V_op | CLmax/CL at V_op | width at 1.2 Vs,turn | I_batt A |\n|---|---|---|---|---|---|")
for t_ in TURN:
    A(f"| {t_['bank']} deg | {t_['n']:.2f} | {t_['width_op']:.1f} m | {t_['margin_op']:.2f} | {t_['width_min']:.1f} m ({t_['V_min']:.1f} m/s) | {t_['I']:.2f} |")
A("\n| Item | Value |\n|---|---|")
for k, v in [("Static thrust / T/W", f"{PERF['T_static_g']:.0f} g / {PERF['TW']:.2f} ({PERF['rpm_static']:.0f} rpm, {PERF['I_static']:.1f} A)"),
             ("Thrust at 7 m/s launch", f"{PERF['T7_g']:.0f} g, initial acceleration {PERF['a_launch']:.1f} m/s2"),
             ("Best climb (full throttle)", f"{PERF['ROC']:.1f} m/s at {PERF['V_climb']:.1f} m/s, {PERF['gamma']:.0f} deg, {PERF['I_climb']:.1f} A; 1.8 -> 20 m in {PERF['t20']:.1f} s over {PERF['x20']:.0f} m"),
             ("Practical 25 deg climb", f"{ev['roc25']:.1f} m/s at {ev['thr_climb25']*100:.0f} % throttle, {ev['I_climb25']:.1f} A; 20 m reached in {ev['x_climb_20m']:.0f} m ground"),
             ("Glide from 10 m (motor off, prop folded)", f"{PERF['x_glide_LD']:.0f} m at best L/D; {PERF['x_glide_fast']:.0f} m diving at {PERF['V_fast']:.1f} m/s; {PERF['x_glide_spoileron']:.0f} m with -25 deg spoilerons at V_op (L/D {PERF['LD_spoileron']:.1f})"),
             ("Roll", f"Cl_da {LAT['Clda']:.4f}/deg (AeroBuildup x0.8), Clp {LAT['der']['Clp']:.3f}; pb/2V 0.08 at {LAT['da_req']:.1f} deg -> {LAT['roll_rate_dps']:.0f} deg/s at V_op; 15 deg gives pb/2V {LAT['pb2V_per_deg']*15:.2f}")]:
    A(f"| {k} | {v} |")
d_ = LAT["der"]
A("\n## 7. Lateral-directional checks (at V_op, VLM + AeroBuildup pod/boom increment)\n| Derivative / mode | Value | Verdict |\n|---|---|---|")
A(f"| Cl_beta | {d_['Clb']:.4f} /rad | {'OK (<0)' if d_['Clb'] < 0 else 'FAIL'} |")
A(f"| Cn_beta | {d_['Cnb']:.4f} /rad | {'OK (>0)' if d_['Cnb'] > 0 else 'FAIL'} |")
A(f"| Clp, Cnp, Clr, Cnr | {d_['Clp']:.3f}, {d_['Cnp']:.3f}, {d_['Clr']:.3f}, {d_['Cnr']:.4f} | |")
A(f"| Spiral: Cl_b Cn_r - Cn_b Cl_r | {LAT['E_spiral']:+.5f} | {'stable' if LAT['E_spiral'] > 0 else 'slightly divergent'}: lambda {LAT['lam_spiral']:+.3f} 1/s, time to {'half' if LAT['E_spiral']>0 else 'double'} {LAT['t2_spiral']:.0f} s (acceptable > 20 s; FC levels it) |")
A(f"| Dutch roll (approx.) | wn {LAT['wn_dr']:.1f} rad/s, zeta {LAT['zeta_dr']:.2f} | {'OK (zeta > 0.08)' if LAT['zeta_dr'] > 0.08 else 'low damping'} |")
A("\n## 8. Propulsion and endurance\n| Item | Value |\n|---|---|")
for k, v in [("Motor", "A2212/13T 1000 KV, 50 g, 3.17 mm shaft, front (firewall) mount on printed PETG mount, 2 deg down/right thrust"),
             ("Prop", "9x5 folding (Aeronaut CAM-type blades) on 30-32 mm spinner/hub, 3.17 mm collet, 18 g"),
             ("ESC", "Hobbywing Skywalker 20 A, 19 g (BEC NOT used for servos if FC has its own 5 V/servo regulator; else 5 V/2 A linear)"),
             ("Battery", f"{BN} LiPo (~45C+), {BATTS[BN][0]*1e3:.0f} g, XT30; centre at x = {p['x_batt']*1e3:.0f} mm"),
             ("Static current / ESC margin", f"{ev['I_static']:.1f} A / {ev['esc_margin']:.1f}x"),
             ("Level current at V_op", f"{endur[BN]['I_level']:.2f} A (incl. 0.5 A avionics)")]:
    A(f"| {k} | {v} |")
A("\n| Battery | AUW g | level-only (80 % usable) | racetrack mission | conservative (25 % time climbing) |\n|---|---|---|---|---|")
for bn, e_ in endur.items():
    A(f"| {bn} | {e_['AUW']*1e3:.0f} | {e_['t_level']:.1f} min | {e_['t_mission']:.1f} min | {e_['t_beginner']:.1f} min |")
A(f"\nChoice: **{BN}** -- meets >= 10 min on the conservative profile with 20 % reserve; the 850 adds 13 g for +4 min.\n")
A("## 9. Structure and print plan\n| Item | Value |\n|---|---|")
A(f"| Design load | n = 4 x 1.5 = 6, elliptic lift, allowable 400 MPa (CF), AUW {mm['m']*1e3:.0f} g |")
A(f"| Root bending moment | {sp['M_root']:.2f} N m |")
A(f"| Main spar (27 % chord) | {sp['inner']} centre tube {sp['L_in']*1e3:.0f} mm (one piece through pod and both W1 segments); {sp['outer']} outer tubes 2 x {sp['L_out']*1e3:.0f} mm in W2+W3 |")
A(f"| Polyhedral joiners | 2 x 3 mm music wire, 100 mm, pre-bent {p['dih_outer']:.0f} deg; 50 mm into a 4x3 brass sleeve bonded in the 6x4 end, 50 mm into the 5x3 bore; joint moment {sp['M_break']:.2f} N m |")
A(f"| Stress at n = 6 | 6x4 root {sp['sigma_root']/1e6:.0f} MPa, 5x3 at break {sp['sigma_step']/1e6:.0f} MPa (< 400 allowable); wire {sp['sigma_wire']/1e6:.0f} MPa (music wire yield ~1600) ; spar system mass {sp['mass']*1e3:.1f} g |")
A(f"| Rear spar | 2 mm CF rod at ~65 % chord: 1 x {2*p['y_break']*1e3:.0f} mm centre + 2 x {(0.97*b/2-p['y_break'])*1e3:.0f} mm outer (anti-torsion, aligns segments) |")
A(f"| Root thickness | {0.092*g['cr']*1e3:.1f} mm (6 mm tube uses {6/(0.092*g['cr']*1e3)*100:.0f} %) |")
A("\n| Segment (each side, x2) | y range mm | chord mm (in/out) | max thickness mm | content |\n|---|---|---|---|---|")
for s_ in SEG:
    A(f"| W{s_['n']} | {s_['y0']*1e3:.0f}-{s_['y1']*1e3:.0f} | {s_['c0']*1e3:.0f}/{s_['c1']*1e3:.0f} | {s_['t']*1e3:.1f} | {s_['content']} |")
A("\nAll 6 wing segments fit the 256 x 256 mm bed standing on the root rib (height <= 240 mm, footprint <= 155 x 15 mm). Pod printed in 2 halves (x <= 240 mm each) or nose/rear split; tail mounts, motor mount (PETG) and polyhedral keys printed separately.\n")
A("## 10. Top risks\n| # | Risk | Mitigation |\n|---|---|---|")
for i_r, r_ in enumerate([
    ("Landing: power-off glide from 10 m is %.0f m at best L/D, longer than the 100 m field" % PERF['x_glide_LD'],
     "Spoileron mix (-25 deg, glide %.0f m from 10 m); start final at <= 6 m; S-turn on final; motor-off well before the threshold" % PERF['x_glide_spoileron']),
    ("Hand launch at 7 m/s needs CL %.2f (%.0f %% of CLmax %.2f)" % (OPS[1]['CL'], OPS[1]['CL']/CLmax*100, CLmax), "Full throttle before release (T/W %.2f), throw >= 8 m/s, wings level, 5-10 deg nose up" % PERF['TW']),
    ("PLA Aero density varies 0.65-0.85 -> AUW %.0f-%.0f g" % (DENS[650.0]['AUW']*1e3, DENS[850.0]['AUW']*1e3), "Weigh each segment; calibrate flow/temperature first; re-place the battery for SM 12 % (table in section 3)"),
    ("Turn width at 35 deg is %.1f m (target 22 m)" % TURN[1]['width_op'], "Fly 40 deg bank (%.1f m, CLmax/CL %.2f) with FC bank limit 45 deg; turns may overfly the sidelines" % (TURN[2]['width_op'], TURN[2]['margin_op'])),
    ("NP uncertainty (VLM vs AeroBuildup disagree; flat-plate tail at Re ~ 40k)", "Balance at %.0f mm (SM 12 %%); first flights at the forward end (SM 15 %%, %.0f mm); glide-test before powered flight" % (x_cg*1e3, (npd['x_np']-0.15*mac)*1e3)),
    ("CD0 may be nearer 0.03 than the predicted %.3f" % CD0, "Endurance margin is large (level current ~1.2 A); L/D would fall to ~13"),
    ("Mildly divergent spiral (t2 %.0f s)" % LAT['t2_spiral'], "Acceptable for an FC-stabilised aircraft; add 2 deg polyhedral if flown manually"),
], 1):
    A(f"| {i_r} | {r_[0]} | {r_[1]} |")
txt = "\n".join(L).replace("stage-1 stepped CF sizing", "6x4 + 5x3 CF, 2 wire joiners, brass sleeves").replace("polyhedral joints (PETG keys, 2x)", "polyhedral joint ribs (printed, 2x)")
open(os.path.join(HERE, "design_spec.md"), "w").write(txt + "\n")
print("wrote design_spec.md")
