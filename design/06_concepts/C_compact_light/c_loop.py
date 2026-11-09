"""
c_loop.py -- coupled sizing loop for Concept C:
geometry -> tail sizing (V_H, V_v with l_t from current CG) -> VLM neutral point ->
CG target (x_np - SM*MAC) -> mass model with positions -> nose length (motor/ESC/battery bay
station) solved so that the battery sits mid-bay at the target CG -> new CG -> repeat.
"""
import math
import numpy as np
from scipy.optimize import brentq
import aerosandbox as asb
import c_common as cc

X_NOSE_MIN, X_NOSE_MAX = -0.40, -0.15      # shortest nose: motor + ESC + FC shelf (pod >= 0.30 m)


def np_vlm(p, g, alpha=3.0, V=9.0, res=(8, 8)):
    ap = cc.airplane(p, g, x_ref=g["x_cg_used"])
    vlm = asb.VortexLatticeMethod(ap, cc.op(V, alpha=alpha), spanwise_resolution=res[0], chordwise_resolution=res[1])
    r = vlm.run_with_stability_derivatives(alpha=True, beta=False, p=False, q=False, r=False)
    return float(r["x_np"]), float(r["CLa"]), float(r["Cma"])


def solve_battery(p, g, target):
    """Battery CG position giving x_cg = target (clipped to the bay)."""
    lo, hi = cc.batt_bay(p)
    def f(xb):
        q = dict(p, x_batt=xb)
        return cc.mass_model(q, g)["cg"][0] - target
    if f(lo) > 0:
        return lo, False
    if f(hi) < 0:
        return hi, False
    return brentq(f, lo, hi, xtol=1e-5), True


def converge(p, SM=None, res=(8, 8), verbose=False, fix_nose=False, max_it=20):
    p = dict(p)
    SM = p["SM_target"] if SM is None else SM
    w = cc.wing_planform(p)
    x_cg = w["x_le_mac"] + 0.35 * w["mac"]
    hist = []
    for it in range(max_it):
        g = cc.geometry(p, x_cg)
        x_np, CLa, Cma = np_vlm(p, g, res=res)
        target = x_np - SM * g["mac"]
        note = ""
        if not fix_nose:
            def f(xn):
                q = dict(p, x_nose=xn, x_batt=None)
                return cc.mass_model(q, cc.geometry(q, x_cg))["cg"][0] - target
            f_short, f_long = f(X_NOSE_MAX), f(X_NOSE_MIN)
            if f_short < 0:            # even the shortest nose is too nose-heavy -> shortest nose, battery aft
                p["x_nose"] = X_NOSE_MAX
                g = cc.geometry(p, x_cg)
                xb, ok = solve_battery(dict(p, x_batt=None), g, target)
                p["x_batt"] = xb; note = "shortest nose; battery moved aft" + ("" if ok else " (bay limit)")
            elif f_long > 0:           # tail-heavy even with the longest nose -> ballast
                p["x_nose"] = X_NOSE_MIN; p["x_batt"] = None
                g = cc.geometry(p, x_cg)
                def fb(mb):
                    q = dict(p, m_ballast=mb)
                    return cc.mass_model(q, g)["cg"][0] - target
                p["m_ballast"] = brentq(fb, 0.0, 0.2); note = "ballast"
            else:
                p["x_nose"] = brentq(f, X_NOSE_MIN, X_NOSE_MAX, xtol=1e-5); p["x_batt"] = None
        else:
            xb, ok = solve_battery(dict(p, x_batt=None), g, target)
            p["x_batt"] = xb
            note = "battery solved" + ("" if ok else " (bay limit)")
        g = cc.geometry(p, x_cg)
        mm = cc.mass_model(p, g)
        x_new = float(mm["cg"][0])
        hist.append(dict(it=it, x_cg=x_new, x_np=x_np, Sh=g["Sh"], Sv=g["Sv"], lt=g["lt"], m=mm["m"], x_nose=p["x_nose"],
                         x_batt=cc.batt_x(p), SM=(x_np - x_new) / g["mac"], note=note))
        if verbose:
            print(hist[-1])
        if abs(x_new - x_cg) < 2e-4 and it > 0:
            x_cg = x_new
            break
        x_cg = x_new
    g = cc.geometry(p, x_cg)
    mm = cc.mass_model(p, g)
    x_np, CLa, Cma = np_vlm(p, g, res=res)
    return dict(p=p, g=g, mm=mm, x_np=x_np, CLa=CLa, Cma=Cma, x_cg=x_cg,
                SM=(x_np - x_cg) / g["mac"], hist=hist)


def turn_width_simple(WS, CLmax, phi_deg, margin=1.2, rho=cc.RHO):
    n = 1 / math.cos(math.radians(phi_deg))
    Vs = math.sqrt(2 * WS / (rho * CLmax))
    V = margin * Vs * math.sqrt(n)
    R = V * V / (cc.G * math.tan(math.radians(phi_deg)))
    return 2 * R, V, Vs
