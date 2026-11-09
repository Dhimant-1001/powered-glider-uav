"""
s1_sizing.py -- Concept A coupled sizing loop.

geometry -> tail sizing (V_H, V_V from CG) -> component mass layout (x, z) -> CG -> VLM neutral point
-> CG target (SM) -> nose length (firewall x) solved so the CG lands on target with the battery at
the centre of its bay -> W/S -> critical-section CLmax(Re) -> V_stall -> spar re-check ... until converged.
Then: AUW / CG / speeds at PLA-Aero density 0.65 / 0.75 / 0.85 g/cm^3 (geometry frozen at 0.75 design).

Run: /opt/anaconda3/bin/python3 s1_sizing.py
"""
import os
import sys
import math
from dataclasses import asdict

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "lib"))
from concept import *  # noqa
from scipy.optimize import brentq
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

SM_TARGET = 0.13          # VLM static margin target (fraction of MAC); AeroBuildup (with pod) checked separately
RHO_DESIGN = 750.0
log = []


def P(*a):
    s = " ".join(str(x) for x in a)
    print(s)
    log.append(s)


def vlm_np(g, x_cg, z_cg, V=8.0, alpha=3.0):
    ap = build_airplane(g, x_cg, z_cg, mode="vlm")
    d = asb.VortexLatticeMethod(ap, op(V, alpha), **VLM_RES).run_with_stability_derivatives(
        alpha=True, beta=False, p=False, q=False, r=False)
    return float(d["x_np"]), float(d["Cma"]), float(d["CLa"])


def cg_for_fw(g, x_fw, rho_fil, eq, W_guess):
    g.x_fw = x_fw
    items, sp = mass_layout(g, rho_fil, eq, W_guess=W_guess)
    return mass_props(items), items, sp


def converge(g: Geo, rho_fil=RHO_DESIGN, eq=None, verbose=True):
    eq = eq or Equip()
    x_cg_t = 0.06
    W = 5.8
    hist = []
    for it in range(12):
        g.size_tail(x_cg_t)
        x_np, Cma, CLa = vlm_np(g, x_cg_t, -0.015)
        x_cg_new = x_np - SM_TARGET * g.MAC
        g.size_tail(x_cg_new)
        # solve nose (firewall) position so the CG = target with battery at bay centre
        f = lambda xf: cg_for_fw(g, xf, rho_fil, eq, W)[0]["x"] - x_cg_new
        xf = brentq(f, -0.40, -0.08, xtol=1e-5)
        mp, items, sp = cg_for_fw(g, xf, rho_fil, eq, W)
        W = mp["m"] * G
        hist.append(dict(it=it, x_cg=mp["x"], x_np=x_np, S_h=g.S_h, S_v=g.S_v, x_fw=xf, m=mp["m"]))
        if verbose:
            P(f"  it {it}: m={mp['m']*1000:.1f} g  x_np={x_np*1000:.1f} mm  x_cg={mp['x']*1000:.1f} mm  "
              f"S_h={g.S_h*1e4:.1f} cm2  S_v={g.S_v*1e4:.1f} cm2  x_fw={xf*1000:.1f} mm")
        if abs(x_cg_new - x_cg_t) < 2e-4 and it > 1:
            break
        x_cg_t = x_cg_new
    return dict(mp=mp, items=items, spar=sp, x_np=x_np, Cma=Cma, CLa=CLa, hist=hist)


def main():
    g = Geo()
    P("=== Concept A coupled sizing loop (design PLA Aero density 0.75 g/cm3) ===")
    P(f"Wing: b={g.b} m  S={g.S:.4f} m2  AR={g.AR:.2f}  MAC={g.MAC:.4f} m  c_r={g.c_r}  c_t={g.c_t}  "
      f"lambda={g.c_t/g.c_r:.3f}  y_MAC={g.y_mac:.3f}  x_MAC_LE={g.x_mac_le*1000:.1f} mm")
    outer = []
    Vs = 7.0
    for k in range(3):     # outer loop: CLmax/Vs and spar sizing use converged weight
        r = converge(g)
        mp = r["mp"]
        cs = critical_section(g, mp["x"], mp["z"], mp["m"] * G, V0=Vs)
        Vs = cs["V_stall"]
        outer.append(dict(m=mp["m"], CLmax=cs["CLmax"], Vs=Vs))
        P(f"outer {k}: AUW={mp['m']*1000:.1f} g  W/S={mp['m']*G/g.S:.2f} N/m2  CLmax(crit-section, trimmed)="
          f"{cs['CLmax']:.3f} at alpha={cs['alpha']:.2f} deg, de={cs['de']:.2f}; first stall at eta={cs['eta_crit']:.2f} "
          f"(Re {cs['Re_crit']/1e3:.0f}k, Clmax_sec {cs['Clmax_sec']:.3f}); V_stall={Vs:.2f} m/s")
        if k > 0 and abs(outer[-1]["m"] - outer[-2]["m"]) < 1e-4:
            break
    mp = r["mp"]
    tv = g.tail_volumes(mp["x"])
    cg_mac = (mp["x"] - g.x_mac_le) / g.MAC
    np_mac = (r["x_np"] - g.x_mac_le) / g.MAC
    P(f"Converged: AUW {mp['m']*1000:.1f} g, CG x={mp['x']*1000:.1f} mm ({cg_mac*100:.1f} % MAC), z={mp['z']*1000:.1f} mm; "
      f"x_np(VLM)={r['x_np']*1000:.1f} mm ({np_mac*100:.1f} % MAC); SM={((r['x_np']-mp['x'])/g.MAC)*100:.1f} %")
    P(f"Tail: S_h={g.S_h*1e4:.1f} cm2 (b_h={g.b_h*1000:.0f} mm, c_root={g.c_h_root*1000:.0f}, c_tip={g.c_h_root*g.tap_h*1000:.0f}), "
      f"S_v={g.S_v*1e4:.1f} cm2 (h={g.h_v*1000:.0f}, c_root={g.c_v_root*1000:.0f}, c_tip={g.c_v_root*g.tap_v*1000:.0f}); "
      f"l_h={tv['l_h']:.3f} m, l_v={tv['l_v']:.3f} m, V_H={tv['VH']:.3f}, V_V={tv['VV']:.4f}")
    P(f"Inertia about CG [kg m2]: Ixx={mp['Ixx']:.4f} Iyy={mp['Iyy']:.4f} Izz={mp['Izz']:.4f} Ixz={mp['Ixz']:.5f}")
    P(f"Spar: {r['spar']}")
    rows = grouped(r["items"])

    # ------------------- battery travel -> CG range -------------------
    xb0 = g.x_fw + 0.090
    batt_range = []
    for dx in (-0.025, 0.0, 0.025):
        it2, _ = mass_layout(g, RHO_DESIGN, Equip(), x_batt=xb0 + dx, W_guess=mp["m"] * G)
        m2 = mass_props(it2)
        batt_range.append(dict(dx_batt=dx, x_cg=m2["x"], SM=(r["x_np"] - m2["x"]) / g.MAC))
    P("Battery travel +/-25 mm -> CG " + ", ".join(f"{b['x_cg']*1000:.1f} mm (SM {b['SM']*100:.1f} %)" for b in batt_range))

    # ------------------- density cases (geometry frozen) -------------------
    dens = []
    for rho_fil in (650.0, 750.0, 850.0):
        it_d, sp_d = mass_layout(g, rho_fil, Equip(), x_batt=xb0, W_guess=mp["m"] * G)
        md = mass_props(it_d)
        # battery position to restore the design CG
        fb = lambda xb: mass_props(mass_layout(g, rho_fil, Equip(), x_batt=xb, W_guess=md["m"] * G)[0])["x"] - mp["x"]
        try:
            xb_trim = brentq(fb, xb0 - 0.15, xb0 + 0.15)
        except ValueError:
            xb_trim = float("nan")
        cs = critical_section(g, mp["x"], md["z"], md["m"] * G, V0=Vs)
        wing_m = sum(x["m"] for x in grouped(it_d) if x["group"] == "wing")
        dens.append(dict(rho=rho_fil / 1000, m=md["m"], WS=md["m"] * G / g.S, wing_mass=wing_m,
                         x_cg_same_batt=md["x"], SM_same_batt=(r["x_np"] - md["x"]) / g.MAC,
                         dx_batt_to_retrim=xb_trim - xb0, CLmax=cs["CLmax"], V_stall=cs["V_stall"],
                         Ixx=md["Ixx"], Iyy=md["Iyy"], Izz=md["Izz"], Ixz=md["Ixz"]))
        P(f"density {rho_fil/1000:.2f}: AUW {md['m']*1000:.1f} g (wing {wing_m*1000:.1f} g), W/S {md['m']*G/g.S:.2f} N/m2, "
          f"CG same battery {md['x']*1000:.1f} mm (SM {(r['x_np']-md['x'])/g.MAC*100:.1f} %), battery shift to re-trim "
          f"{(xb_trim-xb0)*1000:+.1f} mm, CLmax {cs['CLmax']:.3f}, Vs {cs['V_stall']:.2f} m/s")

    # ------------------- 3S 850 option -------------------
    eq850 = Equip(batt=0.072, batt_name="3S 850 mAh")
    it8, _ = mass_layout(g, RHO_DESIGN, eq850, x_batt=xb0, W_guess=mp["m"] * G)
    m8 = mass_props(it8)
    fb = lambda xb: mass_props(mass_layout(g, RHO_DESIGN, eq850, x_batt=xb, W_guess=m8["m"] * G)[0])["x"] - mp["x"]
    xb8 = brentq(fb, xb0 - 0.15, xb0 + 0.15)
    P(f"3S 850 option: AUW {m8['m']*1000:.1f} g, W/S {m8['m']*G/g.S:.2f}; battery shift to keep CG {1000*(xb8-xb0):+.1f} mm")

    out = dict(geo=asdict(g), derived=dict(S=g.S, AR=g.AR, MAC=g.MAC, y_mac=g.y_mac, x_mac_le=g.x_mac_le,
                                           b_h=g.b_h, c_h_root=g.c_h_root, mac_h=g.mac_h, h_v=g.h_v,
                                           c_v_root=g.c_v_root, mac_v=g.mac_v, x_h_ac=g.x_h_ac, x_v_ac=g.x_v_ac,
                                           x_tail_end=g.x_tail_end, pod_wetted=pod_wetted_area(g)),
               mass=dict(m=mp["m"], x_cg=mp["x"], z_cg=mp["z"], Ixx=mp["Ixx"], Iyy=mp["Iyy"], Izz=mp["Izz"],
                         Ixz=mp["Ixz"], cg_mac=cg_mac, x_batt=xb0),
               np=dict(x_np_vlm=r["x_np"], np_mac=np_mac, SM_vlm=(r["x_np"] - mp["x"]) / g.MAC, Cma=r["Cma"], CLa=r["CLa"]),
               tail=tv, spar=r["spar"], components=rows, battery_range=batt_range, density_cases=dens,
               crit=dict(CLmax=outer[-1]["CLmax"], V_stall=outer[-1]["Vs"]),
               loop_history=r["hist"], outer=outer, batt850=dict(m=m8["m"], WS=m8["m"] * G / g.S, dx_batt=xb8 - xb0),
               items=r["items"])
    save_json("sizing.json", out)
    with open(os.path.join(RES, "s1_sizing_log.txt"), "w") as f:
        f.write("\n".join(log))
    plots(g, out)


def plots(g, out):
    C = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6", "#e34948"]
    plt.rcParams.update({"font.size": 9, "axes.edgecolor": "#888", "axes.grid": True, "grid.color": "#e4e4e4",
                         "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb"})
    # ---------- planform + side view ----------
    fig, ax = plt.subplots(2, 1, figsize=(10, 7.5), gridspec_kw=dict(height_ratios=[2.2, 1]))
    a = ax[0]
    ys = np.array(g.wing_stations())
    for sgn in (1, -1):
        le = [g.x_le(y) for y in ys]
        te = [g.x_le(y) + g.chord(y) for y in ys]
        a.plot(sgn * ys, le, color="#222", lw=1.2)
        a.plot(sgn * ys, te, color="#222", lw=1.2)
        a.plot([sgn * g.s] * 2, [le[-1], te[-1]], color="#222", lw=1.2)
        # ailerons
        ya = np.linspace(g.ail_y0, g.ail_y1, 20)
        a.fill_between(sgn * ya, [g.x_le(y) + (1 - g.ail_cf) * g.chord(y) for y in ya],
                       [g.x_le(y) + g.chord(y) for y in ya], color=C[0], alpha=0.25, lw=0)
        for yj in (0.2333, 0.4667):
            a.plot([sgn * yj] * 2, [g.x_le(yj), g.x_le(yj) + g.chord(yj)], color="#999", lw=0.8, ls="--")
        # spar
        a.plot(sgn * ys, [g.x_le(y) + 0.27 * g.chord(y) for y in ys], color=C[3], lw=1)
        # stab
        yh = np.array([0, g.b_h / 2])
        ch = np.array([g.c_h_root, g.c_h_root * g.tap_h])
        xle_h = g.x_h_le + 0.25 * (g.c_h_root - ch)
        a.plot(sgn * yh, xle_h, color="#222", lw=1.2)
        a.plot(sgn * yh, xle_h + ch, color="#222", lw=1.2)
        a.plot([sgn * yh[1]] * 2, [xle_h[1], xle_h[1] + ch[1]], color="#222", lw=1.2)
        a.fill_between(sgn * yh, xle_h + (1 - g.cf_e) * ch, xle_h + ch, color=C[1], alpha=0.25, lw=0)
    # pod / boom top view
    pp = pod_profile(g)
    a.plot([r for x, r in pp], [x for x, r in pp], color="#555", lw=1)
    a.plot([-r for x, r in pp], [x for x, r in pp], color="#555", lw=1)
    a.plot([g.r_boom, g.r_boom], [0.21, g.x_tail_end], color="#555", lw=1)
    a.plot([-g.r_boom, -g.r_boom], [0.21, g.x_tail_end], color="#555", lw=1)
    m = out["mass"]
    a.plot(0, m["x_cg"], "o", ms=8, color=C[2], label=f"CG {m['x_cg']*1000:.0f} mm ({m['cg_mac']*100:.0f} % MAC)")
    a.plot(0, out["np"]["x_np_vlm"], "^", ms=8, color=C[7], label=f"NP (VLM) {out['np']['x_np_vlm']*1000:.0f} mm")
    a.plot([g.y_mac, g.y_mac], [g.x_mac_le, g.x_mac_le + g.MAC], color=C[6], lw=2, label=f"MAC {g.MAC*1000:.0f} mm")
    a.invert_yaxis()
    a.set_aspect("equal")
    a.set_xlabel("y [m]")
    a.set_ylabel("x [m] (aft +)")
    a.set_title(f"Concept A planform: b {g.b:.2f} m, S {g.S*100:.2f} dm², AR {g.AR:.2f}, S_h {g.S_h*1e4:.0f} cm², S_v {g.S_v*1e4:.0f} cm² "
                "(blue = ailerons, orange = elevator, dashed = print joints, yellow = spar)", fontsize=8.5)
    a.legend(loc="lower right", fontsize=8)
    # side view with components
    b = ax[1]
    b.plot([x for x, r in pp], [g.z_pod + r for x, r in pp], color="#555", lw=1)
    b.plot([x for x, r in pp], [g.z_pod - r for x, r in pp], color="#555", lw=1)
    b.plot([0.12, g.x_tail_end], [g.z_boom] * 2, color="#555", lw=3)
    b.plot([0, g.c_r], [0, 0], color="#222", lw=3)
    b.plot([g.x_h_le, g.x_h_le + g.c_h_root], [g.z_boom + 0.006] * 2, color="#222", lw=2)
    zv0 = g.z_boom + 0.01
    b.plot([g.x_v_le, g.x_v_le + g.h_v * math.tan(math.radians(g.sweep_v)),
            g.x_v_le + g.h_v * math.tan(math.radians(g.sweep_v)) + g.c_v_root * g.tap_v, g.x_v_le + g.c_v_root],
           [zv0, zv0 + g.h_v, zv0 + g.h_v, zv0], color="#222", lw=1.2)
    for row in out["components"]:
        if row["group"] in ("avionics", "propulsion", "battery") and row["y_abs"] < 0.05:
            b.plot(row["x"], row["z"], "s", ms=4 + 40 * row["m"], color=C[0] if row["group"] == "avionics" else
                   (C[1] if row["group"] == "propulsion" else C[3]), alpha=0.8)
    b.plot(m["x_cg"], m["z_cg"], "o", ms=8, color=C[2])
    b.set_aspect("equal")
    b.set_xlabel("x [m]")
    b.set_ylabel("z [m]")
    b.set_title("Side view: squares = avionics (blue), propulsion (orange), battery (yellow); green = CG", fontsize=8.5)
    fig.tight_layout()
    fig.savefig(os.path.join(PLOTS, "A_layout.png"), dpi=140)
    plt.close(fig)

    # ---------- mass breakdown bars ----------
    grp = {}
    for row in out["components"]:
        grp[row["group"]] = grp.get(row["group"], 0) + row["m"]
    fig, ax = plt.subplots(1, 2, figsize=(11, 4))
    names = list(grp.keys())
    vals = [grp[k] * 1000 for k in names]
    o = np.argsort(vals)
    ax[0].barh([names[i] for i in o], [vals[i] for i in o], color=C[0], height=0.6)
    for i, j in enumerate(o):
        ax[0].text(vals[j] + 3, i, f"{vals[j]:.0f} g", va="center", fontsize=8)
    ax[0].set_xlabel("mass [g]")
    ax[0].set_title(f"Mass by group, AUW {out['mass']['m']*1000:.0f} g at 0.75 g/cm³")
    d = out["density_cases"]
    xs = [c["rho"] for c in d]
    ax[1].bar([f"{c['rho']:.2f}" for c in d], [c["m"] * 1000 for c in d], color=[C[2], C[0], C[1]], width=0.5)
    for i, c in enumerate(d):
        ax[1].text(i, c["m"] * 1000 + 5, f"{c['m']*1000:.0f} g\nW/S {c['WS']:.1f} N/m²\nVs {c['V_stall']:.2f} m/s",
                   ha="center", fontsize=8)
    ax[1].set_ylim(0, max(c["m"] for c in d) * 1000 * 1.25)
    ax[1].set_xlabel("PLA Aero printed density [g/cm³]")
    ax[1].set_ylabel("AUW [g]")
    ax[1].set_title("AUW vs filament density (geometry frozen)")
    fig.tight_layout()
    fig.savefig(os.path.join(PLOTS, "A_mass.png"), dpi=140)
    plt.close(fig)


if __name__ == "__main__":
    main()
