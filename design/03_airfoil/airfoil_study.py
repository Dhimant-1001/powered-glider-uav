"""
Airfoil selection study for the Powered Glider UAV (EC4.402, IIIT Hyderabad).

Runs NeuralFoil (XFOIL-trained neural surrogate, via AeroSandbox) over a candidate
list at low Reynolds numbers, extracts selection metrics, scores candidates and
writes CSV tables + JSON with full polars.

Run:  /opt/anaconda3/bin/python3 airfoil_study.py
Outputs (same folder): results/*.csv, results/polars.json
"""
import json, os, warnings
import numpy as np
import aerosandbox as asb

warnings.filterwarnings("ignore")
HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "results")
os.makedirs(OUT, exist_ok=True)

NU = 1.7e-5                       # m^2/s (Hyderabad, 540 m, 30 C)
RE_LIST = [40e3, 60e3, 80e3, 100e3, 150e3]
ALPHA = np.arange(-6.0, 16.0 + 1e-9, 0.25)
NCRIT_LIST = [9, 7]
MODEL = "xlarge"
CL_DESIGN = 0.7                    # representative section Cl at slow cruise (V~7 m/s)
CL_OP = (0.4, 0.7, 1.0)            # fast cruise / slow cruise / slow 30-35 deg bank turn
STALL_MARGIN_DEG = 2.0             # L/D and Cl^1.5/Cd only counted >= 2 deg below stall


# ----------------------------------------------------------------- geometry helpers
def plate(name, t=0.03, camber=0.0, n=80, te_taper=0.15, le_round=True):
    """Constant-thickness sheet (foam / balsa) with a round LE, linearly sanded
    TE over the last `te_taper` of chord, optional circular-arc camber line."""
    beta = np.linspace(0, np.pi, n)
    x = 0.5 * (1 - np.cos(beta))                      # cosine spacing
    # half-thickness: round nose (ellipse over first t of chord) then constant, then taper
    h = np.full_like(x, t / 2)
    if le_round:
        xr = t / 2
        m = x < xr
        h[m] = (t / 2) * np.sqrt(np.clip(1 - ((xr - x[m]) / xr) ** 2, 0, 1))
    m = x > 1 - te_taper
    h[m] = (t / 2) * (1 - (x[m] - (1 - te_taper)) / te_taper) + 0.0015 * ((x[m] - (1 - te_taper)) / te_taper)
    h = np.maximum(h, 0.0)
    # circular-arc camber line, max camber `camber` at 50 %
    if camber > 0:
        R = (0.25 + camber ** 2) / (2 * camber)
        yc = np.sqrt(np.maximum(R ** 2 - (x - 0.5) ** 2, 0)) - (R - camber)
        dyc = -(x - 0.5) / np.sqrt(np.maximum(R ** 2 - (x - 0.5) ** 2, 1e-12))
    else:
        yc = np.zeros_like(x); dyc = np.zeros_like(x)
    th = np.arctan(dyc)
    xu, yu = x - h * np.sin(th), yc + h * np.cos(th)
    xl, yl = x + h * np.sin(th), yc - h * np.cos(th)
    coords = np.vstack([np.column_stack([xu[::-1], yu[::-1]]), np.column_stack([xl[1:], yl[1:]])])
    return asb.Airfoil(name=name, coordinates=coords)


# ----------------------------------------------------------------- candidates
WING = ["ag35", "ag36", "ag37", "ag455ct-02r", "ag24", "ag25", "sd7037", "sd7032", "sd7062", "sd7080",
        "sd7003", "sd7084", "s3021", "s4083", "s4061", "s7012", "s7075", "sa7036", "sa7038",
        "e387", "e205", "e193", "e214", "mh32", "mh30", "rg15", "hq259", "clarky",
        "naca2412", "naca4412", "naca2409", "naca4409", "s1223", "goe417a", "dae11", "ah79100c"]
TAIL = ["naca0006", "naca0008", "naca0009", "ht08", "ht12", "ht13", "ht14", "sd8020"]
PLATES_WING = {
    "curvedplate_c4_t3": dict(t=0.03, camber=0.04),
    "curvedplate_c6_t3": dict(t=0.03, camber=0.06),
    "curvedplate_c4_t5": dict(t=0.05, camber=0.04),
    "flatplate_t3": dict(t=0.03, camber=0.0),
}
PLATES_TAIL = {"flatplate_t3": dict(t=0.03, camber=0.0), "flatplate_t5": dict(t=0.05, camber=0.0)}

LABEL = {"ag455ct-02r": "AG455ct-02r", "clarky": "Clark Y", "goe417a": "GOE 417a",
         "curvedplate_c4_t3": "Curved plate 4%c/3%t", "curvedplate_c6_t3": "Curved plate 6%c/3%t",
         "curvedplate_c4_t5": "Curved plate 4%c/5%t", "flatplate_t3": "Flat plate 3%t",
         "flatplate_t5": "Flat plate 5%t", "hq259": "HQ/W 2.5/9"}


def label(n):
    return LABEL.get(n, n.upper().replace("NACA", "NACA "))


def load(name):
    if name in PLATES_WING:
        return plate(name, **PLATES_WING[name])
    if name in PLATES_TAIL:
        return plate(name, **PLATES_TAIL[name])
    af = asb.Airfoil(name)
    return af.repanel(n_points_per_side=80)


# ----------------------------------------------------------------- metrics
def polar(af, Re, ncrit):
    a = af.get_aero_from_neuralfoil(alpha=ALPHA, Re=Re, mach=0.0, n_crit=ncrit, model_size=MODEL)
    return {k: np.asarray(a[k]).ravel() for k in ["CL", "CD", "CM", "analysis_confidence", "Top_Xtr", "Bot_Xtr"]}


def interp_at_cl(p, cl_target, key):
    """Interpolate `key` at a given Cl on the attached (pre-stall) branch."""
    i_max = int(np.argmax(p["CL"]))
    cl = p["CL"][: i_max + 1]; v = p[key][: i_max + 1]
    # keep monotonic part
    order = np.argsort(cl)
    cl, v = cl[order], v[order]
    if cl_target < cl.min() or cl_target > cl.max():
        return np.nan
    return float(np.interp(cl_target, cl, v))


def metrics(p):
    CL, CD, CM, conf = p["CL"], p["CD"], p["CM"], p["analysis_confidence"]
    i = int(np.argmax(CL))
    clmax, a_st = CL[i], ALPHA[i]
    # stall gentleness: Cl lost 3 deg after the peak (if available) -> smaller = gentler
    j = np.searchsorted(ALPHA, a_st + 3.0)
    drop3 = clmax - CL[min(j, len(CL) - 1)]   # if stall is near 16 deg this is a partial (<3 deg) drop
    pos = CL > 0.05
    LD = np.where(pos, CL / CD, np.nan)
    E = np.where(pos, np.clip(CL, 0, None) ** 1.5 / CD, np.nan)
    # only attached branch for L/D and endurance
    usable = ALPHA <= a_st - STALL_MARGIN_DEG
    LD[~usable] = np.nan; E[~usable] = np.nan
    k = int(np.nanargmax(LD)); m = int(np.nanargmax(E))
    # zero-lift angle and lift slope (linear region)
    a0 = float(np.interp(0.0, CL[: i + 1], ALPHA[: i + 1])) if CL[0] < 0 < clmax else np.nan
    lin = (ALPHA >= -2) & (ALPHA <= 4)
    slope = np.polyfit(ALPHA[lin], CL[lin], 1)[0] * 180 / np.pi
    # mean Cd over operating Cl band 0.3..0.9
    cds = [interp_at_cl(p, c, "CD") for c in CL_OP]
    # confidence over the useful range -2..(stall)
    use = (ALPHA >= -2) & (ALPHA <= a_st)
    return dict(
        Clmax=clmax, alpha_stall=a_st, stall_drop3=drop3, still_rising=bool(i == len(CL) - 1),
        Cdmin=float(np.min(CD)), Cl_at_Cdmin=float(CL[int(np.argmin(CD))]),
        LDmax=float(LD[k]), Cl_LDmax=float(CL[k]), alpha_LDmax=float(ALPHA[k]),
        Emax=float(E[m]), Cl_Emax=float(CL[m]), alpha_Emax=float(ALPHA[m]),
        Cm0=float(np.interp(0.0, ALPHA, CM)), Cm_design=interp_at_cl(p, CL_DESIGN, "CM"),
        alpha_L0=a0, Cla_per_rad=float(slope), Cl_alpha0=float(np.interp(0.0, ALPHA, CL)),
        Cd_cl04=cds[0], Cd_cl07=cds[1], Cd_cl10=cds[2],
        conf_mean=float(np.mean(conf[use])), conf_min=float(np.min(conf[use])),
    )


def build_score(t):
    """Buildability vs thickness: <6% poor, 8.5-11% best, >13% slight drag/weight penalty."""
    return float(np.interp(t, [0.03, 0.06, 0.075, 0.085, 0.11, 0.13, 0.16], [0.15, 0.25, 0.55, 1.0, 1.0, 0.85, 0.6]))


def main():
    allnames = WING + list(PLATES_WING) + [n for n in TAIL] + [n for n in PLATES_TAIL if n not in PLATES_WING]
    rows, polars, geo = [], {}, {}
    for name in allnames:
        try:
            af = load(name)
        except Exception as e:
            print("skip", name, e); continue
        geo[name] = dict(t=float(af.max_thickness()), camber=float(af.max_camber()),
                         x_t=float(af.local_thickness(np.linspace(0, 1, 401)).argmax() / 400))
        polars[name] = {}
        for nc in NCRIT_LIST:
            for Re in RE_LIST:
                p = polar(af, Re, nc)
                polars[name][f"{nc}_{int(Re)}"] = {k: v.round(5).tolist() for k, v in p.items()}
                m = metrics(p)
                rows.append(dict(name=name, ncrit=nc, Re=int(Re), **geo[name], **m))
        print(f"done {name:22s} t={geo[name]['t']*100:5.2f}% c={geo[name]['camber']*100:5.2f}%")

    import csv
    keys = list(rows[0].keys())
    with open(os.path.join(OUT, "metrics_all.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys); w.writeheader(); [w.writerow(r) for r in rows]
    with open(os.path.join(OUT, "polars.json"), "w") as f:
        json.dump(dict(alpha=ALPHA.tolist(), geo=geo, polars=polars), f)

    # ------------------------------------------------------------- scoring (wing)
    R = {(r["name"], r["ncrit"], r["Re"]): r for r in rows}
    wing_names = [n for n in WING + list(PLATES_WING) if n in geo]

    def avg(n, key, res=(60000, 80000), nc=9):
        return np.mean([R[(n, nc, re)][key] for re in res])

    raw = {}
    for n in wing_names:
        E9 = avg(n, "Emax"); E7 = avg(n, "Emax", nc=7)
        raw[n] = dict(
            Emax=E9,
            LDmax=avg(n, "LDmax"),
            Cd_op=np.nanmean([avg(n, "Cd_cl04"), avg(n, "Cd_cl07"), avg(n, "Cd_cl10")]),
            Clmax=avg(n, "Clmax"),
            drop=avg(n, "stall_drop3"),
            Cm=abs(avg(n, "Cm_design")),
            build=build_score(geo[n]["t"]),
            # robustness: Ncrit 7 vs 9 sensitivity + low-Re (40k) degradation relative to 80k
            ncrit_sens=abs(E7 - E9) / E9,
            lowRe=R[(n, 9, 40000)]["Emax"] / R[(n, 9, 80000)]["Emax"],
            conf=avg(n, "conf_mean"),
        )

    def norm(key, higher_better=True):
        v = np.array([raw[n][key] for n in wing_names], float)
        v = np.where(np.isnan(v), np.nanmin(v) if higher_better else np.nanmax(v), v)
        lo, hi = np.percentile(v, 10), np.percentile(v, 90)   # robust to outliers (plates, DAE11)
        s = np.clip((v - lo) / (hi - lo + 1e-12), 0, 1)
        return dict(zip(wing_names, s if higher_better else 1 - s))

    W = dict(Emax=0.20, LDmax=0.075, Cd_op=0.10, Clmax=0.15, drop=0.10, Cm=0.075, build=0.15, robust=0.15)
    sE, sLD, sCd, sCl, sDr, sCm = norm("Emax"), norm("LDmax"), norm("Cd_op", False), norm("Clmax"), norm("drop", False), norm("Cm", False)
    sNs, sLo = norm("ncrit_sens", False), norm("lowRe")
    score_rows = []
    for n in wing_names:
        robust = 0.5 * sNs[n] + 0.5 * sLo[n]
        s = (W["Emax"] * sE[n] + W["LDmax"] * sLD[n] + W["Cd_op"] * sCd[n] + W["Clmax"] * sCl[n]
             + W["drop"] * sDr[n] + W["Cm"] * sCm[n] + W["build"] * raw[n]["build"] + W["robust"] * robust)
        score_rows.append(dict(name=n, label=label(n), score=round(100 * s, 1),
                               t_pct=round(100 * geo[n]["t"], 2), camber_pct=round(100 * geo[n]["camber"], 2),
                               **{k: round(float(v), 4) for k, v in raw[n].items()}))
    score_rows.sort(key=lambda r: -r["score"])
    with open(os.path.join(OUT, "wing_ranking.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(score_rows[0].keys())); w.writeheader(); [w.writerow(r) for r in score_rows]
    print("\nWING RANKING (Re 60k/80k avg, Ncrit 9)")
    print(f"{'#':>2} {'airfoil':22s} {'score':>5} {'t%':>5} {'c%':>5} {'E=Cl^1.5/Cd':>11} {'L/D':>6} {'Cd_op':>7} {'Clmax':>6} {'drop3':>6} {'|Cm|':>6} {'nc-sens':>7} {'40k/80k':>7} {'conf':>5}")
    for i, r in enumerate(score_rows, 1):
        print(f"{i:2d} {r['label']:22s} {r['score']:5.1f} {r['t_pct']:5.2f} {r['camber_pct']:5.2f} {r['Emax']:11.1f} {r['LDmax']:6.1f} {r['Cd_op']:7.4f} {r['Clmax']:6.3f} {r['drop']:6.3f} {r['Cm']:6.3f} {r['ncrit_sens']:7.3f} {r['lowRe']:7.3f} {r['conf']:5.2f}")

    # ------------------------------------------------------------- tail table
    tail_names = [n for n in TAIL + list(PLATES_TAIL) if n in geo]
    tail_rows = []
    for n in tail_names:
        for Re in (40000, 60000, 80000):
            p = polars[n][f"9_{Re}"]
            CL, CD = np.array(p["CL"]), np.array(p["CD"])
            # lift-curve slope near zero (-1..+1 deg) vs (2..5 deg): low-Re dead band indicator
            a = ALPHA
            s0 = np.polyfit(a[(a >= -1) & (a <= 1)], CL[(a >= -1) & (a <= 1)], 1)[0]
            s1 = np.polyfit(a[(a >= 2) & (a <= 5)], CL[(a >= 2) & (a <= 5)], 1)[0]
            r = R[(n, 9, Re)]
            tail_rows.append(dict(name=n, label=label(n), Re=Re, t_pct=round(100 * geo[n]["t"], 2),
                                  Cd0=round(float(np.interp(0, a, CD)), 5),
                                  Cd_cl03=round(float(np.interp(0.3, CL[(a >= 0) & (a <= 8)], CD[(a >= 0) & (a <= 8)])), 5),
                                  Clmax=round(r["Clmax"], 3), alpha_stall=r["alpha_stall"],
                                  slope_0_per_deg=round(float(s0), 4), slope_2_5_per_deg=round(float(s1), 4),
                                  linearity=round(float(s0 / s1), 3), conf=round(r["conf_mean"], 3)))
    with open(os.path.join(OUT, "tail_table.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(tail_rows[0].keys())); w.writeheader(); [w.writerow(r) for r in tail_rows]
    print("\nTAIL (Ncrit 9)")
    for r in tail_rows:
        print(f"{r['label']:16s} Re={r['Re']:6d} t={r['t_pct']:5.2f}% Cd0={r['Cd0']:.4f} Cd@0.3={r["Cd_cl03"]:.4f} Clmax={r['Clmax']:.3f}@{r['alpha_stall']:.1f} "
              f"dCl/da(0)={r['slope_0_per_deg']:.4f} dCl/da(2-5)={r['slope_2_5_per_deg']:.4f} lin={r['linearity']:.2f} conf={r['conf']:.2f}")


if __name__ == "__main__":
    main()
