"""
Post-processing for airfoil_study.py:
  1. Ranking robustness: re-score wing candidates under several weight sets and Re bands.
  2. Polar tables (Re 60k/80k/100k) for the top picks -> results/polar_tables.csv / .md
  3. Plots for the top 6 at Re 60k and 100k (Cl-a, Cd-Cl, L/D-a, Cl^1.5/Cd-a, Cm-a)
  4. Tail plots.
  5. XFLR5-ready Selig .dat files (and 3D-print TE variants).
  6. Reynolds-number grid.
Run after airfoil_study.py:  /opt/anaconda3/bin/python3 postprocess.py
"""
import csv, json, os, warnings
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import aerosandbox as asb

warnings.filterwarnings("ignore")
import airfoil_study as S

HERE = os.path.dirname(os.path.abspath(__file__))
RES = os.path.join(HERE, "results"); PLT = os.path.join(HERE, "plots"); DAT = os.path.join(HERE, "dat")
for d in (PLT, DAT):
    os.makedirs(d, exist_ok=True)

D = json.load(open(os.path.join(RES, "polars.json")))
A = np.array(D["alpha"]); GEO = D["geo"]; P = D["polars"]
rows = list(csv.DictReader(open(os.path.join(RES, "metrics_all.csv"))))
R = {(r["name"], int(r["ncrit"]), int(r["Re"])): {k: (float(v) if k not in ("name", "still_rising") else v) for k, v in r.items()} for r in rows}
wing = [n for n in S.WING + list(S.PLATES_WING) if n in GEO]

# ------------------------------------------------------------------ 1. ranking robustness
WEIGHTSETS = {
    "baseline (slow, efficient, buildable)": dict(Emax=0.20, LDmax=0.075, Cd_op=0.10, Clmax=0.15, drop=0.10, Cm=0.075, build=0.15, robust=0.15),
    "equal weights": dict(Emax=1, LDmax=1, Cd_op=1, Clmax=1, drop=1, Cm=1, build=1, robust=1),
    "efficiency-heavy": dict(Emax=0.30, LDmax=0.15, Cd_op=0.15, Clmax=0.10, drop=0.05, Cm=0.05, build=0.10, robust=0.10),
    "safety/low-speed-heavy": dict(Emax=0.15, LDmax=0.05, Cd_op=0.05, Clmax=0.25, drop=0.20, Cm=0.05, build=0.10, robust=0.15),
    "build/robustness-heavy": dict(Emax=0.15, LDmax=0.05, Cd_op=0.10, Clmax=0.10, drop=0.10, Cm=0.05, build=0.25, robust=0.20),
}
REBANDS = {"Re 60-80k": (60000, 80000), "Re 40-60k": (40000, 60000), "Re 80-100k": (80000, 100000)}


def raw_metrics(res):
    out = {}
    for n in wing:
        g = lambda k, nc=9: np.mean([R[(n, nc, re)][k] for re in res])
        E9, E7 = g("Emax"), g("Emax", 7)
        out[n] = dict(Emax=E9, LDmax=g("LDmax"),
                      Cd_op=np.nanmean([g("Cd_cl04"), g("Cd_cl07"), g("Cd_cl10")]),
                      Clmax=g("Clmax"), drop=g("stall_drop3"), Cm=abs(g("Cm_design")),
                      build=S.build_score(GEO[n]["t"]), ncrit_sens=abs(E7 - E9) / E9,
                      lowRe=R[(n, 9, 40000)]["Emax"] / R[(n, 9, 80000)]["Emax"])
    return out


def score(raw, W):
    tot = sum(W.values())
    def norm(key, hb=True):
        v = np.array([raw[n][key] for n in wing], float)
        v = np.where(np.isnan(v), np.nanmin(v) if hb else np.nanmax(v), v)
        lo, hi = np.percentile(v, 10), np.percentile(v, 90)
        s = np.clip((v - lo) / (hi - lo + 1e-12), 0, 1)
        return dict(zip(wing, s if hb else 1 - s))
    s = {k: norm(k, hb) for k, hb in [("Emax", 1), ("LDmax", 1), ("Cd_op", 0), ("Clmax", 1), ("drop", 0), ("Cm", 0), ("ncrit_sens", 0), ("lowRe", 1)]}
    out = {}
    for n in wing:
        rob = 0.5 * s["ncrit_sens"][n] + 0.5 * s["lowRe"][n]
        v = (W["Emax"] * s["Emax"][n] + W["LDmax"] * s["LDmax"][n] + W["Cd_op"] * s["Cd_op"][n] + W["Clmax"] * s["Clmax"][n]
             + W["drop"] * s["drop"][n] + W["Cm"] * s["Cm"][n] + W["build"] * raw[n]["build"] + W["robust"] * rob)
        out[n] = 100 * v / tot
    return out


ranks = {}
for bname, band in REBANDS.items():
    raw = raw_metrics(band)
    for wname, W in WEIGHTSETS.items():
        if bname != "Re 60-80k" and wname != "baseline (slow, efficient, buildable)":
            continue
        sc = score(raw, W)
        order = sorted(wing, key=lambda n: -sc[n])
        ranks[f"{wname} | {bname}"] = {n: (order.index(n) + 1, sc[n]) for n in wing}
cases = list(ranks)
mean_rank = {n: np.mean([ranks[c][n][0] for c in cases]) for n in wing}
final = sorted(wing, key=lambda n: mean_rank[n])
with open(os.path.join(RES, "ranking_robustness.csv"), "w", newline="") as f:
    w = csv.writer(f); w.writerow(["airfoil", "mean_rank"] + cases)
    for n in final:
        w.writerow([S.label(n), round(mean_rank[n], 2)] + [ranks[c][n][0] for c in cases])
print("RANKING ROBUSTNESS (rank under each weighting / Re band)")
print(f"{'airfoil':22s} {'mean':>5} " + " ".join(f"{i+1:>3}" for i in range(len(cases))))
for n in final[:20]:
    print(f"{S.label(n):22s} {mean_rank[n]:5.1f} " + " ".join(f"{ranks[c][n][0]:3d}" for c in cases))
for i, c in enumerate(cases):
    print(f"  case {i+1}: {c}")

# ------------------------------------------------------------------ 2. polar tables
# top 6 by mean rank across all weighting / Re cases + SD7037 (best wind-tunnel-validated reference)
TOP6 = ["s7075", "naca4409", "s4061", "hq259", "sa7038", "e387", "sd7037"]
DAT_WING = ["s7075", "naca4409", "sd7037"]
TAIL_PICK = "ht14"
TAB_ALPHA = [-2, 0, 2, 3, 4, 5, 6, 7, 8, 10, 12]


def at(p, key, a):
    return float(np.interp(a, A, np.array(p[key])))


lines = ["| Airfoil | Re | " + " | ".join(f"a={a}" for a in TAB_ALPHA) + " |",
         "|---|---|" + "---|" * len(TAB_ALPHA)]
with open(os.path.join(RES, "polar_tables.csv"), "w", newline="") as f:
    w = csv.writer(f); w.writerow(["airfoil", "Re", "ncrit", "alpha_deg", "Cl", "Cd", "Cm", "L/D", "Cl^1.5/Cd", "confidence"])
    for n in TOP6 + [TAIL_PICK, "naca0009", "flatplate_t3"]:
        for Re in (40000, 60000, 80000, 100000):
            p = P[n][f"9_{Re}"]
            for a in TAB_ALPHA:
                cl, cd, cm = at(p, "CL", a), at(p, "CD", a), at(p, "CM", a)
                w.writerow([S.label(n), Re, 9, a, round(cl, 4), round(cd, 5), round(cm, 4), round(cl / cd, 2),
                            round(max(cl, 0) ** 1.5 / cd, 2), round(at(p, "analysis_confidence", a), 3)])
            if Re != 40000:
                lines.append(f"| {S.label(n)} | {Re//1000}k | " + " | ".join(
                    f"{at(p,'CL',a):.2f} / {at(p,'CD',a)*1e4:.0f} / {at(p,'CM',a):+.3f}" for a in TAB_ALPHA) + " |")
open(os.path.join(RES, "polar_tables.md"), "w").write(
    "Cells: Cl / Cd x 1e4 / Cm(c/4), NeuralFoil xlarge, Ncrit 9, Mach 0\n\n" + "\n".join(lines) + "\n")

# metrics summary table for the top picks
with open(os.path.join(RES, "top_metrics.md"), "w") as f:
    f.write("| Airfoil | Re | Clmax | a_stall | Cl drop 3deg post-stall | Cdmin | (L/D)max @Cl | (Cl^1.5/Cd)max @Cl | Cd@Cl0.4 | Cd@Cl0.7 | Cd@Cl1.0 | Cm(a=0) | Cm@Cl0.7 | a_L0 | Cla /rad | conf |\n")
    f.write("|---" * 16 + "|\n")
    for n in TOP6:
        for Re in (40000, 60000, 80000, 100000, 150000):
            m = R[(n, 9, Re)]
            f.write(f"| {S.label(n)} | {Re//1000}k | {m['Clmax']:.3f} | {m['alpha_stall']:.1f} | {m['stall_drop3']:.2f} | {m['Cdmin']:.4f} | "
                    f"{m['LDmax']:.1f} @{m['Cl_LDmax']:.2f} | {m['Emax']:.1f} @{m['Cl_Emax']:.2f} | {m['Cd_cl04']:.4f} | {m['Cd_cl07']:.4f} | "
                    f"{m['Cd_cl10']:.4f} | {m['Cm0']:+.3f} | {m['Cm_design']:+.3f} | {m['alpha_L0']:.2f} | {m['Cla_per_rad']:.2f} | {m['conf_mean']:.2f} |\n")

# ------------------------------------------------------------------ 3. plots
# categorical palette (validated reference palette, fixed order) + line styles as secondary encoding
COL = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#6250d6", "#e34948"]
LS = ["-", "--", "-.", ":", (0, (5, 1, 1, 1)), (0, (3, 1, 1, 1, 1, 1))]
plt.rcParams.update({"font.size": 9, "axes.edgecolor": "#888", "axes.labelcolor": "#222", "xtick.color": "#444",
                     "ytick.color": "#444", "axes.grid": True, "grid.color": "#e6e6e6", "grid.linewidth": 0.6,
                     "axes.spines.top": False, "axes.spines.right": False, "legend.frameon": False})


def stall_cut(p):
    """Index slice up to 4 deg past stall (keeps plots readable)."""
    cl = np.array(p["CL"]); i = int(np.argmax(cl))
    return slice(0, min(len(A), np.searchsorted(A, A[i] + 4.0)))


def polar_figure(names, Re, fname, title):
    fig, ax = plt.subplots(2, 3, figsize=(15, 8.6))
    ax = ax.ravel()
    for k, n in enumerate(names):
        p = P[n][f"9_{Re}"]; s = stall_cut(p)
        cl, cd, cm = np.array(p["CL"])[s], np.array(p["CD"])[s], np.array(p["CM"])[s]; a = A[s]
        kw = dict(color=COL[k % 8], ls=LS[k % 6], lw=2, label=S.label(n))
        ax[0].plot(a, cl, **kw)
        ax[1].plot(cd, cl, **kw)
        ax[2].plot(a, np.where(cl > 0.05, cl / cd, np.nan), **kw)
        ax[3].plot(a, np.where(cl > 0.05, np.clip(cl, 0, None) ** 1.5 / cd, np.nan), **kw)
        ax[4].plot(a, cm, **kw)
        ax[5].plot(A, P[n][f"9_{Re}"]["analysis_confidence"], **kw)
    for a_, xl, yl, t in zip(ax, ["alpha [deg]", "Cd", "alpha [deg]", "alpha [deg]", "alpha [deg]", "alpha [deg]"],
                             ["Cl", "Cl", "Cl/Cd", "Cl^1.5/Cd (endurance / min-sink)", "Cm (c/4)", "NeuralFoil analysis_confidence"],
                             ["Lift curve", "Drag polar", "Lift-to-drag", "Endurance parameter", "Pitching moment", "Surrogate confidence"]):
        a_.set_xlabel(xl); a_.set_ylabel(yl); a_.set_title(t, loc="left", fontsize=10)
    ax[1].set_xlim(0, 0.06)
    ax[0].legend(loc="lower right", fontsize=8)
    fig.suptitle(title, x=0.01, ha="left", fontsize=12)
    fig.tight_layout(); fig.savefig(os.path.join(PLT, fname), dpi=140); plt.close(fig)


for Re in (60000, 100000):
    polar_figure(TOP6, Re, f"top6_polars_Re{Re//1000}k.png",
                 f"Top-6 wing airfoils, Re = {Re//1000}k, Ncrit 9 (NeuralFoil xlarge, XFOIL surrogate)")
polar_figure(["s7075", "sd7037", "curvedplate_c6_t3", "curvedplate_c4_t3", "goe417a", "flatplate_t3"], 60000,
             "plates_vs_airfoils_Re60k.png", "Sheet/plate profiles vs conventional sections, Re = 60k (plate results LOW confidence)")
polar_figure(["ht14", "ht13", "naca0009", "naca0008", "flatplate_t3", "sd8020"], 50000 if "9_50000" in P["ht14"] else 40000,
             "tail_polars_Re40k.png", "Tail candidates, Re = 40k, Ncrit 9")
polar_figure(["ht14", "ht13", "naca0009", "naca0008", "flatplate_t3", "sd8020"], 60000,
             "tail_polars_Re60k.png", "Tail candidates, Re = 60k, Ncrit 9")

# Re-sweep for the recommended wing airfoil
fig, ax = plt.subplots(1, 3, figsize=(15, 4.4))
for k, Re in enumerate((40000, 60000, 80000, 100000, 150000)):
    p = P["s7075"][f"9_{Re}"]; s = stall_cut(p); cl, cd = np.array(p["CL"])[s], np.array(p["CD"])[s]
    kw = dict(color=COL[k], ls=LS[k], lw=2, label=f"Re {Re//1000}k")
    ax[0].plot(A[s], cl, **kw); ax[1].plot(cd, cl, **kw); ax[2].plot(A[s], np.where(cl > 0.05, cl ** 1.5 / cd, np.nan), **kw)
for a_, xl, yl in zip(ax, ["alpha [deg]", "Cd", "alpha [deg]"], ["Cl", "Cl", "Cl^1.5/Cd"]):
    a_.set_xlabel(xl); a_.set_ylabel(yl)
ax[1].set_xlim(0, 0.05); ax[0].legend(fontsize=8)
fig.suptitle("S7075 Reynolds-number sweep (Ncrit 9)", x=0.01, ha="left"); fig.tight_layout()
fig.savefig(os.path.join(PLT, "s7075_Re_sweep.png"), dpi=140); plt.close(fig)

# geometry plot
fig, ax = plt.subplots(figsize=(11, 8))
show = TOP6 + ["ht14", "curvedplate_c6_t3"]
for k, n in enumerate(show):
    af = S.load(n); c = af.coordinates
    ax.plot(c[:, 0], c[:, 1] - 0.13 * k, color=COL[k % 8], lw=1.6)
    ax.text(1.03, -0.13 * k, f"{S.label(n)}  t={GEO[n]['t']*100:.1f}%  f={GEO[n]['camber']*100:.1f}%", va="center", fontsize=8.5)
ax.set_aspect("equal"); ax.set_xlim(-0.02, 1.55); ax.axis("off")
ax.set_title("Section shapes (offset vertically)", loc="left")
fig.tight_layout(); fig.savefig(os.path.join(PLT, "geometries.png"), dpi=140); plt.close(fig)

# ------------------------------------------------------------------ 5. dat files
def write_dat(af, name, path):
    c = af.coordinates
    with open(path, "w") as f:
        f.write(name + "\n")
        for x, y in c:
            f.write(f"{x: .6f} {y: .6f}\n")
    return len(c)


print("\nDAT files")
for n in DAT_WING + [TAIL_PICK, "flatplate_t3"]:
    if n in S.PLATES_WING or n in S.PLATES_TAIL:
        af = S.load(n); nm = "FLATPLATE-3pct" if n == "flatplate_t3" else n
    else:
        af = asb.Airfoil(n).repanel(n_points_per_side=75)   # 149 pts, cosine spacing
        nm = n.upper()
    npts = write_dat(af, nm, os.path.join(DAT, f"{nm.lower()}.dat"))
    msg = f"  {nm:16s} {npts} pts  t={af.max_thickness()*100:.2f}%  TE={af.TE_thickness()*100:.3f}%c"
    if n in DAT_WING + [TAIL_PICK]:
        # printable variant: 0.5 % chord blunt TE (~0.7 mm at 140 mm chord)
        af2 = af.set_TE_thickness(0.005)
        write_dat(af2, nm + "-TE05", os.path.join(DAT, f"{nm.lower()}_te05.dat"))
        p0 = af.get_aero_from_neuralfoil(alpha=np.array([0, 4, 8.0]), Re=70000, n_crit=9, model_size="xlarge")
        p1 = af2.get_aero_from_neuralfoil(alpha=np.array([0, 4, 8.0]), Re=70000, n_crit=9, model_size="xlarge")
        msg += "  | TE0.5% effect @Re70k a=0/4/8: dCd=" + "/".join(f"{(b-a)*1e4:+.0f}" for a, b in zip(p0["CD"], p1["CD"])) + " e-4"
        msg += "  dCl=" + "/".join(f"{(b-a):+.3f}" for a, b in zip(p0["CL"], p1["CL"]))
    print(msg)

# ------------------------------------------------------------------ 6. Re grid
print("\nRe = V c / nu  (nu = 1.7e-5)")
chords = [0.08, 0.10, 0.12, 0.14, 0.16, 0.18, 0.20]
print("V\\c  " + " ".join(f"{c:6.2f}" for c in chords))
with open(os.path.join(RES, "re_grid.csv"), "w", newline="") as f:
    w = csv.writer(f); w.writerow(["V_mps"] + chords)
    for V in (5, 6, 7, 8, 9, 10, 12):
        row = [V * c / S.NU for c in chords]; w.writerow([V] + [int(round(r, -2)) for r in row])
        print(f"{V:3d}  " + " ".join(f"{r/1e3:5.0f}k" for r in row))
