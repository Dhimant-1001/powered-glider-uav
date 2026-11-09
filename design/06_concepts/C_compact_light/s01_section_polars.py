"""
s01_section_polars.py  --  Concept C (compact & light): section data for the wing and tail.

* Builds the blunt-trailing-edge wing section actually produced by a printed / hybrid wing
  (TE ~0.8-1.0 mm on 115-165 mm chords -> 0.7 % c), from the stage-1 SD7037 / S7075 files.
* NeuralFoil (xlarge) polars at Re 25k-160k, Ncrit 9 (baseline) and 7 (rough field air).
* Section Cl_max(Re) table used by the critical-section CL_max method (s04).
* SD7037 vs S7075 drag in Concept C's operating band (Cl 0.4-0.9, Re 50-90k) to confirm
  the stage-1 critique's choice for this concept's higher wing loading.
Outputs: dat/sd7037_te07.dat, dat/s7075_te07.dat, results/section_polars.npz,
         results/section_summary.md, plots/section_polars.png
"""
import os, json
import numpy as np
import aerosandbox as asb
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
DAT = os.path.join(HERE, "dat")
TE = 0.007

def load(name):
    return asb.Airfoil(name=name, coordinates=os.path.join(DAT, name + ".dat"))

foils = {}
for base in ("sd7037", "s7075"):
    af = load(base).repanel(n_points_per_side=100).set_TE_thickness(TE)
    af.name = f"{base}_te07"
    af.write_dat(os.path.join(DAT, f"{base}_te07.dat"))
    foils[base] = af
fp = load("flatplate-3pct")

alphas = np.arange(-8, 16.01, 0.25)
Res = np.array([25e3, 30e3, 40e3, 50e3, 60e3, 70e3, 80e3, 90e3, 100e3, 120e3, 160e3])
out = {}
summary = {}
for key, af in list(foils.items()) + [("flatplate", fp)]:
    for nc in (9, 7):
        CL = np.zeros((len(Res), len(alphas))); CD = np.zeros_like(CL); CM = np.zeros_like(CL); CF = np.zeros_like(CL)
        for i, Re in enumerate(Res):
            a = af.get_aero_from_neuralfoil(alpha=alphas, Re=Re, n_crit=nc, model_size="xlarge")
            CL[i], CD[i], CM[i], CF[i] = a["CL"], a["CD"], a["CM"], a["analysis_confidence"]
        out[f"{key}_n{nc}_CL"] = CL; out[f"{key}_n{nc}_CD"] = CD; out[f"{key}_n{nc}_CM"] = CM; out[f"{key}_n{nc}_conf"] = CF
        # Cl_max = first local maximum in alpha (pre-stall peak) restricted to alpha <= 15
        clmax = []; amax = []
        for i in range(len(Res)):
            j = int(np.argmax(CL[i])); clmax.append(float(CL[i, j])); amax.append(float(alphas[j]))
        summary[f"{key}_n{nc}"] = dict(Re=Res.tolist(), Clmax=clmax, alpha_clmax=amax)
np.savez(os.path.join(HERE, "results", "section_polars.npz"), alphas=alphas, Res=Res, **out)

def cd_at_cl(key, nc, Re_i, cl):
    CL = out[f"{key}_n{nc}_CL"][Re_i]; CD = out[f"{key}_n{nc}_CD"][Re_i]
    j = int(np.argmax(CL)); return float(np.interp(cl, CL[:j + 1], CD[:j + 1]))

lines = ["# Section data (NeuralFoil xlarge), blunt TE 0.7 % c\n",
         "## Cl_max vs Re (Ncrit 9 / Ncrit 7)\n", "| Re | SD7037-te07 n9 | n7 | S7075-te07 n9 | n7 | flat plate 3 % n9 |", "|---|---|---|---|---|---|"]
for i, Re in enumerate(Res):
    lines.append(f"| {Re/1e3:.0f}k | {summary['sd7037_n9']['Clmax'][i]:.3f} @{summary['sd7037_n9']['alpha_clmax'][i]:.1f} | "
                 f"{summary['sd7037_n7']['Clmax'][i]:.3f} | {summary['s7075_n9']['Clmax'][i]:.3f} @{summary['s7075_n9']['alpha_clmax'][i]:.1f} | "
                 f"{summary['s7075_n7']['Clmax'][i]:.3f} | {summary['flatplate_n9']['Clmax'][i]:.3f} |")
lines += ["\n## Cd (x1e4) in Concept C operating band (Ncrit 9)\n", "| Re | Cl | SD7037-te07 | S7075-te07 | SD7037 advantage |", "|---|---|---|---|---|"]
rows = []
for Re in (50e3, 70e3, 90e3):
    i = int(np.where(Res == Re)[0][0])
    for cl in (0.4, 0.5, 0.6, 0.7, 0.8, 0.9):
        a, b = cd_at_cl("sd7037", 9, i, cl), cd_at_cl("s7075", 9, i, cl)
        rows.append((Re, cl, a, b))
        lines.append(f"| {Re/1e3:.0f}k | {cl:.1f} | {a*1e4:.0f} | {b*1e4:.0f} | {(b-a)/b*100:+.1f} % |")
mean_adv = np.mean([(b - a) / b for _, _, a, b in rows]) * 100
lines.append(f"\nMean SD7037 drag advantage over the band: {mean_adv:.1f} %")
# blunt-TE penalty
sharp = load("sd7037")
pen = []
for Re in (50e3, 80e3):
    a1 = sharp.get_aero_from_neuralfoil(alpha=alphas, Re=Re, n_crit=9, model_size="xlarge")
    a2 = foils["sd7037"].get_aero_from_neuralfoil(alpha=alphas, Re=Re, n_crit=9, model_size="xlarge")
    for cl in (0.5, 0.7):
        j1, j2 = np.argmax(a1["CL"]), np.argmax(a2["CL"])
        c1 = np.interp(cl, a1["CL"][:j1 + 1], a1["CD"][:j1 + 1]); c2 = np.interp(cl, a2["CL"][:j2 + 1], a2["CD"][:j2 + 1])
        pen.append((Re, cl, c1, c2))
lines += ["\n## Blunt-TE penalty (SD7037 sharp vs 0.7 % TE), Ncrit 9\n", "| Re | Cl | sharp Cd | te07 Cd | delta |", "|---|---|---|---|---|"]
for Re, cl, c1, c2 in pen:
    lines.append(f"| {Re/1e3:.0f}k | {cl} | {c1:.4f} | {c2:.4f} | {(c2-c1)/c1*100:+.1f} % |")
with open(os.path.join(HERE, "results", "section_summary.md"), "w") as f:
    f.write("\n".join(lines) + "\n")
json.dump(summary, open(os.path.join(HERE, "results", "section_clmax.json"), "w"), indent=1)
print("\n".join(lines))

fig, ax = plt.subplots(1, 3, figsize=(14, 4.2))
for Re, col in ((40e3, "#eb6834"), (60e3, "#2a78d6"), (90e3, "#1baf7a")):
    i = int(np.where(Res == Re)[0][0])
    for key, ls in (("sd7037", "-"), ("s7075", "--")):
        CL = out[f"{key}_n9_CL"][i]; CD = out[f"{key}_n9_CD"][i]
        ax[0].plot(alphas, CL, ls, color=col, label=f"{key} Re {Re/1e3:.0f}k")
        ax[1].plot(CD, CL, ls, color=col)
        ax[2].plot(alphas, CL ** 1.5 / CD * (CL > 0), ls, color=col)
ax[0].set_xlabel("alpha [deg]"); ax[0].set_ylabel("Cl"); ax[0].legend(fontsize=7)
ax[1].set_xlabel("Cd"); ax[1].set_ylabel("Cl"); ax[1].set_xlim(0, 0.06)
ax[2].set_xlabel("alpha [deg]"); ax[2].set_ylabel("Cl^1.5/Cd")
for a in ax: a.grid(alpha=0.3)
fig.suptitle("Wing sections with 0.7 % blunt TE, NeuralFoil xlarge, Ncrit 9 (solid SD7037, dashed S7075)")
fig.tight_layout(); fig.savefig(os.path.join(HERE, "plots", "section_polars.png"), dpi=130)
