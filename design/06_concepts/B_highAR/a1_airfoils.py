"""
a1_airfoils.py -- section study for the high-AR wing (concept B).

Question: with a 72 mm tip chord the tip runs at Re ~35-47k.  Which of the stage-1
candidates (SD7037, SA7038, S7075) loses least there, with the printed 0.8 mm blunt TE,
and what Cl_max(Re) does the critical-section method need?

Outputs: results/airfoils.json, results/clmax_table.json, plots/a1_*.png
Run: /opt/anaconda3/bin/python3 a1_airfoils.py
"""
from common import *

plt = style()
FOILS = ["sd7037", "sa7038", "s7075"]
CHORDS = {"tip 72 mm": 0.072, "mid 108 mm": 0.108, "root 135 mm": 0.135}
RES_LIST = [30e3, 40e3, 50e3, 60e3, 70e3, 80e3, 90e3, 110e3]


def metrics(t):
    cl, cd, cm = t["cl"], t["cd"], t["cm"]
    i = int(np.argmax(cl))
    pre = slice(0, i + 1)
    ld = cl[pre] / cd[pre]
    e = np.where(cl[pre] > 0, cl[pre] ** 1.5 / cd[pre], 0)
    out = dict(clmax=float(cl[i]), a_clmax=float(t["alpha"][i]), ldmax=float(ld.max()),
               cl_ldmax=float(cl[pre][np.argmax(ld)]), emax=float(e.max()), cl_emax=float(cl[pre][np.argmax(e)]))
    for c in (0.4, 0.6, 0.8, 1.0):
        out[f"cd@{c}"] = float(cd_at_cl(t, c)[0])
    out["cm@0.6"] = float(np.interp(0.6, cl[pre][np.argmin(cl[pre]):], cm[pre][np.argmin(cl[pre]):]))
    a0 = float(np.interp(0.0, cl[pre][np.argmin(cl[pre]):], t["alpha"][pre][np.argmin(cl[pre]):]))
    out["alpha_L0"] = a0
    return out


def main():
    out = {}
    clmax_tab = {}
    for f in FOILS:
        out[f] = {}
        clmax_tab[f] = {}
        for lab, ch in CHORDS.items():
            foil = load_foil(f, ch)
            for nc in (9.0, 7.0):
                for R in RES_LIST:
                    t = section_table(foil, R, nc)
                    mtr = metrics(t)
                    out[f][f"{lab}|Ncrit{nc:.0f}|Re{R/1e3:.0f}k"] = mtr
                    if nc == 9.0:
                        clmax_tab[f].setdefault(f"{ch:.3f}", {})[f"{R:.0f}"] = mtr["clmax"]
    save_json("airfoils.json", out)
    save_json("clmax_table.json", clmax_tab)

    # ---- console summary at the three representative points
    print("Section metrics, blunt printed TE 0.8 mm, NeuralFoil xlarge->'large', Ncrit 9 (Ncrit 7 in brackets)")
    hdr = f"{'foil':8s} {'station':12s} {'Re':>5s} {'Clmax':>6s} {'Cd@.4':>7s} {'Cd@.6':>7s} {'Cd@.8':>7s} {'Cd@1.0':>7s} {'(L/D)max':>9s} {'E=Cl^1.5/Cd':>12s} {'Cm@.6':>6s}"
    print(hdr)
    rows = []
    for f in FOILS:
        for lab, R in (("tip 72 mm", 40e3), ("mid 108 mm", 60e3), ("root 135 mm", 80e3)):
            m9 = out[f][f"{lab}|Ncrit9|Re{R/1e3:.0f}k"]
            m7 = out[f][f"{lab}|Ncrit7|Re{R/1e3:.0f}k"]
            rows.append((f, lab, R, m9, m7))
            print(f"{f:8s} {lab:12s} {R/1e3:4.0f}k {m9['clmax']:6.2f} {m9['cd@0.4']:7.4f} {m9['cd@0.6']:7.4f} "
                  f"{m9['cd@0.8']:7.4f} {m9['cd@1.0']:7.4f} {m9['ldmax']:5.1f}({m7['ldmax']:4.1f}) "
                  f"{m9['emax']:6.1f}({m7['emax']:5.1f}) {m9['cm@0.6']:6.3f}")
    # tip-Re penalty: E_max(40k)/E_max(80k) and Cd@0.7 ratio
    print("\nTip-Re penalty (same blunt-TE foil): Emax(40k)/Emax(80k), Cd@0.8(40k)/Cd@0.8(80k)")
    pen = {}
    for f in FOILS:
        a = out[f]["tip 72 mm|Ncrit9|Re40k"]
        b = out[f]["root 135 mm|Ncrit9|Re80k"]
        pen[f] = dict(E_ratio=a["emax"] / b["emax"], cd08_ratio=a["cd@0.8"] / b["cd@0.8"],
                      clmax_tip=a["clmax"], clmax_root=b["clmax"])
        print(f"  {f:8s} E ratio {pen[f]['E_ratio']:.2f}   Cd@0.8 ratio {pen[f]['cd08_ratio']:.2f}   "
              f"Clmax tip(40k) {a['clmax']:.2f} vs root(80k) {b['clmax']:.2f}")
    save_json("airfoil_penalty.json", pen)

    # ---- plots: polars at 40k (tip) and 70k (MAC-ish)
    fig, axs = plt.subplots(1, 3, figsize=(13, 4.2))
    cols = {"sd7037": C1, "sa7038": C3, "s7075": C2}
    for f in FOILS:
        for R, ls, ch in ((40e3, "--", 0.072), (70e3, "-", 0.116)):
            t = section_table(load_foil(f, ch), R, 9.0)
            i = int(np.argmax(t["cl"])) + 4
            axs[0].plot(t["cd"][:i], t["cl"][:i], ls, color=cols[f], label=f"{f} Re {R/1e3:.0f}k")
            axs[1].plot(t["alpha"], t["cl"], ls, color=cols[f])
            e = np.where(t["cl"] > 0, t["cl"] ** 1.5 / t["cd"], np.nan)
            axs[2].plot(t["cl"][:i], e[:i], ls, color=cols[f])
    axs[0].set_xlim(0, 0.06); axs[0].set_xlabel("Cd"); axs[0].set_ylabel("Cl"); axs[0].legend(fontsize=7)
    axs[0].set_title("Drag polar (blunt TE 0.8 mm, Ncrit 9)", loc="left")
    axs[1].set_xlabel("alpha (deg)"); axs[1].set_ylabel("Cl"); axs[1].set_title("Lift curve", loc="left")
    axs[2].set_xlabel("Cl"); axs[2].set_ylabel("Cl^1.5/Cd"); axs[2].set_title("Endurance parameter", loc="left")
    axs[2].set_ylim(0, 60)
    fig.tight_layout(); fig.savefig(os.path.join(PLOTS, "a1_section_polars.png")); plt.close(fig)

    fig, ax = plt.subplots(figsize=(6.5, 4))
    for f in FOILS:
        for ch, ls in ((0.072, "--"), (0.135, "-")):
            ax.plot(np.array(RES_LIST) / 1e3, [clmax_tab[f][f"{ch:.3f}"][f"{R:.0f}"] for R in RES_LIST], ls,
                    color=cols[f], marker="o", ms=3, label=f"{f} c={ch*1000:.0f} mm")
    ax.set_xlabel("Re (thousands)"); ax.set_ylabel("section Cl_max (NeuralFoil, Ncrit 9)")
    ax.set_title("Cl_max vs Re (used for the critical-section CL_max)", loc="left"); ax.legend(fontsize=7)
    fig.tight_layout(); fig.savefig(os.path.join(PLOTS, "a1_clmax_vs_Re.png")); plt.close(fig)


if __name__ == "__main__":
    main()
