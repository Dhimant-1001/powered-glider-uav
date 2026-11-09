"""
s02_propulsion_trade.py -- right-size the propulsion set for Concept C's lower AUW.

Uses the stage-1 propulsion model (stage1_copies/propulsion_model.py, unmodified: UIUC
folding-prop data + calibrated 3-constant motor model) and adds smaller batteries / a
lighter ESC that the stage-1 catalogue did not contain:
  Tattu 3S 450 mAh 75C   42 g   (stage-1 mass_model source: readymaderc)
  Tattu 3S 650 mAh 75C   59 g   (getfpv / rotorama / readymaderc listings, 58x31x16 mm)
  Orange 2S 850 mAh 30C  50 g   (robu.in p=54646)
  Orange 2S 1000 mAh 30C 65 g   (robu.in p=26383)
  Hobbywing Skywalker 12AE 10 g, 12 A cont (stage-1 source funduinoshop)
Airframe without propulsion+battery comes from c_common.mass_model (design point, 0.75 g/cm^3).
Preliminary polar CD0 = 0.035, e = 0.80 (replaced by the AeroBuildup polar in s07).
Feasibility (stage-1 rules, tightened for a beginner):
  0.85*T/W >= 0.75 ; ESC cont >= 1.3 x static current ; static current <= 0.5 x label C x capacity
  (real C ~ half of label) ; beginner-profile endurance (80 % usable) >= 12 min ; pitch speed >= 1.8 V_cruise
"""
import os, sys, json, math
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "stage1_copies"))
import propulsion_model as pm
import c_common as cc

p = dict(cc.PARAMS)
g = cc.geometry(p)
mm = cc.mass_model(p, g)
m_prop_batt = p["m_motor"] + p["m_prop"] + p["m_esc"] + p["m_batt"]
m_airframe = mm["m"] - m_prop_batt * (1)          # contingency is not applied to battery/motor in the model
print(f"airframe w/o motor, prop, ESC, battery: {m_airframe*1000:.0f} g (S = {g['S']:.4f} m2)")

props = pm.load_props()
M, E = pm.MOTORS, dict(pm.ESCS)
E["SKY12"] = pm.ESC("Hobbywing Skywalker 12AE", 12, 10, price_inr=700, source="stage-1 source (funduinoshop)")
B = dict(pm.BATTS)
B["3S450"] = pm.Battery(3, 450, 42, 75, price_inr=1100, source="Tattu 3S 450 75C 42 g")
B["3S650"] = pm.Battery(3, 650, 59, 75, price_inr=1400, source="Tattu 3S 650 75C 59 g")
B["2S850"] = pm.Battery(2, 850, 50, 30, price_inr=700, source="Orange 2S 850 30C 50 g (robu)")
B["2S1000"] = pm.Battery(2, 1000, 65, 30, price_inr=800, source="Orange 2S 1000 30C 65 g (robu)")

combos = {}
for mk in ("CF2822_1200", "A2212_1000", "A2212_1400"):
    for pk in ("F9x4", "F9x5", "F9x6", "F10x5", "F10x6"):
        for bk in ("3S450", "3S650", "3S850", "2S850", "2S1000", "2S1300"):
            for ek in ("SKY12", "SKY20"):
                combos[f"{mk}|{pk}|{bk}|{ek}"] = pm.Combo(M[mk], props[pk], B[bk], E[ek], I_avionics=0.5)

rows = []
for key, c in combos.items():
    mass = m_airframe + (c.mass_g - 2) / 1000.0     # model prop 18 g vs our 16 g spinner+blades (+2 g)
    ac = pm.Airframe(mass=mass, S=g["S"], AR=p["AR"], CD0=0.035, e=0.80, CLmax=1.0, CL_cruise=0.55)
    try:
        r = pm.evaluate(ac, c, R_turn=9.0)
    except Exception as ex:
        continue
    TW = r["TW_static"]
    feas = dict(
        tw=0.85 * TW >= 0.75,
        esc=c.esc.I_cont >= 1.3 * r["I_static"],
        batt=r["I_static"] <= 0.5 * c.batt.C * c.batt.cap_mAh / 1000,
        endur=r["endurance_beginner_min"] >= 12.0,
        pitch=r["pitch_speed"] >= 1.8 * r["V_cruise"],
        motor=r["motor_I_ok"],
    )
    rows.append(dict(key=key, AUW_g=mass * 1000, combo_g=c.mass_g, TW=TW, I0=r["I_static"], Vp=r["pitch_speed"],
                     Vc=r["V_cruise"], I_c=r["I_cruise"], roc=r["roc_max"], end_phys=r["endurance_min"],
                     end_beg=r["endurance_beginner_min"], a_launch=r["a_launch"], ok=all(feas.values()),
                     fails=",".join(k for k, v in feas.items() if not v)))
rows.sort(key=lambda d: (not d["ok"], d["AUW_g"]))
lines = ["| combo (motor/prop/battery/ESC) | AUW g | prop+batt+ESC+motor g | T/W | I_static A | pitch speed m/s | V_c | I_cruise A | ROC max | endur. physics / beginner min | feasible |",
         "|---|---|---|---|---|---|---|---|---|---|---|"]
for d in rows[:25]:
    lines.append(f"| {d['key']} | {d['AUW_g']:.0f} | {d['combo_g']:.0f} | {d['TW']:.2f} | {d['I0']:.1f} | {d['Vp']:.1f} | {d['Vc']:.1f} | "
                 f"{d['I_c']:.2f} | {d['roc']:.1f} | {d['end_phys']:.0f} / {d['end_beg']:.0f} | {'yes' if d['ok'] else 'no: ' + d['fails']} |")
txt = "\n".join(lines)
print(txt)
open(os.path.join(HERE, "results", "propulsion_trade.md"), "w").write(
    "# Propulsion trade (s02), lightest 25 combos sorted feasible-first by AUW\n\n" + txt + "\n")
json.dump(rows, open(os.path.join(HERE, "results", "propulsion_trade.json"), "w"), indent=1)
