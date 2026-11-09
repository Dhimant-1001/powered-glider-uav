# Construction method and mass model: small hand-launched powered glider

Folder: `design/02_construction/`
Files: `construction_recommendation.md` (this file), `mass_model.py` (parametric mass model), `example_breakdowns.md` (tables produced by the model), `auw_wingloading_vs_span.png` (plot).
Re-run with `/opt/anaconda3/bin/python3 mass_model.py`.

---

## 0. Bottom line

**Recommended construction (A1):**
- **Wing:** printed in **Bambu PLA Aero** (foaming PLA) as single-wall "vase-mode" segments with internal ribs, about 240 mm long each.
- **Spar:** the segments slide onto a **stepped, telescoping pultruded carbon spar** (for example a 6x4 mm centre tube with 4x2 mm tubes outboard) plus a 2 mm carbon rear rod.
- **Fuselage:** printed PLA Aero pod (2 walls) on an **8x6 mm carbon tail boom**.
- **Tails:** **3 mm Depron/XPS flat-plate tail surfaces** (all-printed tails as a fallback).

Why this option:
- Every material can be bought online in India.
- The Bambu P1S does most of the work.
- The wing comes out as the exact XFLR5 airfoil, lofted from the same `.dat` file. This matters for the "XFLR5 vs flight" deliverable.
- It is the simplest route for someone with no aircraft experience.

**Weight is the critical issue.** Turn radius scales directly with wing loading (R ∝ W/S). To make a 180° turn inside a 20 m wide field (radius about 8 m or less at 45° bank, with a 30 % stall margin), wing loading needs to be **about 22 g/dm² or less**. With full UAV avionics (F405 wing flight controller, GPS, 4 servos, folding prop, 3S 450 mAh pack):

| Construction | Span needed for WL ≤ 22 g/dm² at AR 10 / 11 / 12 | AUW there |
|---|---|---|
| A1 PLA Aero printed (recommended, tuned to ~0.65 g/cm³) | 1.61 / 1.70 / 1.79 m | 570–585 g |
| A3 LW-PLA (colorFabb/Polymaker, imported) | 1.42 / 1.51 / 1.60 m | 440–470 g |
| C printed D-box + Depron aft skins | 1.50 / 1.59 / 1.67 m | 495–510 g |

- **Implication for the configuration study:** with printed construction, the design should be about **1.5–1.7 m span, AR 10–11**, rather than 1.0–1.2 m. A 1.0–1.2 m printed aircraft carrying a flight controller and GPS ends up at 30–40 g/dm² and needs a 12–15 m turn radius, which does not fit a 20 m wide field.
- **Span is not a problem for the field.** It only sets the size the student hand-launches, and 1.6 m is normal for a hand-launched glider.
- **Upgrade path, in order of payoff:**
  1. Tune PLA Aero toward 0.55 g/cm³.
  2. Use LW-PLA filament.
  3. Use the "light avionics" set (F411-WTE, no GPS, 4 g servos, 2S): about −50 g.
  4. Hybrid D-box wing (option C).

---

## 1. Options compared

Each option was run through the mass model at b = 1.4 m, AR 11 (S = 17.8 dm²), with identical avionics. The full table is in `example_breakdowns.md`.

| Rank | Option | Wing shell (g/dm²) | Wing incl. spars (g/dm²) | AUW (g) | WL (g/dm²) | Sourcing in India | Beginner difficulty | Airfoil fidelity (XFLR5 vs flight) | Verdict |
|---|---|---|---|---|---|---|---|---|---|
| **1** | **A1: PLA Aero printed wing + pod, carbon spar/boom, 3 mm Depron tails** | 8.1 | 10.4 | 472 | 26.5 | Easy: PLA Aero ₹4.0–7.0k/kg; carbon tubes Robu/IRS | **Low**: print, slide on spar, CA glue | **Best**: exact lofted section; ridges run streamwise | **Recommended** |
| 2 | A2: same, tails also printed (vase, about 7 % thick) | 8.1 | 10.4 | 487 | 27.3 | Easiest (one material) | Lowest | Best | Fallback if Depron is unavailable (+15 g at the tail, needs more nose weight) |
| 3 | A3: as A1 in colorFabb/Polymaker **LW-PLA** | 5.9 | 8.3 | 423 | 23.8 | Imported: Polymaker LW-PLA 500 g ≈ ₹8,190 (Tanotis) | Low (more tuning, stringing) | Best | **Weight upgrade** if budget allows |
| 4 | C: printed PLA Aero D-box (0–30 % chord) + 3 mm Depron aft skins over printed ribs (Planeprint-style) | 6.6 | 8.9 | 444 | 24.9 | Depron only from Vortex-RC / IndiaMART, quality varies | Medium–high (bending foam, foam-safe glue, alignment) | Medium: facets/sag between ribs aft of the D-box | Lightness fallback |
| 5 | B1: hot-wire-cut XPS core + packing-tape/laminate covering, printed pod | 6.5 (at 127 mm chord) | 8.5 | 436 | 24.5 | XPS insulation board ₹375–728/m² (IndiaMART); EPS "thermocol" everywhere | Medium–high: needs a DIY nichrome bow + printed templates; thin trailing edges are hard to cut | Good if cut well, variable | Viable for a second airframe |
| 6 | B2: folded 5 mm paper-faced foamboard (FliteBoard/DTFB-style) | about 8.3 (assumed) | 10.3 | 471 | 26.4 | Vortex-RC FliteBoard | Easy to build | **Poor**: folded/"FT" airfoil does not match XFLR5 | Not recommended |
| A0 | A1 with **untuned** PLA Aero (~0.85 g/cm³) | 10.5 | 13.1 | 553 | 31.0 | – | – | – | Shows why a density test coupon is mandatory |
| X | Indian 5 mm **PVC Sunboard** | about 52 | 55 | 1337 | 75 | Everywhere | Easy | Poor | **Rejected**: PVC foam board is 0.35–0.55 g/cm³, about 10× Depron |

Notes:
- **Why A1 over the lighter foam options:** A1 is only 25–35 g heavier than C or B1 at the same span (5–7 %). In return it gives:
  - a reproducible airfoil, which the graded XFLR5-vs-flight comparison depends on;
  - no Depron supply risk (RCIndia reports that model-grade Depron production stopped and sheets now arrive bent);
  - spares printed overnight;
  - about 15 fewer hours of skilled manual work.
- **What actually drives wing loading:** span and area (bigger wing), avionics choice, and filament density matter more than wing construction method.
- **Sunboard:** the "Sunboard" sold in Indian stationery and sign shops is PVC foam, not polystyrene. RCIndia members call it "way too heavy to be any use for aeromodelling". Do not use it for any flying surface.

---

## 2. Recommended build method (A1) in detail

### 2.1 Wing
1. **Geometry and segmentation**
   - Loft the XFLR5 wing (same `.dat` airfoil, chord, twist and dihedral breakpoints) in CAD (Fusion 360 / Onshape).
   - Split it into segments of **≤ 240 mm** (P1S Z-height is 256 mm). Print each segment standing on its root rib, with the span along Z.
   - Segment counts: 1.4 m span → 6 segments; 1.6 m → 8.
   - Each segment of a 145 mm-chord wing takes about 1.2–1.8 h on the P1S in vase mode. A 1.6 m wing is about 12–15 h of printing in total.
2. **Vase mode with internal structure**
   - Use spiral/vase mode with a single 0.42–0.50 mm wall.
   - Create internal ribs and diagonal webs with the standard "0.01 mm slot" trick: thin cuts in the solid that the slicer turns into double walls.
   - Include a spar tunnel at the maximum-thickness location (25–30 % chord) and a 2 mm rear-rod tunnel at about 65 % chord.
   - Reference designs that use this method: 3DLabPrint, Planeprint, Eclipson.
3. **Spar**
   - Stepped telescoping pultruded carbon tubes with 1 mm walls. The OD of the outer tube equals the ID of the inner tube (8x6 ⊃ 6x4 ⊃ 4x2 ⊃ 2 mm rod), so no adapters are needed.
   - The centre tube is one piece (≤ 1 m stock length) and doubles as the wing joiner.
   - Sizing in the model: ultimate load factor 6 (4 g limit × 1.5), elliptic lift, allowable stress 400 MPa (pultruded carbon is about 1,000+ MPa in flexure, so this is conservative), spar outer diameter ≤ 80 % of the local airfoil thickness.
   - Results: 1.2 m → 5x3 tube; 1.4–1.6 m → 6x4 centre tube with 4x2 outboard of about 35 % semi-span.
4. **Assembly**
   - Slide the segments onto the spar and rod, and glue each joint with thin CA plus kicker, or 5-min epoxy at the root.
   - Ailerons are printed as part of the segment and then cut free along the hinge line. Hinge them with clear tape (top hinge, bevelled bottom).
   - Mount the servos in printed bays on 5 g-class servos, with 1 mm wire or carbon pushrods.
5. **Minimum printable thickness**
   - The wall must not merge with the opposite skin, so the local thickness needs to be at least 2 line widths plus clearance (≥ ~1.2 mm) everywhere except the trailing edge.
   - The spar needs **t ≥ OD + 2 walls + 0.4 mm**: 6x4 needs ≥ 7.3 mm (≥ 81 mm chord at 9 % t/c); 4x2 needs ≥ 5.3 mm.
   - **Recommend airfoil t/c of 8–10 %.** Sections thinner than about 7 % at chords under 100 mm leave no room for the spar and internal ribs.
6. **Trailing edge**
   - In vase mode the practical trailing-edge thickness is about 0.8–1.0 mm, since the two walls merge. That is about 0.6–0.7 % of chord at 145 mm.
   - Model it in XFLR5 with the "Refine / TE gap" function (TE gap ≈ 0.6 % c). XFOIL handles blunt trailing edges, and the predicted small drag penalty then shows up in the polar instead of becoming an unexplained flight discrepancy.
7. **Surface fidelity**
   - Printing span-wise along Z makes every layer an airfoil section, so the layer ridges run **chordwise/streamwise**, which is the least harmful direction.
   - Vase mode has no Z-seam.
   - Foaming makes walls grow about 0.1–0.2 mm, which adds less than 0.3 % c to t/c at 145 mm chord. This is negligible.
   - The matte foamed surface is slightly rough. In XFLR5, run polars at Ncrit 9 (default) and Ncrit 5–7 (rougher/turbulent field air) and treat the spread as the uncertainty band in the flight comparison.
8. **Material handling**
   - Dry PLA Aero before printing (55 °C for 8 h, per the Bambu TDS) and store it at under 20 % RH.
   - **Print a density coupon first.** Print a 100 mm vase cylinder, weigh it and compute g/cm³. Tune nozzle temperature (230–260 °C) and flow (~0.5–0.6) until density is ≤ 0.65 g/cm³ (Bambu range: 0.55–1.05 g/cm³).
   - PLA Tg is about 55 °C. **Never leave the airframe in a parked car in Hyderabad**, and prefer white or light-grey filament.

### 2.2 Fuselage and tail
- **Pod:** printed in PLA Aero with 2 walls (~0.8 mm), 0–5 % sparse infill, in 2–3 pieces, about 40 mm × 50 mm in section to fit a 44 × 29 mm flight controller and a 3S 450–850 mAh pack. Additional parts:
  - motor mount in PLA Basic/PETG (it carries thrust and heat);
  - finger-grip area under the wing for hand launch;
  - magnet hatch;
  - packing-tape skid on the belly for grass landings.
- **Tail boom:** 8x6 mm pultruded carbon tube (6x4 if shorter than 0.5 m), bonded into a printed socket. The H and V tails attach with a printed clamp.
- **Tails:** 3 mm Depron (or 3 mm XPS/"modelling foam") flat plates, stiffened with a 1–2 mm carbon strip, tape-hinged. Areal density is about 1.8 g/dm² versus about 6.8 g/dm² for printed tails. Saving tail weight also saves nose ballast, because the tail sits about 0.6 m behind the CG.

### 2.3 Expected build effort (A1, 1.5–1.6 m)
| Task | Hours |
|---|---|
| CAD: loft wing from XFLR5, segment, ribs, pod (beginner, with tutorials) | 15–25 |
| Density coupons and test segment | 3 (plus about 4 h printing) |
| Printing (wing ~14 h, pod ~7 h, mounts/horns ~3 h), unattended | about 24 printer-hours |
| Assembly: spar, gluing, ailerons, tail, linkages | 8–12 |
| Electronics install, wiring, ArduPilot/INAV setup, CG and throws | 8–12 |
| **Total hands-on** | **about 35–50 h** (+ about 25 printer-hours) |

### 2.4 Risks and mitigations
| Risk | Effect | Mitigation |
|---|---|---|
| PLA Aero prints denser than assumed (0.85 vs 0.65 g/cm³) | +80 g at 1.4 m (WL 26.5 → 31 g/dm²) | Density coupon, tuned profile, or switch to LW-PLA |
| Thin single-wall skin dented by handling or hard landings | Local airfoil damage | Internal ribs ≤ 40 mm apart; print spare segments; tape the leading edge |
| Segment misalignment or twist | Roll trim, asymmetric stall | Two-rod (spar + rear rod) alignment; glue on a flat jig |
| Heat softening (Tg ~55 °C) | Warped wing | Shade; no closed car |
| Weight creep (glue, wires, ballast) | WL rises | 4 % glue and 5 % contingency already in the model; weigh every part and keep a log |
| Carbon tube stock-outs (Robu often out of stock) | Schedule | Alternatives: IndianRoboStore, Ibots, TNT E-Comp, UAV Garage |

---

## 3. Mass model (`mass_model.py`)

```python
from mass_model import mass_breakdown, performance_hint
m = mass_breakdown(span_m, wing_area_m2, htail_area_m2, vtail_area_m2,
                   fuselage_length_m, battery_mass_kg, motor_mass_kg,
                   construction='recommended',      # or any key of CONSTRUCTIONS
                   airfoil_tc=0.09, taper_ratio=0.6, n_servos=4,
                   servo='servo_emax_es9052md', fc='fc_matek_f405_wte',
                   rx='rx_none_integrated', gps='gps_matek_m10q_5883',
                   airspeed='airspeed_none', esc='esc_generic_12_20a',
                   n_ult=6.0, sigma_allow_pa=400e6, contingency_frac=0.05)
m['total']   # kg
m['info']    # wing loading, spar chosen, segments, areal densities, pod/boom lengths
```

`construction` keys: `recommended` (= `A1_printed_aero_foam_tail`), `A2_printed_aero_all`, `A3_printed_lwpla_foam_tail`, `A3_printed_lwpla_all`, `A0_printed_aero_untuned`, `C_dbox_hybrid`, `B1_hotwire_xps`, `B2_foamboard_folded`, `X_sunboard_5mm`.

### 3.1 Areal densities used (planform basis)
| Item | Value | Basis |
|---|---|---|
| Printed wing shell, PLA Aero (A1) | **8.1 g/dm²** = 2.04 (wetted/planform) × 0.45 mm wall × 0.65 g/cm³ × 1.35 (internal ribs) | Bambu PLA Aero foamed density 0.55–1.05 g/cm³ (tuned value 0.65 is an ASSUMPTION within that range) |
| Printed wing shell, LW-PLA (A3) | **5.9 g/dm²** (ρ = 0.48 g/cm³) | colorFabb LW-PLA 0.40–0.48 g/cm³ at max foaming; CNC Kitchen measured 44 % of PLA density at 250 °C |
| Printed wing shell, PLA Aero untuned | 10.5 g/dm² (ρ = 0.85) | Mid-range of Bambu data |
| Wing total incl. spar, rear rod, joints, hardware (A1) | **10.2–11.3 g/dm²** (1.0–1.6 m) | Model output |
| Hybrid D-box + 3 mm Depron skins (C) | 6.6 g/dm² shell | Depron 3 mm ≈ 40 kg/m³ → 1.2 g/dm² per skin (Depron datasheet) |
| Hot-wire XPS core + tape (B1) | 0.68·t/c·c·32 kg/m³ + 1.0 tape + 0.3 → 6.5 g/dm² at 127 mm chord | XPS 28–32 kg/m³ (IndiaMART listings); tape 50 g/m² ASSUMED |
| Depron 3 mm flat tails | **1.8 g/dm²** (1.2 foam + 0.6 carbon strip/tape) | Depron datasheet; extras ASSUMED |
| Printed tails, PLA Aero | 6.8 g/dm² | same wall model |
| Printed pod wall | 0.8 mm × 0.75 g/cm³ = 6.0 g/dm² of wetted area + 19 g fixed (motor mount, saddle, bolts, magnets, bulkheads, boom socket) | ASSUMED |
| Carbon tubes (ρ 1.55 g/cm³) | 4x2: 14.6 g/m; 5x3: 19.5; 6x4: 24.3; 8x6: 34.1; 10x8: 43.8; 2 mm rod: 4.9; 3 mm rod: 11.0 | Geometry; density ASSUMED (typical pultruded) |
| Glue | 4 % of structure | ASSUMED |
| Contingency | 5 % of all non-battery, non-motor mass | ASSUMED |

**Calibration:** the published LW-PLA aircraft below were re-created in the model using their simple equipment (2 servos, receiver only, no flight controller or GPS, fixed prop; this reference equipment is assumed). The model is slightly **conservative**:

| Reference | Published TOW | Model |
|---|---|---|
| 3DLabPrint Bungee Prop (1.18 m, 21.2 dm², 216 g LW-PLA print) | 320 g | 342 g (+7 %) |
| Planeprint Rise nano (1.02 m, ~12.6 dm², 19 g/dm²) | 240 g | 251 g (+5 %) |

Other published LW-PLA whole-airframe data points (printed mass / wing area):

| Aircraft | Span | Printed mass / wing area | Takeoff weight / wing loading |
|---|---|---|---|
| Eclipson Model A | 1.0 m | 220 g / 16 dm² = 13.8 g/dm² | 490 g TOW (730 g in plain PLA) |
| Eclipson Model V | 1.37 m | 170 g / 16 dm² | 380 g RTF |
| Eclipson Model S | 1.8 m | 430 g / 30 dm² | 750 g TOW |
| Planeprint Rise | 2.0 m | 560 g of material | ≥ 650 g, 19 g/dm² |
| Planeprint Falcon | 1.4 m | 340 g of material | 520 g, 20 g/dm² |
| Planeprint Swift S1 (hybrid printed + foam, towed glider) | 1.4 m | 120 g printed | 190 g, 10 g/dm² |
| MakerWorld glider printed in Bambu PLA Aero | 1 m | – | 15–18 g/dm² |

### 3.2 Fixed avionics and propulsion masses (sourced)
| Component | Mass | Source |
|---|---|---|
| **Matek F405-WTE** (default; integrated ELRS 2.4 GHz receiver, 2 BECs, current sensor; ArduPilot + INAV) | 22 g incl. USB/buzzer adapter | mateksys.com, team-blacksheep |
| Matek F411-WTE (INAV only, no receiver) | 10 g | mateksys.com |
| Matek F405-miniTE (20×20, no wing PDB) | 5 g | team-blacksheep, defiancerc |
| SpeedyBee F405 Wing Mini (FC + PDB stack) | 23 g (FC alone 5.6 g) | oscarliang.com |
| RadioMaster RP1 ELRS receiver | 2.2 g incl. antenna | radiomasterrc.com |
| BetaFPV ELRS Lite receiver | 0.46–0.53 g | retailers |
| EMAX ES9051 servo (plastic gear, 0.8 kg·cm) | 4.1 g | retailers |
| **EMAX ES9052MD** (default; metal-gear digital) | 5.5 g | getfpv / racedayquads |
| TowerPro SG90 (1.8 kg·cm; many counterfeits) | 9 g | datasheet; TowerPro counterfeit warning |
| EMAX ES08MA II | 12 g | retailers |
| Corona DS-929MG | 12.5 g | servodatabase, hobbyking |
| Matek M10Q-5883 GPS + compass (default) | 7.5–8 g | retailers |
| Beitian BN-220 GPS | 5.3 g | retailers |
| ESC: Hobbywing Skywalker 12AE / 20A | 10 g / 19 g (default generic 12–20 A without BEC: 12 g, ASSUMED) | retailers |
| Motors: SunnySky X2204 / EMAX MT2204 / EMAX RS2205 / SunnySky X2208 | 21 / 25 / 30 / 44 g | getfpv, oscarliang |
| LiPo: Tattu 2S 450 / Tattu 3S 450 / Orange 2S 1000 (20C) / Orange 3S 1000 | 29 / 42 / 51 / 85 g | retailers, robu.in |
| 30 mm folding spinner (Hyperflight MFZ30) + blades | 12 g + about 4 g (blades ASSUMED) | hyperflight.co.uk |
| Wiring, XT30, servo extensions, strap | 8 g + 5 g/m of span (ASSUMED) | – |
| Horns, pushrods, clevises | 1.5 g per servo (ASSUMED) | – |
| Airspeed sensor (optional, recommended for the XFLR5-vs-flight comparison) | about 6 g (ASSUMED) | – |

Servo sizing check: hinge moment at 12 m/s on a 1.5 m wing aileron (about 0.014 m², 35 mm chord) is about 0.2 kg·cm, so a 5 g servo (0.8–1.5 kg·cm) has margin of 4× or more. SG90 clones are not recommended (poor centring and deadband, counterfeits).

### 3.3 Example breakdowns (A1, AR 11; full tables for AR 10/11/12 in `example_breakdowns.md`)
Defaults: fuselage length 0.70 b, tail arm 0.62 L, V_h 0.45, V_v 0.028, 30 g motor, 3S 450 mAh (42 g), F405-WTE, M10Q GPS, 4 × ES9052MD.

| (g) | b = 1.0 m | 1.2 m | 1.4 m | 1.6 m |
|---|---|---|---|---|
| Wing area (dm²) / mean chord (mm) | 9.1 / 91 | 13.1 / 109 | 17.8 / 127 | 23.3 / 145 |
| Main spar (telescoping) | 4x2 + 2 mm rod | 5x3 + 3 mm rod | 5x3 + 3 mm rod | 6x4 + 4x2 |
| Wing (shell + spars + joints + hardware) | 102 | 142 | 185 | 241 |
| Pod + boom + tail mounts | 59 | 70 | 77 | 84 |
| H + V tail (Depron) | 5 | 6 | 7 | 9 |
| Avionics + propulsion (excl. battery) | 129 | 130 | 131 | 132 |
| Battery (3S 450) | 42 | 42 | 42 | 42 |
| Glue + contingency | 20 | 25 | 30 | 36 |
| **AUW** | **357** | **415** | **472** | **543** |
| Wing loading (g/dm²) | 39.2 | 31.7 | 26.5 | 23.3 |
| Vs at CLmax 1.2 (m/s) | 7.7 | 6.9 | 6.3 | 5.9 |
| 45° turn radius at 1.3 Vs,banked (m) | 14.3 | 11.6 | 9.7 | 8.5 |

AR 10, same spans: AUW 365 / 427 / 488 / 565 g, WL 36.5 / 29.7 / 24.9 / 22.1 g/dm², R45 13.3 / 10.8 / 9.1 / 8.1 m.

The "light avionics" variant at 1.6 m AR 10 is about 516 g and 20.2 g/dm². It uses F411-WTE + RP1, no GPS, 4 × ES9051, 2S 450 and an X2204.

Sensitivities at 1.4 m: 3S 1000 mAh instead of 450 adds 48 g. A 2208 motor instead of a 2205 adds 14 g. 4 × SG90 adds 14 g. The light avionics set saves about 49 g.

---

## 4. Indian sourcing and indicative prices (INR, incl. GST where stated; check before buying)
| Item | Price / where |
|---|---|
| Bambu PLA Aero 1 kg | ₹3,999 (LayerX); ₹4,099–6,999 (3idea.in); ₹5,253 (Moglix). One spool is enough (wing + pod + test prints ≈ 350–450 g) |
| Polymaker PolyLite LW-PLA 500 g | about ₹8,190 incl. import duty (Tanotis), roughly 4× the per-kg cost of PLA Aero |
| Bambu PLA Basic (motor mount, servo trays) | ₹1,199–1,299 (KSP) |
| Carbon tube 8x6x1000 pultruded | ₹506 / pack of 2 (Robu, often out of stock); ₹835 (IndianRoboStore) |
| Carbon tube 6x4x1000 pultruded | ₹795–999 (IndianRoboStore); ₹782 / pack of 2 (Ibots); ₹780 (TNT E-Comp) |
| Carbon tube 4x2x1000 | pack of 4 at The Engineer Store (price not captured) |
| Carbon rod 2 mm × 1000 | ₹279 / pack of 2 (Robu); 3 mm ₹469 / pack of 2 |
| Depron 3 mm | ₹1,828 / 20 sheets 30"×20" (Vortex-RC); ₹525 / 100×30 cm sheet (IndiaMART, Jodhpur) |
| XPS insulation board 25 mm (for B1 or tail sheets after slicing) | ₹375–728 / m² (IndiaMART) |
| Indian PVC Sunboard | cheap and everywhere, but **do not use** (0.35–0.55 g/cm³) |

---

## 5. Sources
- Planeprint Rise / Rise nano / Falcon / Swift S1 specs: https://www.skyraccoon.com/aircraft/PLANEPRINT_Rise_rise , https://www.skyraccoon.com/aircraft/PLANEPRINT_Rise-nano_Risenano , https://www.skyraccoon.com/aircraft/PLANEPRINT_Falcon_falcon , https://www.skyraccoon.com/aircraft/PLANEPRINT_Swift-S1_swifts1
- Eclipson Model A / Model S / Apex: https://www.skyraccoon.com/aircraft/Eclipson-Airplanes_Model-A_ModelA , https://www.air-rc.com/aircraft/Eclipson-Airplanes_Model-S_models , https://www.skyraccoon.com/aircraft/Eclipson-Airplanes_Apex_apex ; Model V: https://learn.colorfabb.com/tag/eclipson
- 3DLabPrint Bungee Prop (1180 mm, 21.2 dm², 216 g print, 320 g TOW, vase mode LW-PLA): https://www.skyraccoon.com/aircraft/3D-LabPrint_Bungee-Prop_bungee ; Swift: https://3dlabprint.com/shop/swift-s-1/
- MakerWorld PLA-Aero glider (1 m, 15–18 g/dm²): https://www.makerworld.com/en/models/156104 ; LW-PLA 1.16 m glider 330 g: https://makerworld.com/models/3187790
- Bambu PLA Aero TDS (filament 1.21 g/cm³, foaming, drying 55 °C 8 h): https://cdn.shopify.com/s/files/1/0574/3116/2995/files/Bambu_PLA_Aero_Technical_Data_Sheet.pdf ; printed density 0.55–1.05 g/cm³, "about half the weight": https://core-electronics.com.au/bambu-lab-pla-aero-gray-filament-with-spool-1kg.html , https://stemfinity.com/collections/bambu-lab/products/bambu-lab-pla-aero
- colorFabb LW-PLA tests (density vs temperature, stiffness/strength): https://www.cnckitchen.com/blog/colorfabb-lw-pla-testing-foaming-pla ; LW-PLA-HT TDS: https://colorfabb.com/media/datasheets/tds/colorfabb/TDS_E%20ColorFabb%20LW-PLA-HT.pdf
- Depron density (3 mm ≈ 40, 6 mm ≈ 33 kg/m³): https://efobasen.efo.no/mime_root/279/DEPRON%20-%2009-2020.pdf ; India Depron: https://dcmpx.remotevs.com/com/vortex-rc/www/PL/product/3mm-depron-30x20-20-sheet-pack , https://m.indiamart.com/city/jodhpur/packaging-foam.html
- Sunboard too heavy / FliteBoard: https://www.rcindia.org/tools-materials-and-building-techniques/introducing-fliteboard-lightweight-paper-laminated-foamboards-for-rc-planes-in/ ; Depron supply note: https://www.rcindia.org/rc-outlets-and-resellers/bigginer-series-all-flite-test-kits-and-rc-powers-kits-are-available-in-pune ; PVC foam density: https://m.indiamart.com/impcat/sun-boards.html
- XPS India prices: https://m.indiamart.com/impcat/xps-sheet.html
- PLA Aero India prices: https://layerx.cc/products/bambu-pla-aero-gray , https://www.3idea.in/product-detail/bambu-lab-pla-aero-3d-printing-filament-grey-reusable-spool-1kg , https://www.moglix.com/bambu-lab-1kg-175mm-aero-pla-grey-3d-printer-filament/mp/msn8klp017pn5v ; Polymaker LW-PLA India: https://www.tanotis.com/products/polymaker-1-75mm-polylite-lw-pla-filament-500g-gray
- Carbon tubes India: https://stgaws.robu.in/?p=653504 , https://indianrobostore.com/product/8-x-6-x-1000-mm-carbon-fiber-tube-thickness-1mm , https://indianrobostore.com/product/6-x-4-x-1000-mm-carbon-fiber-tube-thickness-1mm , https://ibots.in/?p=34790 , https://www.tntecomp.com/products/pultruded-carbon-fiber-tube-hollow-6mm-x-4mm-x-1000mm , https://robu.in/product-category/carbon-fiber/?view=grid
- Matek F405-WTE: https://www.mateksys.com/?p=10142 , https://team-blacksheep.com/products/prod:matek_fc_f405_wte ; F411-WTE: https://www.mateksys.com/?p=10073 ; F405-miniTE: https://team-blacksheep.com/products/prod:matek_fc_f4_minite ; SpeedyBee F405 Wing Mini: https://oscarliang.com/speedybee-f405-wing-mini-fc/
- RadioMaster RP1: https://www.radiomasterrc.com/products/rp1-expresslrs-2-4ghz-nano-receiver ; BetaFPV ELRS Lite: https://fpv24.com/en/betafpv/betafpv-elrs-lite-receiver-flat-antenna
- Servos: ES9051 https://www.unmannedtechshop.co.uk/products/emax-es9051-digital-servo ; ES9052MD https://www.getfpv.com/planes/emax-metal-gear-digital-servo-es9052md.html ; ES08MA II https://racedayquads.com/products/emax-es08ma-ii-12g-analog-metal-gear-servo ; Corona DS-929MG https://servodatabase.com/servos/corona , https://hobbyking.com/corona-929mg-metal-gear-servo-2-2kg-0-10sec-12-5g.html ; SG90 https://docs.circuit.rocks/motors/sg90-servo/
- GPS: Matek M10Q-5883 https://www.rotorama.com/ridici-jednotky/matek-m10-5883-gps-s-kompasem ; BN-220 https://rcdrone.top/products/5-3g-beitian-bn-220
- ESC: Skywalker 20A https://wrekd.com/products/skywalker-v1-20a-esc ; 12AE https://funduinoshop.com/en/electronic-modules/electric-motors-and-accessories/speed-controller/hobbywing-skywalker-12ae-brushless-esc
- Motors: SunnySky X2204 https://www.getfpv.com/sunnysky-x2204-16-2300kv.html ; X2208 https://www.getfpv.com/sunnysky-x2208-15-1260kv-ii.html ; EMAX MT2204 https://www.getfpv.com/discontinued/emax-mt2204-2300kv-brushless-motor-ccw.html ; RS2205 https://oscarliang.com/emax-rs2205-2300kv-2600kv-motors/
- LiPo: Tattu 2S 450 https://www.rotorama.com/product/tattu-450mah-2s-75c ; Tattu 3S 450 https://www.readymaderc.com/products/details/tattu-3s-75c-450mah-lipo-battery-xt30 ; Orange 2S 1000 https://st.robu.in/?p=43021 ; Orange 3S 1000 https://stgaws.robu.in/?p=23749
- Folding spinner 30 mm (12 g): https://www.hyperflight.co.uk/products.asp?code=MFZ30
