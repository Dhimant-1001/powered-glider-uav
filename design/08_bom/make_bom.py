"""Generate Powered_Glider_BOM.xlsx (bill of materials) with openpyxl."""
import os
from openpyxl import Workbook
from openpyxl.styles import Font, PatternFill, Alignment, Border, Side

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Powered_Glider_BOM.xlsx")
G, Y, R = "High Priority / Need Now", "Spare / Get Later", "Available in Lab (Do not order)"
FILL = {G: "C6EFCE", Y: "FFEB9C", R: "FFC7CE"}
NF = "Not found – search Robu.in"

# (name, qty, unit price INR or None, use case, link, priority)
rows = [
    ("A2212/13T 1000KV brushless outrunner motor", 1, 349, "Propulsion",
     "https://robu.in/product/a2212-10t-13t-1000kv-brushless-motor-with-soldered-connector/", G),
    ("Hobbywing Skywalker 20A ESC (with BEC)", 1, 1143, "Motor speed control; 2A BEC powers RX/servos",
     "https://robu.in/product/hobbywing-skywaller-20a-ubec/", G),
    ("SpeedyBee F405 Wing Mini flight controller (ArduPilot)", 1, 3773,
     "Flight control / stabilisation (Robu listing showed out of stock – check; fallback Matek F405-WMN)",
     "https://robu.in/product/speedybee-f405-wing-mini-fixed-wing-flight-controller/", G),
    ("FlySky FS-A8S receiver (iBUS)", 1, 1141,
     "RC link to student's FlySky transmitter (Robu listing showed out of stock – check)",
     "https://robu.in/product/flysky-fs-a8s-2-4g-8ch-receiver/", G),
    ("Matek M10Q-5883 GPS + compass", 1, 2100,
     "Navigation / logging. SUBSTITUTE: Foxeer M10Q-120 GPS + QMC5883 compass (Matek not on Robu)",
     "https://robu.in/product/foxeer-m10q-120-gps-5883-compass/", G),
    ("Airspeed sensor with pitot tube (MS4525DO)", 1, 4691,
     "Airspeed for flight-vs-analysis comparison. MS4525DO + pitot set (I2C)",
     "https://robu.in/product/air-speed-sensor-and-pitot-tube-set-pixhawk/", G),
    ("EMAX ES9051 5 g digital micro servo", 4, 599,
     "2 ailerons, elevator, rudder (not stocked on Robu – Kitsguru)",
     "https://kitsguru.com/products/emax-es9051-4-3g-digital-mini-servo-for-rc-model", G),
    ("3S 11.1V 650 mAh LiPo, 45C or higher", 2, 1465,
     "Flight battery. SUBSTITUTE: Gens Ace 800 mAh 3S 45C (no 650 mAh 45C found; Robu only has 600 mAh 25C, ₹1069)",
     "https://rcmumbai.com/products/gens-ace-800mah-3s-45c-lipo-battery-jst-syp", G),
    ("9×5 folding propeller blades + 30–32 mm spinner (3.17 mm collet)", 1, None,
     "Folding prop for belly landing", NF + " / Zbotic / RC Mumbai", G),
    ("Carbon fibre tube 8 mm OD × 6 mm ID, 1 m", 1, 1129, "Tail boom (731 mm used)",
     "https://robu.in/product/3k-roll-wrapped-carbon-fiber-tube-hollow8mmod6mmid1000mml/", G),
    ("Carbon fibre tube 6 mm OD × 4 mm ID, 1 m", 1, 1079, "Centre wing spar (480 mm)",
     "https://robu.in/product/3k-roll-wrapped-carbon-fibre-tube-hollow-6mmod-4mmid-1000mml/", G),
    ("Carbon fibre tube 5 mm OD × 3 mm ID, 1 m", 1, 699,
     "Outer wing spars (2 × 440 mm); listing is a pack of 2",
     "https://robu.in/product/pultruded-carbon-fibre-tube-hollow-5mmod-3mm-id-1000mm-l/", G),
    ("Carbon fibre rod, 2 mm solid, 1 m (pack of 2)", 1, 339,
     "Rear wing spar / alignment (1.4 m needed)",
     "https://robu.in/product/pultruded-carbon-fibre-rod-solid-2mm-1000mm/", G),
    ("Music wire / spring-steel rod, 3 mm", 1, None,
     "Polyhedral wing joiners (2 × 100 mm)", NF + " / local hardware (ASTM A228)", G),
    ("Brass tube 4 mm OD (3 mm ID)", 1, None,
     "Joiner sleeves. Closest: Metallixity 4 mm OD × 0.5 mm wall × 300 mm ; price not shown",
     "https://www.cart2india.com/tubes/metallixity-brass-tube-4mm-od-x-05mm-wall-t-x-300mm-l-straight-tubing-for-home-furnishing-machinery-diy-handicraft/00000000004391457396", G),
    ("Bambu Lab PLA Aero (lightweight foaming PLA) 1 kg", 1, 3999,
     "Printed wing, pod and tail parts (not on Robu – LayerX)",
     "https://layerx.cc/products/bambu-pla-aero-gray", G),
    ("Depron/XPS foam sheet 3 mm", 1, 430,
     "Horizontal stabiliser and fin (20 sheets 500×200 mm; not on Robu – Vortex-RC)",
     "https://www.vortex-rc.com/product/3mm-depron-20-sheet-pack-500x200mm/", G),
    ("Control horn + clevis + pushrod linkage set", 4, 55,
     "Control linkages (listing is 1 horn + clevis; 4 needed). Pushrods: Robu Z-type rods",
     "https://www.indianhobbycenter.com/products/control-horn-u-clevis-set-for-rc-plane-nylon-medium-size", G),
    ("A2212/13T 1000KV motor (spare)", 1, 349, "Spare propulsion motor",
     "https://robu.in/product/a2212-10t-13t-1000kv-brushless-motor-with-soldered-connector/", Y),
    ("5 g servo (spare) – EMAX ES9051", 2, 599, "Spare control-surface servos",
     "https://kitsguru.com/products/emax-es9051-4-3g-digital-mini-servo-for-rc-model", Y),
    ("9×5 folding prop blades (spare pair)", 1, None, "Spare after belly landings",
     NF + " / Zbotic / RC Mumbai", Y),
    ("3S 650 mAh LiPo (spare)", 1, 1465, "Spare flight battery (Gens Ace 800 mAh 3S 45C substitute)",
     "https://rcmumbai.com/products/gens-ace-800mah-3s-45c-lipo-battery-jst-syp", Y),
    ("FlySky FS-i6X transmitter", 1, 7779, "Student already has it (bundle with FS-iA10B)",
     "https://robu.in/product/flysky-fs-i6x-2-4ghz-10ch-afhds-2a-rc-transmitter-with-fs-ia10b-2-4ghz-10ch-receiver/", R),
    ("LiPo balance charger (SkyRC IMAX B6AC V2)", 1, 5310, "Use the lab charger",
     "https://robu.in/product/imax-b6ac-v2-professional-balance-chargerdischarger/", R),
    ("PETG filament 1 kg (Bambu Lab PETG 1.75 mm)", 1, 1299, "Motor mount and joints (use lab spool)",
     "https://robu.in/product/bambu-lab-petg-1-75mm-filament-translucent-gray/", R),
    ("CA glue + 2-part epoxy (1 each)", 1, None, "Assembly (use foam-safe CA near Depron)",
     NF + " / local hardware (Fevikwik, Araldite)", R),
]

HEAD = ["Product Name", "Quantity", "Cost (List the price)", "Use Case", "Link to Product",
        "Requirement Priority", "Order Status", "Hand-off Status"]
wb = Workbook(); ws = wb.active; ws.title = "BOM"
ws.append(HEAD)
thin = Side(style="thin", color="999999"); border = Border(thin, thin, thin, thin)
for c in ws[1]:
    c.font = Font(bold=True); c.fill = PatternFill("solid", fgColor="D9D9D9")
total = 0
for name, qty, price, use, link, pri in rows:
    ws.append([name, qty, price if price is not None else "Price not found", use, link, pri, None, None])
    r = ws.max_row
    ws.cell(r, 6).fill = PatternFill("solid", fgColor=FILL[pri])
    if link.startswith("http"):
        ws.cell(r, 5).hyperlink = link; ws.cell(r, 5).style = "Hyperlink"
    if price is not None:
        ws.cell(r, 3).number_format = '"₹"#,##0'
    if pri == G and price is not None:
        total += qty * price
ws.append(["TOTAL (Green rows: qty × price)", None, total, "Excludes Green items with no price found", None, None, None, None])
tr = ws.max_row
for c in ws[tr]:
    c.font = Font(bold=True)
ws.cell(tr, 3).number_format = '"₹"#,##0'
for row in ws.iter_rows():
    for c in row:
        c.alignment = Alignment(wrap_text=True, vertical="top"); c.border = border
for col, w in zip("ABCDEFGH", [42, 10, 16, 48, 60, 30, 16, 16]):
    ws.column_dimensions[col].width = w
ws.freeze_panes = "A2"
wb.save(OUT)
print(OUT, "Green total:", total)
