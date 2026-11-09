"""
Concept A "Robust baseline" -- shared model library.

Geometry (XFLR5-expressible), component mass layout with (x, y, z) positions, CG / inertia,
AeroSandbox airplane builders for the VLM and for AeroBuildup, and helpers for strip-wise
lift extraction (critical-section CLmax) and trim.

Axes: AeroSandbox geometry axes. Origin = wing root leading edge, x aft, y right, z up. SI units.
Stage-1 sources reused (copied into lib/): mass_model_stage1.py (areal densities, avionics masses,
CF tube data), propulsion_model.py (UIUC prop data + calibrated motor model).
"""
from __future__ import annotations

import copy
import json
import math
import os
import sys
from dataclasses import dataclass, field, asdict

import numpy as np
import aerosandbox as asb

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
DAT = os.path.join(ROOT, "dat")
RES = os.path.join(ROOT, "results")
PLOTS = os.path.join(ROOT, "plots")
sys.path.insert(0, HERE)
sys.dont_write_bytecode = True

import mass_model_stage1 as mm  # noqa: E402

G = 9.81
# Site air: Hyderabad ~540 m, 30 C  -> rho 1.091, nu 1.705e-5 (matches stage-1 1.09 / 1.7e-5)
ATM = asb.Atmosphere(altitude=540.0, temperature_deviation=18.5)
ATM_HOT = asb.Atmosphere(altitude=540.0, temperature_deviation=27.5)  # rho ~1.06 (hot day)
RHO = float(ATM.density())
NU = float(ATM.kinematic_viscosity())

_AF_CACHE: dict = {}


def airfoil(name: str) -> asb.Airfoil:
    if name not in _AF_CACHE:
        _AF_CACHE[name] = asb.Airfoil(name=name, coordinates=os.path.join(DAT, name + ".dat"))
    return _AF_CACHE[name]


# =====================================================================================
# 1. GEOMETRY
# =====================================================================================
@dataclass
class Geo:
    # ---- wing: double trapezoid (rectangular centre to y_br, then straight taper) ----
    b: float = 1.40
    y_br: float = 0.28            # planform break (40 % semispan)
    c_r: float = 0.150
    c_t: float = 0.115
    y_dih: float = 0.4667         # polyhedral break (= 2nd printed-segment joint)
    dih_in: float = 0.0           # deg, panel 0..y_dih
    dih_out: float = 0.0          # deg, panel y_dih..tip
    i_w: float = 2.0              # deg root incidence (vs boom/pod datum)
    washout: float = 2.0          # deg (tip twist = i_w - washout), linear over tapered panel
    foil: str = "sd7037_te05"
    # ailerons (per side): span stations and chord fraction
    ail_y0: float = 0.25
    ail_y1: float = 0.69
    ail_cf: float = 0.25
    # ---- horizontal tail (Depron flat plate) ----
    VH: float = 0.55
    AR_h: float = 3.8
    tap_h: float = 0.75
    cf_e: float = 0.35            # elevator chord fraction (full span)
    i_h: float = -1.0             # deg stab incidence
    tail_foil: str = "flatplate-3pct"
    # ---- vertical tail ----
    VV: float = 0.032
    AR_v: float = 1.5             # h^2 / S_v
    tap_v: float = 0.60
    sweep_v: float = 25.0         # deg LE sweep
    cf_r: float = 0.38
    # ---- fuselage / layout ----
    l_t_target: float = 0.63      # m, CG -> stab quarter-MAC (sets x of tail)
    x_fw: float = -0.20           # firewall (motor mount face)
    pod_r: float = 0.025          # pod max radius (50 mm)
    z_pod: float = -0.030         # pod centre line / thrust line
    z_boom: float = -0.015
    r_boom: float = 0.004         # 8x6 carbon tube
    # derived / iterated (filled by update())
    S_h: float = 0.0245
    S_v: float = 0.0125
    x_h_le: float = 0.64
    x_v_le: float = 0.67
    x_cg_for_tail: float = 0.05

    # ------------------------------------------------------------------ wing helpers
    @property
    def s(self):
        return self.b / 2

    def chord(self, y):
        y = abs(y)
        if y <= self.y_br:
            return self.c_r
        return self.c_r + (self.c_t - self.c_r) * (y - self.y_br) / (self.s - self.y_br)

    def x_le(self, y):          # unswept quarter-chord line
        return 0.25 * (self.c_r - self.chord(y))

    def z_le(self, y):
        y = abs(y)
        zin = math.tan(math.radians(self.dih_in)) * min(y, self.y_dih)
        zout = math.tan(math.radians(self.dih_out)) * max(0.0, y - self.y_dih)
        return zin + zout

    def twist(self, y):
        y = abs(y)
        f = max(0.0, (y - self.y_br) / (self.s - self.y_br))
        return self.i_w - self.washout * f

    def wing_stations(self):
        ys = sorted(set([0.0, self.y_br, self.ail_y0, self.y_dih, self.ail_y1, self.s]))
        return [y for y in ys if 0 <= y <= self.s + 1e-9]

    @property
    def S(self):
        ys = np.linspace(0, self.s, 2001)
        return 2 * np.trapezoid([self.chord(y) for y in ys], ys)

    @property
    def AR(self):
        return self.b ** 2 / self.S

    @property
    def MAC(self):
        ys = np.linspace(0, self.s, 2001)
        return 2 / self.S * np.trapezoid([self.chord(y) ** 2 for y in ys], ys)

    @property
    def y_mac(self):
        ys = np.linspace(0, self.s, 2001)
        c = np.array([self.chord(y) for y in ys])
        return 2 / self.S * np.trapezoid(c * ys, ys)

    @property
    def x_mac_le(self):
        return self.x_le(self.y_mac)

    # ------------------------------------------------------------------ tail helpers
    @property
    def b_h(self):
        return math.sqrt(self.AR_h * self.S_h)

    @property
    def c_h_root(self):
        return 2 * self.S_h / (self.b_h * (1 + self.tap_h))

    @property
    def mac_h(self):
        cr, l = self.c_h_root, self.tap_h
        return 2 / 3 * cr * (1 + l + l * l) / (1 + l)

    @property
    def h_v(self):
        return math.sqrt(self.AR_v * self.S_v)

    @property
    def c_v_root(self):
        return 2 * self.S_v / (self.h_v * (1 + self.tap_v))

    @property
    def mac_v(self):
        cr, l = self.c_v_root, self.tap_v
        return 2 / 3 * cr * (1 + l + l * l) / (1 + l)

    @property
    def x_h_ac(self):      # quarter MAC of stab (unswept quarter-chord line)
        return self.x_h_le + 0.25 * self.c_h_root

    @property
    def x_v_ac(self):
        cr, l = self.c_v_root, self.tap_v
        z_mac = self.h_v / 3 * (1 + 2 * l) / (1 + l)
        return self.x_v_le + z_mac * math.tan(math.radians(self.sweep_v)) + 0.25 * self.mac_v

    @property
    def x_tail_end(self):
        return max(self.x_h_le + self.c_h_root, self.x_v_le + self.c_v_root) + 0.005

    def size_tail(self, x_cg, n_iter=30):
        """Set S_h, S_v and tail x-locations from V_H, V_V and the CG -> tail arm."""
        self.x_cg_for_tail = x_cg
        for _ in range(n_iter):
            x_hac = x_cg + self.l_t_target
            self.S_h = self.VH * self.S * self.MAC / self.l_t_target
            self.x_h_le = x_hac - 0.25 * self.c_h_root
            # fin: LE at 40 % of stab root chord so the rudder sits behind the elevator hinge
            self.x_v_le = self.x_h_le + 0.40 * self.c_h_root
            l_v = self.x_v_ac - x_cg
            self.S_v = self.VV * self.S * self.b / l_v

    def tail_volumes(self, x_cg):
        lh = self.x_h_ac - x_cg
        lv = self.x_v_ac - x_cg
        return dict(l_h=lh, l_v=lv, VH=self.S_h * lh / (self.S * self.MAC), VV=self.S_v * lv / (self.S * self.b))


# =====================================================================================
# 2. AEROSANDBOX AIRPLANE BUILDERS
# =====================================================================================
def _wing_xsecs(g: Geo, side=+1, mode="vlm", da=0.0, dsp=0.0, eps_dup=0.0015):
    """Cross-sections of one wing half (side=+1 right, -1 left).
    mode 'vlm': control deflections are baked into the airfoil camber (VLM ignores ControlSurface);
    duplicated stations at the aileron ends keep the deflection confined to the aileron span.
    mode 'ab': ControlSurface objects (AeroBuildup/NeuralFoil).
    da: aileron deflection [deg], right TE up / left TE down for da>0 (roll right).
    dsp: symmetric aileron deflection (spoileron <0 = TE up, flaperon >0 = TE down)."""
    af = airfoil(g.foil)
    hinge = 1 - g.ail_cf
    d_side = (-da if side > 0 else +da) + dsp
    ys = g.wing_stations()
    out = []
    for y in ys:
        def mk(yy, defl_here):
            a = af
            cs = []
            if mode == "vlm" and abs(defl_here) > 1e-9:
                a = af.add_control_surface(deflection=defl_here, hinge_point_x=hinge)
            return asb.WingXSec(xyz_le=[g.x_le(yy), side * yy, g.z_le(yy)], chord=g.chord(yy),
                                twist=g.twist(yy), airfoil=a, control_surfaces=cs)
        in_ail = g.ail_y0 <= y < g.ail_y1 - 1e-9
        if mode == "vlm":
            if abs(y - g.ail_y0) < 1e-9 and abs(d_side) > 0:
                out.append(mk(y - eps_dup, 0.0))
                out.append(mk(y, d_side))
            elif abs(y - g.ail_y1) < 1e-9 and abs(d_side) > 0:
                out.append(mk(y, d_side))
                out.append(mk(y + eps_dup, 0.0))
            else:
                out.append(mk(y, d_side if (g.ail_y0 < y < g.ail_y1) else 0.0))
        else:
            xs = mk(y, 0.0)
            if in_ail:
                xs.control_surfaces = [asb.ControlSurface(name="aileron", symmetric=False,
                                                          deflection=0.0, hinge_point=hinge)]
            out.append(xs)
    return out


def build_airplane(g: Geo, x_cg: float, z_cg: float = -0.02, mode="vlm", de=0.0, da=0.0, dsp=0.0,
                   dr=0.0, split_wing=False, tail_foil=None, with_fuselage=None) -> asb.Airplane:
    """Full aircraft. de: elevator [deg, TE down +]; da: aileron (roll-right +); dsp: symmetric
    aileron (spoileron -); dr: rudder [deg, TE left + (nose-left yaw)]."""
    tf = airfoil(tail_foil or g.tail_foil)
    wings = []
    # ---------------- wing ----------------
    if mode == "ab":
        xs = _wing_xsecs(g, +1, "ab")
        for x in xs:
            if x.control_surfaces:
                if abs(dsp) > 0:
                    x.control_surfaces = [asb.ControlSurface(name="spoileron", symmetric=True,
                                                             deflection=dsp, hinge_point=1 - g.ail_cf)]
                else:   # right side TE up for da>0; AeroBuildup mirrors an asymmetric surface with sign flip
                    x.control_surfaces = [asb.ControlSurface(name="aileron", symmetric=False,
                                                             deflection=-da, hinge_point=1 - g.ail_cf)]
        wings.append(asb.Wing(name="Main wing", symmetric=True, xsecs=xs))
    else:
        if split_wing or abs(da) > 0:
            right = _wing_xsecs(g, +1, "vlm", da=da, dsp=dsp)
            left = _wing_xsecs(g, -1, "vlm", da=da, dsp=dsp)[::-1]
            wings.append(asb.Wing(name="Main wing R", symmetric=False, xsecs=right))
            wings.append(asb.Wing(name="Main wing L", symmetric=False, xsecs=left))
        else:
            wings.append(asb.Wing(name="Main wing", symmetric=True,
                                  xsecs=_wing_xsecs(g, +1, "vlm", dsp=dsp)))
    # ---------------- horizontal tail ----------------
    hinge_e = 1 - g.cf_e
    th = tf.add_control_surface(deflection=de, hinge_point_x=hinge_e) if (mode == "vlm" and abs(de) > 0) else tf
    cs_e = [asb.ControlSurface(name="elevator", symmetric=True, deflection=de, hinge_point=hinge_e)] if mode == "ab" else []
    crh, cth = g.c_h_root, g.c_h_root * g.tap_h
    z_h = g.z_boom + g.r_boom + 0.0015
    wings.append(asb.Wing(name="Elevator (HT)", symmetric=True, xsecs=[
        asb.WingXSec(xyz_le=[g.x_h_le, 0, z_h], chord=crh, twist=g.i_h, airfoil=th, control_surfaces=cs_e),
        asb.WingXSec(xyz_le=[g.x_h_le + 0.25 * (crh - cth), g.b_h / 2, z_h], chord=cth, twist=g.i_h,
                     airfoil=th, control_surfaces=copy.deepcopy(cs_e)),
    ]))
    # ---------------- vertical tail ----------------
    hinge_r = 1 - g.cf_r
    tv = tf.add_control_surface(deflection=dr, hinge_point_x=hinge_r) if (mode == "vlm" and abs(dr) > 0) else tf
    cs_r = [asb.ControlSurface(name="rudder", symmetric=True, deflection=dr, hinge_point=hinge_r)] if mode == "ab" else []
    crv, ctv = g.c_v_root, g.c_v_root * g.tap_v
    z_v = z_h + 0.003
    wings.append(asb.Wing(name="Fin (VT)", symmetric=False, xsecs=[
        asb.WingXSec(xyz_le=[g.x_v_le, 0, z_v], chord=crv, twist=0, airfoil=tv, control_surfaces=cs_r),
        asb.WingXSec(xyz_le=[g.x_v_le + g.h_v * math.tan(math.radians(g.sweep_v)), 0, z_v + g.h_v],
                     chord=ctv, twist=0, airfoil=tv, control_surfaces=copy.deepcopy(cs_r)),
    ]))
    fus = []
    if with_fuselage if with_fuselage is not None else (mode == "ab"):
        fus = fuselages(g)
    ap = asb.Airplane(name="Concept A", xyz_ref=[x_cg, 0, z_cg], wings=wings, fuselages=fus,
                      s_ref=g.S, c_ref=g.MAC, b_ref=g.b)
    return ap


def pod_profile(g: Geo):
    """(x, r) stations of the printed pod incl. spinner. Nose = spinner tip."""
    xf = g.x_fw
    pts = [(xf - 0.050, 0.002), (xf - 0.040, 0.011), (xf - 0.025, 0.015), (xf - 0.005, 0.016),
           (xf + 0.000, 0.017), (xf + 0.040, 0.022), (xf + 0.080, g.pod_r), (0.080, g.pod_r),
           (0.140, 0.018), (0.190, 0.009), (0.210, g.r_boom + 0.001)]
    return pts


def fuselages(g: Geo):
    pod = asb.Fuselage(name="Pod", xsecs=[asb.FuselageXSec(xyz_c=[x, 0, g.z_pod + (0.0 if x < 0.08 else
                                                                                  (g.z_boom - g.z_pod) * min(1, (x - 0.08) / 0.13))],
                                                           radius=r) for x, r in pod_profile(g)])
    boom = asb.Fuselage(name="Boom", xsecs=[asb.FuselageXSec(xyz_c=[x, 0, g.z_boom], radius=g.r_boom)
                                             for x in np.linspace(0.20, g.x_tail_end, 6)])
    return [pod, boom]


def pod_wetted_area(g: Geo):
    pts = pod_profile(g)
    A = 0.0
    for (x1, r1), (x2, r2) in zip(pts[:-1], pts[1:]):
        A += math.pi * (r1 + r2) * math.hypot(x2 - x1, r2 - r1)
    # spinner (first 3 points) is not printed pod shell
    return A


# =====================================================================================
# 3. MASS LAYOUT
# =====================================================================================
@dataclass
class Equip:
    """Avionics / propulsion set (masses kg). Sources: stage-1 mass model + propulsion study."""
    fc: float = 0.023          # SpeedyBee F405 Wing Mini FC+PDB stack, 23 g (oscarliang.com; stage-1)
    rx: float = 0.002          # FlySky FS-A8S iBUS, ~2 g (task brief)
    gps: float = 0.008         # Matek M10Q-5883 (stage-1 sourced 7.5-8 g)
    airspeed: float = 0.007    # sensor board + pitot + tubing (task brief 6-8 g)
    servo: float = 0.0055      # EMAX ES9052MD x4 (stage-1 sourced)
    n_servo: int = 4
    esc: float = 0.019         # Hobbywing Skywalker 20A (stage-1 sourced)
    motor: float = 0.050       # A2212/13T 1000KV (propulsion study)
    prop: float = 0.018        # 9x5 folding blades + 30-35 mm spinner/hub (propulsion study)
    batt: float = 0.056        # 3S 650 mAh (stage-1 critique assumption 56 g)
    batt_name: str = "3S 650 mAh"


def wing_spar_masses(g: Geo, W_N: float, n_ult=6.0, sigma=400e6):
    """Stepped CF spar: 6x4 centre tube to +/- y_step (one 1 m stock piece), 4x2 outboard.
    Root bending (elliptic lift, stage-1 function) checked; tube upgraded if overstressed."""
    s = g.s
    y_step = g.y_dih
    eta = y_step / s
    M0 = n_ult * (W_N / 2) * s * mm.elliptic_moment_fraction(0.0)
    Ms = n_ult * (W_N / 2) * s * mm.elliptic_moment_fraction(eta)
    inner = ("6x4", 6.0, 4.0)
    sig0 = M0 * 0.003 / mm.cf_I(6, 4)
    if sig0 > sigma:
        inner = ("8x6", 8.0, 6.0)
        sig0 = M0 * 0.004 / mm.cf_I(8, 6)
    outer = ("4x2", 4.0, 2.0)
    sigs = Ms * 0.002 / mm.cf_I(4, 2)
    m_in = 2 * (y_step + 0.0) * mm.cf_linear_density(inner[1], inner[2])
    m_out = 2 * ((0.97 * s - y_step) + 0.06) * mm.cf_linear_density(outer[1], outer[2])
    return dict(m_inner=m_in, m_outer=m_out, inner=inner[0], outer=outer[0], y_step=y_step,
                M_root_ult=M0, M_step_ult=Ms, sigma_root=sig0, sigma_step=sigs)


def mass_layout(g: Geo, rho_fil=750.0, eq: Equip | None = None, x_batt=None, contingency=0.05,
                glue=0.04, n_ult=6.0, W_guess=5.6):
    """Component list [(name, m, x, y, z)]; wing items are spanwise-distributed (for inertia).
    rho_fil: PLA Aero foamed density kg/m^3 (design 750; check 650/850)."""
    eq = eq or Equip()
    S, s = g.S, g.s
    items = []          # (name, group, m, x, y, z)

    def add(name, grp, m, x, y=0.0, z=0.0):
        items.append([name, grp, m, x, y, z])

    # ---------------- wing (distributed in 24 strips per side) -------------
    shell_areal = 2.04 * 0.45e-3 * rho_fil * 1.35           # stage-1 wall model (kg/m^2 planform)
    ys = np.linspace(0, s, 25)
    yc = 0.5 * (ys[1:] + ys[:-1])
    dy = np.diff(ys)
    for y, d in zip(yc, dy):
        c = g.chord(y)
        for side in (+1, -1):
            add("wing shell", "wing", shell_areal * c * d, g.x_le(y) + 0.42 * c, side * y, g.z_le(y) + 0.02 * c)
    sp = None
    for _ in range(3):
        sp = wing_spar_masses(g, W_guess * 1.0, n_ult=n_ult)
        break
    lin_in = sp["m_inner"] / (2 * sp["y_step"])
    lin_out = sp["m_outer"] / (2 * ((0.97 * s - sp["y_step"]) + 0.06))
    for y, d in zip(yc, dy):
        lin = lin_in if y < sp["y_step"] else lin_out
        for side in (+1, -1):
            if y < 0.97 * s:
                add("main spar (CF)", "wing", lin * d, g.x_le(y) + 0.27 * g.chord(y), side * y, g.z_le(y) + 0.03 * g.chord(y))
            add("rear rod 2 mm (CF)", "wing", 0.97 * mm.cf_linear_density(2.0) * d, g.x_le(y) + 0.65 * g.chord(y),
                side * y, g.z_le(y) + 0.02 * g.chord(y))
    # overlap length of outer tubes inside the inner tube (telescoping)
    for side in (+1, -1):
        add("main spar (CF)", "wing", 0.06 * lin_out, g.x_le(sp["y_step"]) + 0.27 * g.chord(sp["y_step"]),
            side * (sp["y_step"] - 0.03), g.z_le(sp["y_step"]))
    n_seg = 2 * math.ceil(s / 0.24)
    seg_y = np.linspace(0, s, n_seg // 2 + 1)
    for y in seg_y[1:-1]:
        for side in (+1, -1):
            add("segment joints/ribs", "wing", 0.0015, g.x_le(y) + 0.35 * g.chord(y), side * y, g.z_le(y))
    add("segment joints/ribs", "wing", 0.0015 * 2, 0.35 * g.c_r, 0, 0)  # root ribs
    if abs(g.dih_out - g.dih_in) > 0.1:      # polyhedral keys (printed PETG + 4 mm CF rod stub)
        for side in (+1, -1):
            add("polyhedral joint keys", "wing", 0.003, g.x_le(g.y_dih) + 0.27 * g.chord(g.y_dih), side * g.y_dih, g.z_le(g.y_dih))
    if abs(g.dih_in) > 0.1:                   # root V joiner (bent 4 mm steel inside 6x4 tubes, 0.16 m)
        add("root dihedral joiner", "wing", 0.016, 0.27 * g.c_r, 0, 0.0)
    add("wing hardware (bolts, servo covers)", "wing", 0.004, 0.5 * g.c_r, 0, 0.005)
    # ---------------- fuselage / boom / tails -------------
    A_pod = pod_wetted_area(g)
    pod_shell = A_pod * 0.8e-3 * rho_fil * 0.85     # 85 %: spinner area excluded
    xs_prof = pod_profile(g)
    xc = sum(0.5 * (a[0] + b[0]) * (a[1] + b[1]) * (b[0] - a[0]) for a, b in zip(xs_prof[:-1], xs_prof[1:])) / \
        sum((a[1] + b[1]) * (b[0] - a[0]) for a, b in zip(xs_prof[:-1], xs_prof[1:]))
    add("pod shell (printed)", "fuselage", pod_shell, xc, 0, g.z_pod)
    add("motor mount (PETG)", "fuselage", 0.006, g.x_fw + 0.004, 0, g.z_pod)
    add("pod fixed: saddle, hatch, bulkheads, boom socket", "fuselage", 0.013, 0.02, 0, g.z_pod)
    x_b0, x_b1 = 0.12, g.x_tail_end
    add("tail boom 8x6 CF", "fuselage", (x_b1 - x_b0) * mm.cf_linear_density(8, 6), 0.5 * (x_b0 + x_b1), 0, g.z_boom)
    add("tail mounts (printed clamp)", "tail", 0.008, g.x_h_le + 0.4 * g.c_h_root, 0, g.z_boom)
    tail_areal = mm.tail_areal_density(mm.CONSTRUCTIONS["recommended"])
    add("horizontal tail (Depron 3 mm)", "tail", g.S_h * tail_areal + 0.001, g.x_h_le + 0.45 * g.c_h_root, 0, g.z_boom + 0.006)
    z_vmac = g.h_v / 3 * (1 + 2 * g.tap_v) / (1 + g.tap_v)
    add("vertical tail (Depron 3 mm)", "tail", g.S_v * tail_areal + 0.001,
        g.x_v_le + z_vmac * math.tan(math.radians(g.sweep_v)) + 0.45 * g.mac_v, 0, g.z_boom + 0.01 + z_vmac)
    add("tail pushrods (2 x 1 mm CF + guides)", "tail", 0.0035, 0.5 * (0.11 + g.x_h_le), 0, g.z_boom)
    # ---------------- propulsion -------------
    add("motor A2212/13T 1000KV", "propulsion", eq.motor, g.x_fw - 0.016, 0, g.z_pod)
    add("9x5 folding prop + spinner", "propulsion", eq.prop, g.x_fw - 0.038, 0, g.z_pod)
    add("ESC Skywalker 20A", "propulsion", eq.esc, g.x_fw + 0.030, 0, g.z_pod - 0.012)
    x_b = x_batt if x_batt is not None else g.x_fw + 0.090
    add("battery " + eq.batt_name, "battery", eq.batt, x_b, 0, g.z_pod - 0.006)
    # ---------------- avionics -------------
    add("flight controller (F405 wing)", "avionics", eq.fc, 0.035, 0, g.z_pod + 0.005)
    add("receiver FS-A8S", "avionics", eq.rx, 0.085, 0, g.z_pod)
    add("GPS M10Q-5883 (on wing centre top)", "avionics", eq.gps, 0.6 * g.c_r, 0, 0.016)
    add("airspeed sensor board", "avionics", eq.airspeed - 0.002, 0.07, 0, g.z_pod + 0.010)
    add("pitot tube (wing LE, y=0.30)", "avionics", 0.002, g.x_le(0.30) - 0.03, 0.30, g.z_le(0.30))
    for side in (+1, -1):
        add("aileron servo", "avionics", eq.servo, g.x_le(g.ail_y0 + 0.02) + 0.45 * g.chord(g.ail_y0),
            side * (g.ail_y0 + 0.02), g.z_le(g.ail_y0))
    add("elevator + rudder servos (in pod)", "avionics", 2 * eq.servo, 0.105, 0, g.z_pod)
    add("control hardware (horns, clevises) - tail", "avionics", 2 * 0.0015, g.x_h_le + 0.7 * g.c_h_root, 0, g.z_boom)
    for side in (+1, -1):
        add("control hardware (horns, clevises) - wing", "avionics", 0.0015, g.x_le(g.ail_y0 + 0.02) + 0.75 * g.chord(g.ail_y0),
            side * (g.ail_y0 + 0.02), g.z_le(g.ail_y0))
    wiring = 0.008 + 0.005 * g.b
    add("wiring/connectors (pod)", "avionics", 0.008, -0.04, 0, g.z_pod)
    for side in (+1, -1):
        add("wiring (wing servo leads)", "avionics", (wiring - 0.008) / 2, 0.4 * g.c_r, side * 0.15, 0.0)
    # ---------------- glue / contingency -------------
    struct = [it for it in items if it[1] in ("wing", "fuselage", "tail")]
    m_struct = sum(it[2] for it in struct)
    xs_ = sum(it[2] * it[3] for it in struct) / m_struct
    zs_ = sum(it[2] * it[5] for it in struct) / m_struct
    add("glue (4 % of structure)", "glue+contingency", glue * m_struct, xs_, 0, zs_)
    nb = [it for it in items if it[1] not in ("battery",) and not it[0].startswith("motor")]
    m_nb = sum(it[2] for it in nb)
    xnb = sum(it[2] * it[3] for it in nb) / m_nb
    znb = sum(it[2] * it[5] for it in nb) / m_nb
    add("contingency (5 % non-batt/motor)", "glue+contingency", contingency * m_nb, xnb, 0, znb)
    return items, sp


def mass_props(items):
    m = sum(it[2] for it in items)
    x = sum(it[2] * it[3] for it in items) / m
    y = sum(it[2] * it[4] for it in items) / m
    z = sum(it[2] * it[5] for it in items) / m
    Ixx = Iyy = Izz = Ixz = 0.0
    for _, _, mi, xi, yi, zi in items:
        dx, dy, dz = xi - x, yi - y, zi - z
        Ixx += mi * (dy * dy + dz * dz)
        Iyy += mi * (dx * dx + dz * dz)
        Izz += mi * (dx * dx + dy * dy)
        Ixz += mi * dx * dz
    # add self-inertia of distributed pod+boom (rod about own CG, along x) -- small, include boom
    return dict(m=m, x=x, y=y, z=z, Ixx=Ixx, Iyy=Iyy, Izz=Izz, Ixz=Ixz)


def grouped(items):
    out = {}
    for name, grp, mi, xi, yi, zi in items:
        k = name
        if k not in out:
            out[k] = [grp, 0.0, 0.0, 0.0, 0.0]
        out[k][1] += mi
        out[k][2] += mi * xi
        out[k][3] += mi * abs(yi)
        out[k][4] += mi * zi
    rows = []
    for k, (grp, mi, mx, my, mz) in out.items():
        rows.append(dict(name=k, group=grp, m=mi, x=mx / mi, y_abs=my / mi, z=mz / mi))
    return rows


# =====================================================================================
# 4. AERO HELPERS
# =====================================================================================
def op(V, alpha=0.0, beta=0.0, p=0.0, q=0.0, r=0.0, atm=None):
    return asb.OperatingPoint(atmosphere=atm or ATM, velocity=V, alpha=alpha, beta=beta, p=p, q=q, r=r)


VLM_RES = dict(spanwise_resolution=6, chordwise_resolution=8)


def run_vlm(ap, opp, strips=False, **kw):
    res = dict(VLM_RES)
    res.update(kw)
    v = asb.VortexLatticeMethod(airplane=ap, op_point=opp, **res)
    out = v.run()
    if strips:
        out["strips"] = vlm_strips(v, ap, opp, res["chordwise_resolution"])
    return out, v


def vlm_strips(v, ap, opp, nc):
    """Local lift coefficient per spanwise strip of the main wing (first wing(s) named 'Main wing*')."""
    F = np.array(v.forces_geometry)
    A = np.array(v.areas)
    C = np.array(v.vortex_centers)
    Vinf = np.array(v.steady_freestream_direction)
    q = 0.5 * float(opp.atmosphere.density()) * opp.velocity ** 2
    # count panels belonging to main wing(s)
    n_main = 0
    idx = 0
    out = []
    # panels are ordered wing by wing; for each wing, strips of nc chordwise panels
    for w in ap.wings:
        npan = 0
        # number of panels for this wing = number of spanwise strips * nc; infer from mesh
        ww = w.subdivide_sections(ratio=VLM_RES["spanwise_resolution"], spacing_function=asb.numpy.cosspace) \
            if VLM_RES["spanwise_resolution"] > 1 else w
        pts, faces = ww.mesh_thin_surface(method="quad", chordwise_resolution=nc, add_camber=True)
        npan = len(faces)
        if w.name.startswith("Main wing"):
            Fw, Aw, Cw = F[idx:idx + npan], A[idx:idx + npan], C[idx:idx + npan]
            ns = npan // nc
            for k in range(ns):
                sl = slice(k * nc, (k + 1) * nc)
                f = Fw[sl].sum(axis=0)
                fperp = f - np.dot(f, Vinf) * Vinf
                lift = np.linalg.norm(fperp) * np.sign(fperp[2])
                area = Aw[sl].sum()
                y = Cw[sl, 1].mean()
                out.append((y, lift / (q * area), area))
        idx += npan
    out.sort(key=lambda t: t[0])
    return np.array(out)


def ab_run(ap, opp, model_size="large", stab=False, **kw):
    a = asb.AeroBuildup(airplane=ap, op_point=opp, model_size=model_size)
    return a.run_with_stability_derivatives(**kw) if stab else a.run()


def save_json(name, obj):
    def conv(o):
        if isinstance(o, (np.floating, np.integer)):
            return o.item()
        if isinstance(o, np.ndarray):
            return o.tolist()
        raise TypeError(type(o))
    with open(os.path.join(RES, name), "w") as f:
        json.dump(obj, f, indent=1, default=conv)


def load_json(name):
    with open(os.path.join(RES, name)) as f:
        return json.load(f)


def geo_from_json(d):
    g = Geo()
    for k, v in d.items():
        if hasattr(g, k):
            setattr(g, k, v)
    return g


# =====================================================================================
# 5. SECTION CLmax(Re) (NeuralFoil xlarge) and CRITICAL-SECTION 3-D CLmax
# =====================================================================================
RE_GRID = [20e3, 25e3, 30e3, 35e3, 40e3, 50e3, 60e3, 70e3, 80e3, 90e3, 100e3, 120e3, 150e3, 200e3]


def section_polar(foil, Re, ncrit=9.0, alphas=None, model_size="xlarge"):
    alphas = np.arange(-6, 18.01, 0.25) if alphas is None else alphas
    a = airfoil(foil).get_aero_from_neuralfoil(alpha=alphas, Re=Re, n_crit=ncrit, model_size=model_size)
    return alphas, np.array(a["CL"]), np.array(a["CD"]), np.array(a["CM"]), np.array(a["analysis_confidence"])


def clmax_table(foil, ncrit=9.0):
    fn = os.path.join(RES, f"section_clmax_{foil}_n{int(ncrit)}.json")
    if os.path.exists(fn):
        return json.load(open(fn))
    rows = []
    for Re in RE_GRID:
        al, cl, cd, cm, conf = section_polar(foil, Re, ncrit)
        i = int(np.argmax(cl))
        rows.append(dict(Re=Re, Clmax=float(cl[i]), alpha_stall=float(al[i]), conf=float(conf[i])))
    json.dump(rows, open(fn, "w"), indent=1)
    return rows


def clmax_of_Re(foil, Re, ncrit=9.0):
    t = clmax_table(foil, ncrit)
    return np.interp(np.log(Re), np.log([r["Re"] for r in t]), [r["Clmax"] for r in t])


def vlm_eval(g, x_cg, z_cg, V, alpha, de, strips=False, atm=None, **kw):
    ap = build_airplane(g, x_cg, z_cg, mode="vlm", de=de, **kw)
    o, _ = run_vlm(ap, op(V, alpha, atm=atm), strips=strips)
    return o


def trim_vlm(g, x_cg, z_cg, V, CL_target=None, alpha=None, q_rad=0.0, atm=None):
    """Solve (alpha, de) for Cm = 0 and CL = CL_target (or at fixed alpha). Linear Newton, 2 passes."""
    a0, d0 = (alpha if alpha is not None else 3.0), 0.0
    for _ in range(3):
        f = lambda a, d: vlm_eval(g, x_cg, z_cg, V, a, d, atm=atm)
        b = f(a0, d0)
        ba = f(a0 + 1.0, d0)
        bd = f(a0, d0 - 3.0)
        CLa, Cma = ba["CL"] - b["CL"], ba["Cm"] - b["Cm"]
        CLd, Cmd = (b["CL"] - bd["CL"]) / 3.0, (b["Cm"] - bd["Cm"]) / 3.0
        if alpha is None:
            J = np.array([[CLa, CLd], [Cma, Cmd]])
            dx = np.linalg.solve(J, [CL_target - b["CL"], -b["Cm"]])
            a0, d0 = a0 + dx[0], d0 + dx[1]
        else:
            d0 = d0 - b["Cm"] / Cmd
    o = vlm_eval(g, x_cg, z_cg, V, a0, d0, strips=True, atm=atm)
    o["alpha"], o["de"] = a0, d0
    return o


def critical_section(g, x_cg, z_cg, W, foil=None, ncrit=9.0, V0=7.5, atm=None, n_iter=4):
    """3-D CLmax (1 g, trimmed) = aircraft CL when the first wing strip's VLM local Cl reaches the
    NeuralFoil section Clmax at that strip's Reynolds number. Iterates V_stall <-> Re."""
    foil = foil or g.foil
    atm = atm or ATM
    rho = float(atm.density())
    nu = float(atm.kinematic_viscosity())
    V = V0
    res = None
    for _ in range(n_iter):
        def margin(alpha):
            o = trim_vlm(g, x_cg, z_cg, V, alpha=alpha, atm=atm)
            s = o["strips"]
            c = np.array([g.chord(y) for y in s[:, 0]])
            Re = V * c / nu
            clm = clmax_of_Re(foil, Re, ncrit)
            m = clm - s[:, 1]
            k = int(np.argmin(m))
            return m[k], o, s[k, 0], Re[k], clm[k]
        a1, a2 = 8.0, 11.0
        m1 = margin(a1)[0]
        m2 = margin(a2)[0]
        for _ in range(6):
            a3 = a2 - m2 * (a2 - a1) / (m2 - m1)
            a1, m1 = a2, m2
            a2 = a3
            out = margin(a2)
            m2 = out[0]
            if abs(m2) < 2e-3:
                break
        _, o, ycrit, Recrit, clm = out
        CLmax = o["CL"]
        V = math.sqrt(2 * W / (rho * g.S * CLmax))
        res = dict(CLmax=CLmax, alpha=a2, de=o["de"], y_crit=abs(ycrit), eta_crit=abs(ycrit) / g.s,
                   Re_crit=Recrit, Clmax_sec=clm, V_stall=V, strips=o["strips"].tolist())
    return res
