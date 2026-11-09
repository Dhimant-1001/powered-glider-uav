# Airfoil Selection: Powered Glider UAV (EC4.402)

Folder: `design/03_airfoil/` · Tools: AeroSandbox 4.2.10 + NeuralFoil 0.3.3 (`xlarge` model, an XFOIL-trained surrogate) · Air: rho = 1.09 kg/m^3, nu = 1.7e-5 m^2/s (Hyderabad, 540 m, 30 C)

## 1. Recommendation

| Role | Airfoil | t / camber | Why |
|---|---|---|---|
| **Main wing (primary)** | **S7075** (Selig) | 9.0 % / 3.0 % | Ranked 1st or 2nd in 6 of 7 scoring cases and 1st in the low-Re (40-60k) case. Its drag bucket sits at section Cl 0.9-1.05, which is where a slow wing that must turn tightly flies: Cd = 0.0224 at Re 60k and 0.0188 at 80k at Cl = 1.0. It loses the least endurance at Re 40k of the three wing picks (Emax(40k)/Emax(80k) = 0.63). Cm at design Cl is a moderate -0.085, and the 9 % thickness gives 13.5 mm of spar depth at 150 mm chord. |
| Wing, alternate A | **SD7037** | 9.2 % / 3.0 % | The best-documented low-Re airfoil (UIUC wind-tunnel data). It has the lowest drag in the Cl 0.4-0.8 band (Cd 0.0243 at Cl 0.7, Re 60k, versus 0.0280 for S7075) and stalls slightly later (11.2 deg versus 10.5 deg at 60k). Choose it if sizing gives a cruise section Cl below about 0.8. |
| Wing, alternate B | **NACA 4409** | 9.0 % / 4.0 % | Highest Emax (54.9) and Clmax (1.36), and XFLR5 can generate it natively. It pays for that with Cm = -0.106 (more tail load and trim drag), Cd = 0.031 at Cl 0.7 and Re 60k, and the worst Re 40k retention of the three (0.52). |
| **Horizontal & vertical tail** | **HT14** (Drela) | 7.5 % / 0 | Its lift curve is linear through zero (dCl/da near 0 deg divided by dCl/da over 2-5 deg = 1.02 at Re 60k). NACA 0009 has a laminar-bubble "dead band" near zero (ratio 0.42) and SD8020 is worse (0.24). Cd0 = 0.0129 at Re 60k, versus 0.0150 for NACA 0009. At 7.5 % it is still printable or buildable, about 6-7.5 mm thick at 80-100 mm chord. |
| Tail, simplest build | Flat plate 3 % (3 mm balsa or Depron, round LE, sanded TE) | 3 % | Acceptable: Cd0 = 0.014 at Re 60k and linear near zero. Drag rises quickly with Cl (0.026 at Cl 0.3), but tail Cl is small. NeuralFoil confidence is low (0.77). |

**Design guidance for the sizing and stability team:**
- Size the wing so that slow cruise needs a 3-D wing CL of about 0.7-0.85 (section Cl about 0.8-0.95). That sits in the S7075 low-drag region and leaves a CLmax/CL margin of at least 1.35 (speed margin of at least 1.16 x Vstall).
- Keep the tip chord at or above about 0.11 m. That keeps Re at or above about 45k at 7 m/s. Below 40k every candidate loses 35-60 % of its Cl^1.5/Cd.
- Avoid strong taper. Use a rectangular centre panel with a modest taper (lambda of at least 0.6) and 1.5-2.5 deg of washout so the root stalls first.
- Section data for the stability work: alpha_L0 = -0.8 deg (Re 60k) to -0.9 deg (80k), Cla of about 7.8-8.9 /rad when fitted over alpha -2 to 4 deg. That is above 2*pi because the laminar bubble makes the low-Re lift curve S-shaped, so use the XFLR5 polars rather than a linear model. Cm0 is about -0.04 to -0.05 and Cm at Cl 0.7 is -0.085.

## 2. Operating Reynolds numbers (Re = V c / nu, nu = 1.7e-5)

| V \ c [m] | 0.08 | 0.10 | 0.12 | 0.14 | 0.16 | 0.18 | 0.20 |
|---|---|---|---|---|---|---|---|
| 5 m/s | 24k | 29k | 35k | 41k | 47k | 53k | 59k |
| 6 m/s | 28k | 35k | 42k | 49k | 56k | 64k | 71k |
| 7 m/s | 33k | 41k | 49k | 58k | 66k | 74k | 82k |
| 8 m/s | 38k | 47k | 56k | 66k | 75k | 85k | 94k |
| 9 m/s | 42k | 53k | 64k | 74k | 85k | 95k | 106k |
| 10 m/s | 47k | 59k | 71k | 82k | 94k | 106k | 118k |
| 12 m/s | 56k | 71k | 85k | 99k | 113k | 127k | 141k |

The 100 m x 20 m field forces turns of about 8-9 m radius. At 7 m/s that needs about 30-35 deg of bank (R = V^2/(g tan phi)), so slow flight at 6-8 m/s is mandatory. Wing Re is therefore about 50-90k at the root and 40-60k at the tip. The tail (chord about 0.08-0.10 m) runs at about 35-55k. Scoring used Re 60-80k as the baseline, with 40-60k and 80-100k bands as checks.

## 3. Method

1. **Candidates (44 sections).** Wing: AG35/36/37, AG455ct-02r, AG24/25, SD7037/7032/7062/7080/7003/7084, S3021, S4083, S4061, S7012, S7075, SA7036/7038, E387/205/193/214, MH32/30, RG15, HQ/W 2.5/9, Clark Y, NACA 2412/4412/2409/4409, S1223, GOE 417a, DAE11, AH79-100C. Plates generated in the script: curved plate 4 %/3 %t, 6 %/3 %t, 4 %/5 %t, and a 3 % flat plate. Tail: NACA 0006/0008/0009, HT08/12/13/14, SD8020, and flat plates of 3 % and 5 %. Coordinates come from the UIUC database through `asb.Airfoil(name)`, repanelled to 159 cosine-spaced points. AG455ct is not in the database, so AG455ct-02r was used. HQ/W 2.5/9 is `hq259`.
2. **NeuralFoil runs.** alpha = -6 to 16 deg in 0.25 deg steps; Re = 40k, 60k, 80k, 100k, 150k; Ncrit = 9 (baseline) and 7 (sensitivity); Mach 0; `model_size='xlarge'`.
3. **Metrics** (`results/metrics_all.csv`): Clmax and alpha_stall; stall gentleness, defined as the Cl lost in the 3 deg after the peak; Cdmin; (Cl/Cd)max and (Cl^1.5/Cd)max with the Cl at each, counted only at least 2 deg below stall so the operating point is usable; Cd at Cl 0.4, 0.7 and 1.0 (fast cruise, slow cruise, 30-35 deg bank turn); Cm(alpha=0) and Cm at Cl 0.7; alpha_L0; Cla; thickness, camber and NeuralFoil `analysis_confidence`.
4. **Score** (0-100). Each metric is normalised between the 10th and 90th percentile of the candidates and clipped, so outliers do not compress the range. Baseline weights: Cl^1.5/Cd 0.20, (L/D)max 0.075, operating-band Cd 0.10, Clmax 0.15, stall gentleness 0.10, abs(Cm) 0.075, buildability 0.15, robustness 0.15.
   - Buildability is a function of thickness: below 6 % is poor, 8.5-11 % is best, and above 13 % takes a slight penalty.
   - Robustness is half Ncrit 7-vs-9 sensitivity and half the ratio Emax(40k)/Emax(80k), which is a proxy for laminar-bubble sensitivity.
5. **Ranking robustness.** The ranking was repeated with four other weight sets (equal; efficiency-heavy; safety/low-speed-heavy; build/robustness-heavy) and two other Re bands (40-60k and 80-100k). Candidates are ordered by mean rank over the 7 cases (`results/ranking_robustness.csv`).

## 4. Ranking (ordered by mean rank over 7 cases; metric values are averages over Re 60k and 80k, Ncrit 9)

| # | Airfoil | t % | camber % | mean rank (7 cases) | baseline score | (Cl^1.5/Cd)max | (L/D)max | mean Cd @Cl 0.4/0.7/1.0 | Clmax | Cl loss 3deg past stall | abs Cm @Cl0.7 | Emax(40k)/Emax(80k) | confidence |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | S7075 | 9.0 | 3.0 | 2.1 | 76.8 | 49.3 | 49.1 | 0.0233 | 1.31 | 0.42 | 0.086 | 0.63 | 0.98 |
| 2 | NACA 4409 | 9.0 | 4.0 | 2.3 | 76.8 | 54.9 | 50.3 | 0.0260 | 1.36 | 0.42 | 0.106 | 0.52 | 0.97 |
| 3 | S4061 | 9.6 | 3.9 | 4.0 | 75.6 | 52.7 | 47.9 | 0.0288 | 1.39 | 0.45 | 0.095 | 0.41 | 0.97 |
| 4 | HQ/W 2.5/9 | 9.0 | 2.5 | 5.9 | 71.9 | 45.9 | 47.1 | 0.0233 | 1.20 | 0.34 | 0.078 | 0.64 | 0.97 |
| 5 | SA7038 | 9.2 | 3.2 | 6.3 | 71.3 | 47.1 | 45.7 | 0.0224 | 1.35 | 0.43 | 0.086 | 0.54 | 0.98 |
| 6 | E387 | 9.1 | 3.8 | 7.4 | 71.1 | 51.0 | 47.1 | 0.0289 | 1.30 | 0.36 | 0.093 | 0.45 | 0.96 |
| 7 | SD7037 | 9.2 | 3.0 | 8.4 | 69.8 | 45.5 | 45.1 | 0.0220 | 1.30 | 0.40 | 0.081 | 0.57 | 0.98 |
| 8 | MH32 | 8.7 | 2.4 | 9.6 | 67.4 | 42.6 | 44.4 | 0.0223 | 1.16 | 0.34 | 0.066 | 0.63 | 0.98 |
| 9 | SA7036 | 9.2 | 2.8 | 10.6 | 68.2 | 44.2 | 43.9 | 0.0217 | 1.29 | 0.41 | 0.075 | 0.58 | 0.98 |
| 10 | E214 | 11.1 | 4.0 | 11.3 | 70.8 | 57.2 | 49.4 | 0.0316 | 1.44 | 0.44 | 0.116 | 0.24 | 0.97 |
| 11 | SD7032 | 10.0 | 3.7 | 14.1 | 68.3 | 48.5 | 44.7 | 0.0236 | 1.45 | 0.48 | 0.097 | 0.31 | 0.98 |
| 12 | AG24 | 8.4 | 2.2 | 14.4 | 64.7 | 41.0 | 43.0 | 0.0206 | 1.18 | 0.31 | 0.069 | 0.65 | 0.98 |
| 13 | SD7080 | 9.2 | 2.5 | 15.0 | 65.1 | 41.4 | 42.9 | 0.0215 | 1.25 | 0.39 | 0.067 | 0.61 | 0.98 |
| 14 | GOE 417a | 3.2 | 5.9 | 15.6 | 66.6 | 60.8 | 55.7 | 0.0297 | 1.42 | 0.41 | 0.101 | 0.61 | 0.71 |
| 15 | S7012 | 8.8 | 2.0 | 16.7 | 63.9 | 41.0 | 42.5 | 0.0222 | 1.13 | 0.30 | 0.060 | 0.62 | 0.97 |
| 16 | NACA 4412 | 12.0 | 4.0 | 17.7 | 66.2 | 50.4 | 43.9 | 0.0309 | 1.42 | 0.42 | 0.098 | 0.12 | 0.98 |
| 17 | RG15 | 8.9 | 1.8 | 17.9 | 63.0 | 39.6 | 43.0 | 0.0239 | 1.10 | 0.30 | 0.060 | 0.66 | 0.98 |
| 18 | S3021 | 9.5 | 3.0 | 18.1 | 64.3 | 44.4 | 42.9 | 0.0232 | 1.23 | 0.38 | 0.067 | 0.42 | 0.98 |
| 19 | Clark Y | 11.7 | 3.4 | 18.3 | 63.9 | 44.5 | 41.1 | 0.0271 | 1.40 | 0.22 | 0.083 | 0.21 | 0.98 |
| 20 | E193 | 10.2 | 3.6 | 19.1 | 63.3 | 48.6 | 45.2 | 0.0317 | 1.25 | 0.36 | 0.083 | 0.36 | 0.96 |
| 21 | NACA 2409 | 9.0 | 2.0 | 19.6 | 62.8 | 39.6 | 43.5 | 0.0252 | 1.09 | 0.30 | 0.060 | 0.66 | 0.98 |
| 22 | AG35 | 8.7 | 4.4 | 20.4 | 62.5 | 39.5 | 40.7 | 0.0209 | 1.21 | 0.37 | 0.053 | 0.61 | 0.98 |
| 23 | S4083 | 8.0 | 3.5 | 22.3 | 61.7 | 45.5 | 41.0 | 0.0198 | 1.33 | 0.36 | 0.088 | 0.45 | 0.98 |
| 24 | AG25 | 7.6 | 2.4 | 22.9 | 59.7 | 41.0 | 43.8 | 0.0202 | 1.17 | 0.28 | 0.071 | 0.66 | 0.98 |
| 25 | NACA 2412 | 12.0 | 2.0 | 23.3 | 61.0 | 39.6 | 40.6 | 0.0239 | 1.24 | 0.47 | 0.060 | 0.60 | 0.98 |
| 26 | AG36 | 8.2 | 4.1 | 24.4 | 59.6 | 38.8 | 40.7 | 0.0210 | 1.17 | 0.33 | 0.051 | 0.64 | 0.98 |
| 27 | SD7084 | 9.6 | 2.3 | 26.6 | 59.3 | 39.3 | 40.3 | 0.0220 | 1.20 | 0.39 | 0.055 | 0.55 | 0.98 |
| 28 | MH30 | 7.8 | 1.8 | 27.1 | 56.1 | 36.7 | 42.1 | 0.0283 | 1.01 | 0.23 | 0.046 | 0.68 | 0.97 |
| 29 | AH79100C | 9.9 | 6.7 | 27.4 | 50.1 | 40.5 | 34.0 | 0.0517 | 1.61 | 0.10 | 0.155 | 0.16 | 0.91 |
| 30 | S1223 | 12.1 | 8.7 | 28.3 | 55.4 | 53.6 | 45.5 | 0.0435 | 1.67 | 0.48 | 0.171 | 0.21 | 0.85 |
| 31 | AG37 | 7.7 | 3.9 | 29.3 | 55.2 | 37.7 | 40.1 | 0.0212 | 1.15 | 0.31 | 0.048 | 0.65 | 0.98 |
| 32 | E205 | 10.5 | 3.0 | 29.4 | 56.3 | 44.2 | 41.4 | 0.0277 | 1.22 | 0.38 | 0.063 | 0.15 | 0.97 |
| 33 | Curved plate 6%c/3%t | 3.1 | 6.0 | 30.0 | 47.6 | 36.4 | 36.3 | 0.0305 | 1.33 | 0.19 | 0.137 | 0.80 | 0.65 |
| 34 | SD7003 | 8.5 | 1.5 | 31.7 | 48.6 | 33.0 | 35.5 | 0.0229 | 1.09 | 0.29 | 0.032 | 0.61 | 0.98 |
| 35 | AG455ct-02r | 6.5 | 1.8 | 35.0 | 39.0 | 32.3 | 36.2 | 0.0240 | 1.04 | 0.20 | 0.027 | 0.64 | 0.98 |
| 36 | DAE11 | 12.8 | 6.6 | 36.1 | 35.6 | 10.9 | 10.3 | 0.0911 | 1.31 | 0.01 | 0.120 | 0.34 | 0.83 |
| 37 | Curved plate 4%c/5%t | 5.1 | 4.0 | 36.9 | 34.5 | 25.1 | 28.2 | 0.0337 | 1.02 | 0.11 | 0.091 | 0.86 | 0.63 |
| 38 | SD7062 | 14.0 | 4.0 | 37.9 | 32.0 | 31.1 | 30.1 | 0.0274 | 1.23 | 0.37 | 0.083 | 0.13 | 0.96 |
| 39 | Flat plate 3%t | 3.0 | 0.0 | 37.9 | 32.1 | 8.3 | 12.5 | 0.0773 | 0.83 | 0.00 | 0.027 | 1.06 | 0.72 |
| 40 | Curved plate 4%c/3%t | 3.0 | 4.0 | 38.7 | 29.2 | 27.1 | 31.2 | 0.0489 | 1.06 | 0.03 | 0.098 | 0.86 | 0.58 |

Rank of the leaders in each case. Cases: 1 baseline at 60-80k; 2 equal weights; 3 efficiency-heavy; 4 safety-heavy; 5 build-heavy; 6 baseline at 40-60k; 7 baseline at 80-100k.

| Airfoil | 1 | 2 | 3 | 4 | 5 | 6 (40-60k) | 7 (80-100k) | mean |
|---|---|---|---|---|---|---|---|---|
| S7075 | 2 | 1 | 2 | 3 | 1 | **1** | 5 | 2.1 |
| NACA 4409 | 1 | 3 | 1 | 1 | 2 | 5 | 3 | 2.3 |
| S4061 | 3 | 4 | 3 | 2 | 4 | 11 | 1 | 4.0 |
| HQ/W 2.5/9 | 4 | 2 | 8 | 10 | 3 | 2 | 12 | 5.9 |
| SA7038 | 5 | 6 | 6 | 8 | 6 | 6 | 7 | 6.3 |
| E387 | 6 | 9 | 5 | 7 | 8 | 15 | 2 | 7.4 |
| SD7037 | 8 | 7 | 10 | 13 | 7 | 3 | 11 | 8.4 |

**Reading the ranking:**
- **S4061 and E387 drop out.** They do well at Re 80-100k but fall to 11th and 15th at 40-60k. E387 shows its well-known laminar-bubble drag hump: Cd = 0.037 at Cl 0.7 and Re 60k, and 0.063 at Re 40k.
- **HQ/W 2.5/9 and MH32 drop out.** They have low drag and low Cm, but their Clmax of 1.16-1.20 is too low for slow, banked flight.
- **E214, SD7032, Clark Y, NACA 4412 and SD7062 drop out.** These 10-14 % sections have good lift but are heavily Re-sensitive: Emax(40k)/Emax(80k) = 0.13-0.31.
- **AG35/36/37 and AG455ct drop out.** These Drela DLG sections have very low drag at Cl 0.2-0.6 and the smallest Cm, but they are thin (6.5-8.7 %), give a lower Clmax (1.15-1.21) and their L/D peaks at Cl around 0.6-0.8. They suit faster flight and are a poor fit for slow turns at Cl around 1.
- **S1223, AH79-100C and DAE11 drop out.** These high-lift or human-powered-aircraft sections reach Clmax 1.6-1.7 but have Cd of 0.04-0.09 in the operating band at Re 60-80k, Cm of -0.12 to -0.17, and poor low-Re behaviour.
- **The high-lift references are a poor fit.** S1223 is not appropriate at this Re and size.

## 5. Detailed metrics for the leaders (Ncrit 9)

| Airfoil | Re | Clmax | a_stall | Cl drop 3deg post-stall | Cdmin | (L/D)max @Cl | (Cl^1.5/Cd)max @Cl | Cd@Cl0.4 | Cd@Cl0.7 | Cd@Cl1.0 | Cm(a=0) | Cm@Cl0.7 | a_L0 | Cla /rad | conf |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| S7075 | 40k | 1.232 | 10.0 | 0.36 | 0.0217 | 32.5 @1.06 | 33.5 @1.06 | 0.0356 | 0.0420 | 0.0325 | -0.044 | -0.082 | -0.86 | 5.54 | 0.98 |
| S7075 | 60k | 1.296 | 10.5 | 0.40 | 0.0192 | 44.9 @1.01 | 45.3 @1.03 | 0.0271 | 0.0280 | 0.0224 | -0.044 | -0.085 | -0.79 | 7.84 | 0.98 |
| S7075 | 80k | 1.332 | 11.0 | 0.43 | 0.0177 | 53.3 @1.00 | 53.2 @1.00 | 0.0223 | 0.0210 | 0.0188 | -0.051 | -0.087 | -0.92 | 8.94 | 0.98 |
| S7075 | 100k | 1.351 | 11.0 | 0.44 | 0.0154 | 60.0 @0.98 | 59.5 @1.00 | 0.0190 | 0.0170 | 0.0168 | -0.065 | -0.089 | -1.23 | 9.02 | 0.98 |
| S7075 | 150k | 1.376 | 11.8 | 0.41 | 0.0118 | 72.0 @0.92 | 70.1 @0.96 | 0.0139 | 0.0120 | 0.0144 | -0.085 | -0.090 | -1.97 | 7.97 | 0.98 |
| NACA 4409 | 40k | 1.314 | 10.8 | 0.38 | 0.0265 | 28.9 @1.19 | 31.6 @1.19 | 0.0381 | 0.0529 | 0.0565 | -0.062 | -0.098 | -1.50 | 6.51 | 0.97 |
| NACA 4409 | 60k | 1.347 | 11.0 | 0.42 | 0.0237 | 45.4 @1.19 | 49.5 @1.19 | 0.0275 | 0.0312 | 0.0325 | -0.082 | -0.104 | -1.58 | 8.30 | 0.97 |
| NACA 4409 | 80k | 1.369 | 11.0 | 0.41 | 0.0202 | 55.2 @1.19 | 60.2 @1.19 | 0.0211 | 0.0212 | 0.0223 | -0.099 | -0.108 | -2.08 | 8.25 | 0.97 |
| NACA 4409 | 100k | 1.380 | 11.2 | 0.40 | 0.0165 | 62.2 @1.18 | 67.5 @1.18 | 0.0169 | 0.0165 | 0.0183 | -0.104 | -0.108 | -2.53 | 7.66 | 0.97 |
| NACA 4409 | 150k | 1.390 | 11.5 | 0.33 | 0.0115 | 74.4 @1.14 | 79.6 @1.16 | 0.0116 | 0.0120 | 0.0141 | -0.108 | -0.105 | -3.27 | 6.64 | 0.98 |
| S4061 | 40k | 1.319 | 11.0 | 0.39 | 0.0266 | 21.8 @1.17 | 23.6 @1.17 | 0.0406 | 0.0559 | 0.0634 | -0.054 | -0.092 | -0.78 | 6.54 | 0.97 |
| S4061 | 60k | 1.383 | 12.0 | 0.44 | 0.0241 | 43.1 @1.23 | 47.8 @1.23 | 0.0302 | 0.0370 | 0.0335 | -0.083 | -0.094 | -1.62 | 7.36 | 0.98 |
| S4061 | 80k | 1.404 | 12.2 | 0.46 | 0.0201 | 52.6 @1.18 | 57.7 @1.20 | 0.0226 | 0.0262 | 0.0236 | -0.095 | -0.097 | -2.66 | 6.86 | 0.97 |
| S4061 | 100k | 1.414 | 12.2 | 0.41 | 0.0164 | 60.0 @1.17 | 65.0 @1.19 | 0.0173 | 0.0199 | 0.0190 | -0.100 | -0.096 | -3.32 | 6.38 | 0.98 |
| S4061 | 150k | 1.425 | 13.0 | 0.34 | 0.0119 | 74.1 @1.10 | 78.6 @1.15 | 0.0121 | 0.0135 | 0.0143 | -0.094 | -0.090 | -4.06 | 6.19 | 0.98 |
| HQ/W 2.5/9 | 40k | 1.154 | 9.8 | 0.30 | 0.0209 | 31.3 @1.00 | 31.4 @1.02 | 0.0337 | 0.0387 | 0.0320 | -0.035 | -0.075 | -0.59 | 5.53 | 0.98 |
| HQ/W 2.5/9 | 60k | 1.190 | 10.0 | 0.33 | 0.0184 | 43.1 @0.96 | 42.3 @0.96 | 0.0261 | 0.0267 | 0.0241 | -0.034 | -0.077 | -0.47 | 7.78 | 0.98 |
| HQ/W 2.5/9 | 80k | 1.212 | 10.2 | 0.35 | 0.0171 | 51.1 @0.93 | 49.4 @0.95 | 0.0213 | 0.0205 | 0.0213 | -0.044 | -0.079 | -0.75 | 8.88 | 0.97 |
| HQ/W 2.5/9 | 100k | 1.223 | 10.2 | 0.35 | 0.0153 | 57.8 @0.91 | 55.2 @0.93 | 0.0175 | 0.0167 | 0.0194 | -0.061 | -0.080 | -1.14 | 8.74 | 0.97 |
| HQ/W 2.5/9 | 150k | 1.237 | 10.5 | 0.32 | 0.0118 | 70.5 @0.87 | 65.9 @0.89 | 0.0120 | 0.0118 | 0.0167 | -0.075 | -0.077 | -2.14 | 7.39 | 0.96 |
| SA7038 | 40k | 1.214 | 9.2 | 0.33 | 0.0225 | 26.5 @1.07 | 27.5 @1.07 | 0.0350 | 0.0410 | 0.0394 | -0.043 | -0.084 | -0.92 | 6.81 | 0.98 |
| SA7038 | 60k | 1.344 | 11.5 | 0.42 | 0.0200 | 41.3 @1.05 | 43.0 @1.11 | 0.0257 | 0.0253 | 0.0245 | -0.053 | -0.087 | -0.92 | 9.03 | 0.98 |
| SA7038 | 80k | 1.366 | 12.0 | 0.44 | 0.0178 | 50.0 @1.01 | 51.1 @1.08 | 0.0201 | 0.0186 | 0.0200 | -0.074 | -0.086 | -1.40 | 8.83 | 0.98 |
| SA7038 | 100k | 1.373 | 12.2 | 0.41 | 0.0152 | 56.9 @0.99 | 57.4 @1.06 | 0.0163 | 0.0153 | 0.0176 | -0.083 | -0.084 | -1.87 | 8.00 | 0.98 |
| SA7038 | 150k | 1.377 | 13.0 | 0.33 | 0.0112 | 69.0 @0.94 | 68.5 @1.03 | 0.0113 | 0.0115 | 0.0146 | -0.089 | -0.081 | -2.68 | 6.70 | 0.98 |
| E387 | 40k | 1.275 | 11.2 | 0.35 | 0.0261 | 23.9 @1.17 | 25.9 @1.17 | 0.0386 | 0.0629 | 0.0622 | -0.055 | -0.089 | -0.80 | 6.69 | 0.95 |
| E387 | 60k | 1.293 | 11.2 | 0.37 | 0.0216 | 41.5 @1.17 | 45.0 @1.17 | 0.0263 | 0.0373 | 0.0394 | -0.090 | -0.093 | -2.15 | 6.45 | 0.96 |
| E387 | 80k | 1.313 | 11.2 | 0.36 | 0.0161 | 52.7 @1.17 | 57.0 @1.17 | 0.0181 | 0.0250 | 0.0274 | -0.097 | -0.093 | -3.06 | 6.03 | 0.96 |
| E387 | 100k | 1.324 | 11.2 | 0.33 | 0.0132 | 60.5 @1.16 | 65.2 @1.16 | 0.0148 | 0.0192 | 0.0216 | -0.093 | -0.090 | -3.53 | 6.01 | 0.97 |
| E387 | 150k | 1.329 | 11.5 | 0.29 | 0.0106 | 74.1 @1.13 | 78.9 @1.15 | 0.0114 | 0.0137 | 0.0155 | -0.086 | -0.084 | -3.83 | 6.24 | 0.97 |
| SD7037 | 40k | 1.206 | 9.5 | 0.34 | 0.0215 | 27.4 @1.04 | 28.3 @1.09 | 0.0342 | 0.0394 | 0.0371 | -0.039 | -0.080 | -0.84 | 6.77 | 0.98 |
| SD7037 | 60k | 1.293 | 11.2 | 0.40 | 0.0190 | 40.9 @1.00 | 41.7 @1.07 | 0.0250 | 0.0243 | 0.0244 | -0.046 | -0.082 | -0.81 | 8.99 | 0.98 |
| SD7037 | 80k | 1.306 | 11.8 | 0.40 | 0.0171 | 49.3 @0.96 | 49.4 @1.03 | 0.0196 | 0.0181 | 0.0204 | -0.066 | -0.081 | -1.27 | 8.96 | 0.98 |
| SD7037 | 100k | 1.311 | 12.0 | 0.35 | 0.0150 | 55.9 @0.94 | 55.4 @1.01 | 0.0160 | 0.0150 | 0.0181 | -0.076 | -0.079 | -1.63 | 8.20 | 0.97 |
| SD7037 | 150k | 1.317 | 12.8 | 0.31 | 0.0111 | 67.5 @0.92 | 66.1 @0.99 | 0.0113 | 0.0114 | 0.0151 | -0.084 | -0.076 | -2.42 | 6.83 | 0.97 |

## 6. Polar tables for the orchestrator and sizing team (Ncrit 9)

`results/polar_tables.csv` is the machine-readable version, with Re 40-100k, Cl, Cd, Cm, L/D, Cl^1.5/Cd and confidence. `results/polars.json` holds the full alpha sweep for every candidate, Re and Ncrit.

Cells: Cl / Cd x 1e4 / Cm(c/4), NeuralFoil xlarge, Ncrit 9, Mach 0

| Airfoil | Re | a=-2 | a=0 | a=2 | a=3 | a=4 | a=5 | a=6 | a=7 | a=8 | a=10 | a=12 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| S7075 | 60k | -0.09 / 230 / -0.052 | 0.08 / 213 / -0.044 | 0.37 / 266 / -0.067 | 0.54 / 290 / -0.081 | 0.70 / 280 / -0.085 | 0.88 / 239 / -0.086 | 0.99 / 223 / -0.078 | 1.06 / 251 / -0.067 | 1.14 / 312 / -0.060 | 1.29 / 502 / -0.052 | 1.08 / 1039 / -0.047 |
| S7075 | 80k | -0.10 / 206 / -0.050 | 0.12 / 197 / -0.051 | 0.48 / 227 / -0.082 | 0.62 / 221 / -0.085 | 0.78 / 197 / -0.087 | 0.91 / 181 / -0.083 | 1.00 / 187 / -0.076 | 1.06 / 222 / -0.067 | 1.13 / 276 / -0.060 | 1.30 / 438 / -0.055 | 1.26 / 768 / -0.033 |
| S7075 | 100k | -0.10 / 185 / -0.051 | 0.20 / 180 / -0.065 | 0.53 / 188 / -0.086 | 0.68 / 173 / -0.088 | 0.81 / 158 / -0.087 | 0.91 / 155 / -0.082 | 1.00 / 168 / -0.076 | 1.07 / 203 / -0.068 | 1.13 / 254 / -0.061 | 1.31 / 392 / -0.056 | 1.31 / 668 / -0.036 |
| NACA 4409 | 60k | -0.05 / 278 / -0.064 | 0.24 / 252 / -0.082 | 0.54 / 295 / -0.102 | 0.67 / 308 / -0.104 | 0.79 / 326 / -0.105 | 0.91 / 336 / -0.105 | 1.05 / 314 / -0.102 | 1.16 / 277 / -0.093 | 1.19 / 282 / -0.073 | 1.30 / 490 / -0.056 | 1.30 / 818 / -0.037 |
| NACA 4409 | 80k | 0.01 / 243 / -0.075 | 0.36 / 209 / -0.099 | 0.64 / 212 / -0.107 | 0.76 / 214 / -0.107 | 0.88 / 219 / -0.106 | 0.99 / 223 / -0.103 | 1.09 / 223 / -0.098 | 1.18 / 214 / -0.090 | 1.19 / 260 / -0.072 | 1.31 / 449 / -0.058 | 1.33 / 758 / -0.040 |
| NACA 4409 | 100k | 0.09 / 214 / -0.088 | 0.41 / 169 / -0.104 | 0.68 / 165 / -0.108 | 0.79 / 169 / -0.106 | 0.90 / 175 / -0.104 | 1.00 / 183 / -0.101 | 1.09 / 188 / -0.096 | 1.18 / 189 / -0.089 | 1.19 / 248 / -0.073 | 1.31 / 417 / -0.059 | 1.34 / 702 / -0.042 |
| S4061 | 60k | -0.06 / 277 / -0.066 | 0.24 / 265 / -0.083 | 0.49 / 324 / -0.092 | 0.61 / 351 / -0.093 | 0.72 / 373 / -0.093 | 0.85 / 376 / -0.092 | 0.99 / 338 / -0.089 | 1.12 / 298 / -0.082 | 1.21 / 282 / -0.073 | 1.29 / 392 / -0.052 | 1.38 / 620 / -0.042 |
| S4061 | 80k | 0.10 / 233 / -0.091 | 0.35 / 218 / -0.095 | 0.59 / 252 / -0.099 | 0.70 / 262 / -0.097 | 0.82 / 260 / -0.094 | 0.93 / 245 / -0.090 | 1.04 / 230 / -0.084 | 1.14 / 222 / -0.079 | 1.22 / 235 / -0.072 | 1.28 / 361 / -0.053 | 1.40 / 559 / -0.044 |
| S4061 | 100k | 0.18 / 191 / -0.100 | 0.42 / 175 / -0.100 | 0.63 / 197 / -0.097 | 0.74 / 200 / -0.095 | 0.85 / 198 / -0.091 | 0.95 / 193 / -0.087 | 1.05 / 189 / -0.082 | 1.14 / 192 / -0.078 | 1.22 / 212 / -0.072 | 1.28 / 339 / -0.053 | 1.41 / 516 / -0.045 |
| HQ/W 2.5/9 | 60k | -0.13 / 209 / -0.037 | 0.04 / 199 / -0.034 | 0.32 / 248 / -0.054 | 0.51 / 277 / -0.072 | 0.68 / 272 / -0.077 | 0.84 / 230 / -0.074 | 0.94 / 220 / -0.065 | 1.02 / 254 / -0.056 | 1.08 / 318 / -0.048 | 1.19 / 566 / -0.035 | 0.90 / 1300 / -0.051 |
| HQ/W 2.5/9 | 80k | -0.14 / 190 / -0.036 | 0.10 / 183 / -0.044 | 0.45 / 216 / -0.073 | 0.60 / 218 / -0.078 | 0.75 / 197 / -0.078 | 0.86 / 178 / -0.071 | 0.95 / 186 / -0.063 | 1.02 / 224 / -0.056 | 1.08 / 283 / -0.048 | 1.21 / 511 / -0.038 | 1.00 / 1056 / -0.035 |
| HQ/W 2.5/9 | 100k | -0.12 / 171 / -0.041 | 0.20 / 165 / -0.061 | 0.51 / 178 / -0.078 | 0.65 / 172 / -0.081 | 0.77 / 159 / -0.077 | 0.87 / 153 / -0.070 | 0.95 / 169 / -0.063 | 1.02 / 206 / -0.057 | 1.08 / 262 / -0.050 | 1.22 / 473 / -0.039 | 1.05 / 942 / -0.029 |
| SA7038 | 60k | -0.10 / 244 / -0.049 | 0.13 / 223 / -0.053 | 0.49 / 264 / -0.083 | 0.64 / 258 / -0.086 | 0.79 / 243 / -0.085 | 0.90 / 236 / -0.081 | 1.00 / 244 / -0.075 | 1.09 / 266 / -0.070 | 1.18 / 301 / -0.064 | 1.30 / 408 / -0.049 | 1.33 / 651 / -0.033 |
| SA7038 | 80k | -0.08 / 220 / -0.051 | 0.26 / 194 / -0.074 | 0.57 / 196 / -0.087 | 0.70 / 186 / -0.086 | 0.81 / 182 / -0.082 | 0.91 / 186 / -0.077 | 1.01 / 201 / -0.073 | 1.10 / 226 / -0.069 | 1.18 / 258 / -0.064 | 1.30 / 352 / -0.050 | 1.37 / 553 / -0.036 |
| SA7038 | 100k | -0.02 / 195 / -0.062 | 0.32 / 163 / -0.083 | 0.60 / 156 / -0.088 | 0.72 / 153 / -0.084 | 0.82 / 154 / -0.080 | 0.91 / 162 / -0.076 | 1.01 / 178 / -0.072 | 1.10 / 202 / -0.068 | 1.18 / 231 / -0.063 | 1.29 / 318 / -0.050 | 1.37 / 500 / -0.036 |
| E387 | 60k | 0.03 / 269 / -0.078 | 0.31 / 243 / -0.090 | 0.53 / 301 / -0.094 | 0.62 / 338 / -0.094 | 0.71 / 378 / -0.093 | 0.80 / 418 / -0.091 | 0.92 / 426 / -0.088 | 1.08 / 349 / -0.081 | 1.17 / 282 / -0.067 | 1.23 / 462 / -0.042 | 1.26 / 773 / -0.030 |
| E387 | 80k | 0.16 / 206 / -0.096 | 0.41 / 183 / -0.097 | 0.61 / 226 / -0.095 | 0.71 / 251 / -0.093 | 0.80 / 274 / -0.091 | 0.90 / 283 / -0.088 | 1.02 / 271 / -0.084 | 1.12 / 237 / -0.077 | 1.17 / 232 / -0.064 | 1.24 / 427 / -0.044 | 1.28 / 730 / -0.032 |
| E387 | 100k | 0.20 / 168 / -0.098 | 0.42 / 150 / -0.093 | 0.63 / 181 / -0.091 | 0.73 / 197 / -0.089 | 0.83 / 211 / -0.087 | 0.93 / 218 / -0.085 | 1.03 / 213 / -0.081 | 1.13 / 194 / -0.075 | 1.18 / 214 / -0.063 | 1.25 / 401 / -0.045 | 1.29 / 694 / -0.033 |
| SD7037 | 60k | -0.10 / 229 / -0.046 | 0.11 / 214 / -0.046 | 0.47 / 255 / -0.079 | 0.63 / 250 / -0.082 | 0.77 / 236 / -0.081 | 0.88 / 230 / -0.075 | 0.98 / 240 / -0.070 | 1.07 / 266 / -0.065 | 1.15 / 306 / -0.059 | 1.27 / 422 / -0.043 | 1.28 / 669 / -0.026 |
| SD7037 | 80k | -0.10 / 207 / -0.046 | 0.22 / 188 / -0.066 | 0.55 / 191 / -0.083 | 0.68 / 182 / -0.081 | 0.79 / 178 / -0.077 | 0.89 / 184 / -0.072 | 0.99 / 200 / -0.068 | 1.08 / 228 / -0.063 | 1.15 / 262 / -0.057 | 1.26 / 367 / -0.043 | 1.31 / 582 / -0.028 |
| SD7037 | 100k | -0.06 / 187 / -0.054 | 0.30 / 160 / -0.076 | 0.58 / 152 / -0.083 | 0.70 / 150 / -0.079 | 0.80 / 152 / -0.075 | 0.89 / 162 / -0.071 | 0.99 / 178 / -0.067 | 1.08 / 204 / -0.063 | 1.15 / 235 / -0.057 | 1.25 / 335 / -0.043 | 1.31 / 535 / -0.029 |
| HT14 | 60k | -0.25 / 143 / +0.007 | -0.00 / 129 / +0.000 | 0.26 / 143 / -0.008 | 0.35 / 176 / +0.000 | 0.45 / 211 / +0.005 | 0.55 / 257 / +0.008 | 0.64 / 329 / +0.011 | 0.73 / 494 / +0.007 | 0.68 / 986 / -0.037 | 0.69 / 1312 / -0.047 | 0.73 / 1592 / -0.058 |
| HT14 | 80k | -0.25 / 126 / +0.004 | -0.00 / 112 / +0.000 | 0.25 / 126 / -0.004 | 0.34 / 153 / +0.002 | 0.45 / 182 / +0.005 | 0.55 / 219 / +0.007 | 0.65 / 276 / +0.009 | 0.74 / 385 / +0.012 | 0.72 / 834 / -0.017 | 0.70 / 1268 / -0.043 | 0.74 / 1564 / -0.055 |
| HT14 | 100k | -0.24 / 115 / +0.001 | -0.00 / 100 / +0.000 | 0.24 / 115 / -0.001 | 0.34 / 138 / +0.003 | 0.45 / 163 / +0.005 | 0.55 / 195 / +0.006 | 0.65 / 243 / +0.008 | 0.75 / 321 / +0.011 | 0.80 / 545 / +0.003 | 0.71 / 1228 / -0.041 | 0.74 / 1540 / -0.054 |
| NACA 0009 | 60k | -0.12 / 169 / -0.010 | 0.00 / 150 / +0.000 | 0.12 / 169 / +0.010 | 0.38 / 174 / -0.019 | 0.48 / 178 / -0.005 | 0.56 / 224 / +0.004 | 0.65 / 294 / +0.008 | 0.73 / 397 / +0.013 | 0.77 / 586 / +0.017 | 0.63 / 1210 / -0.024 | 0.66 / 1497 / -0.031 |
| NACA 0009 | 80k | -0.19 / 147 / +0.001 | 0.00 / 132 / -0.000 | 0.19 / 147 / -0.001 | 0.40 / 143 / -0.018 | 0.48 / 153 / -0.004 | 0.56 / 198 / +0.003 | 0.65 / 258 / +0.007 | 0.74 / 345 / +0.011 | 0.78 / 505 / +0.017 | 0.64 / 1151 / -0.017 | 0.67 / 1462 / -0.028 |
| NACA 0009 | 100k | -0.25 / 128 / +0.012 | 0.00 / 122 / -0.000 | 0.25 / 128 / -0.012 | 0.40 / 126 / -0.013 | 0.48 / 139 / -0.004 | 0.56 / 182 / +0.002 | 0.65 / 233 / +0.005 | 0.74 / 308 / +0.009 | 0.80 / 443 / +0.016 | 0.66 / 1089 / -0.012 | 0.67 / 1434 / -0.026 |
| Flat plate 3%t | 60k | -0.25 / 236 / -0.002 | -0.00 / 141 / +0.000 | 0.25 / 236 / +0.002 | 0.37 / 300 / +0.005 | 0.49 / 427 / -0.019 | 0.57 / 573 / -0.024 | 0.65 / 711 / -0.013 | 0.68 / 867 / -0.014 | 0.69 / 1026 / -0.019 | 0.71 / 1306 / -0.030 | 0.74 / 1579 / -0.042 |
| Flat plate 3%t | 80k | -0.25 / 221 / -0.002 | -0.00 / 152 / +0.000 | 0.25 / 221 / +0.002 | 0.36 / 287 / +0.006 | 0.47 / 405 / -0.003 | 0.57 / 532 / -0.001 | 0.63 / 667 / +0.001 | 0.67 / 822 / -0.005 | 0.68 / 988 / -0.014 | 0.71 / 1280 / -0.029 | 0.74 / 1561 / -0.042 |
| Flat plate 3%t | 100k | -0.25 / 209 / -0.002 | -0.00 / 160 / +0.000 | 0.25 / 209 / +0.002 | 0.36 / 277 / +0.006 | 0.46 / 382 / +0.001 | 0.55 / 504 / +0.005 | 0.62 / 634 / +0.005 | 0.67 / 794 / -0.002 | 0.68 / 964 / -0.013 | 0.70 / 1262 / -0.029 | 0.74 / 1549 / -0.043 |

## 7. Foam-sheet or plate construction alternative

If the wing is built from sheet foam (Depron or sunboard) rather than a printed or rib structure, a thin curved plate is the natural profile. Results at Re 60k / 80k (NeuralFoil):

| Profile | Clmax | (Cl^1.5/Cd)max | (L/D)max | Cd @Cl 0.7 | Cm @Cl 0.7 | Emax 40k/80k | confidence |
|---|---|---|---|---|---|---|---|
| S7075 (reference) | 1.30 / 1.33 | 45.3 / 53.2 | 44.9 / 53.3 | 0.0280 / 0.0210 | -0.085 / -0.087 | 0.63 | 0.98 |
| Curved plate, 6 % camber, 3 %t | 1.33 / 1.32 | 34.8 / 38.1 | 34.2 / 38.5 | 0.0307 / 0.0292 | -0.132 / -0.142 | 0.80 | **0.61-0.69** |
| Curved plate, 4 % camber, 3 %t | 1.06 / 1.07 | 26.2 / 28.0 | 29.8 / 32.7 | 0.0237 / 0.0214 | -0.096 / -0.100 | 0.86 | **0.58** |
| Curved plate, 4 % camber, 5 %t (5 mm sheet at 100 mm chord) | 1.02 / 1.02 | 24.3 / 26.0 | 27.2 / 29.2 | 0.0276 / 0.0257 | -0.089 / -0.093 | 0.86 | **0.57-0.70** |
| GOE 417a (3.2 %t, 5.9 % camber, thin classic) | 1.41 / 1.43 | 55.4 / 66.2 | 50.4 / 61.0 | 0.0345 / 0.0282 | -0.100 / -0.103 | 0.61 | **0.69-0.73** |
| Flat plate 3 %t | 0.83 | 8.3 | 12.5 | >0.12 | ~0 | 1.06 | 0.66-0.77 |

**Interpretation:**
- NeuralFoil is outside its training distribution for plates (confidence 0.58-0.73), so these numbers are only indicative.
- Wind-tunnel literature finds that thin plates with 4-6 % camber are competitive with, or better than, conventional sections below about Re 70k, and are insensitive to Re and to turbulence (Laitone 1997; Pelletier & Mueller 2000).
- The plates' retention at 40k (0.80-0.86, versus 0.57-0.63 for the airfoils) is consistent with that literature.
- **Practical verdict:** a 5-6 % cambered sheet wing is a valid fallback for a pure-foam build, but it has three drawbacks:
  - It gives roughly 25-30 % lower predicted endurance than S7075 at Re 60-80k.
  - It has a stronger nose-down Cm (-0.13 to -0.14 for 6 % camber), so it needs a larger tail load.
  - It provides no spar depth, so a high-AR plate wing needs an external or bonded carbon spar and is torsionally soft.
- For a high-aspect-ratio wing built with a P1S printer available, a 9 % section (S7075) built with printed LW-PLA ribs or shells, or a foam core around a carbon tube spar, is recommended.
- If the construction team chooses sheet foam, use the 6 % camber, 3 %t curved plate as the XFLR5 section and validate it in XFLR5/XFOIL (XFOIL handles thin plates poorly; expect convergence gaps).

## 8. Buildability notes (3D printing / balsa)

- **Thickness.** At 9 %: 150 mm chord gives 13.5 mm depth (at about 30 % chord for S7075). That fits an 8-10 mm carbon tube spar plus about 1.5-2 mm of skin on each side. At 120 mm chord it gives 10.8 mm, which fits an 8 mm tube.
- **Trailing edge.** The FDM printer cannot make a zero-thickness TE. `*_te05.dat` files have a 0.5 %c blunt TE (about 0.7 mm at 140 mm chord), which is printable as 2 perimeters with a 0.4 mm nozzle.
  - NeuralFoil effect at Re 70k (alpha 0/4/8 deg): S7075 dCd = +6/-5/+15 e-4 and dCl = +0.07/+0.05/+0.04; SD7037 dCd = +4/+3/+5 e-4; HT14 is negligible.
  - Analyse the TE05 file in XFLR5 if that is what will be built. In XFLR5 you can also use Design -> Set T.E. Gap (0.5 %).
- **Surface quality.**
  - At Re 50-90k the laminar separation bubble dominates drag.
  - Layer lines on top of the section act like a weak turbulator. That is not harmful here, and may even help below Re 60k, but uncontrolled bumps or waviness ahead of about 30 % chord change the bubble location.
  - Print the leading 30 % smooth, with the seam on the lower surface near the TE.
  - Treat a 0.3-0.5 mm zig-zag turbulator strip at 20-30 % chord on the upper surface as an optional flight-test experiment.
  - NeuralFoil's Ncrit 7-vs-9 sensitivity was small for all leaders (Emax change below 3 %; S7075 -0.4 %). Real XFOIL usually shows more, so check it in XFLR5.
- **Undercamber.** S7075, SD7037 and NACA 4409 are flat or slightly concave aft on the lower surface. S7075 has a small aft concavity, which is easy to print and needs care if building with sheeted foam. NACA 4409 has the flattest lower surface, which makes it the easiest to build on a flat table.

## 9. Caveats: NeuralFoil versus XFLR5 / XFOIL

- NeuralFoil is a physics-informed neural surrogate trained on tens of millions of XFOIL runs. The authors report mean drag errors against XFOIL of about 0.4 % on simple cases and about 2 % on a test set that includes transitional and post-stall cases (Sharpe & Hansman 2025). It therefore inherits XFOIL's assumptions: 2-D, steady flow, e^N transition and a viscous-inviscid interaction model.
- At Re 40-100k, XFOIL (and so NeuralFoil) typically:
  - overpredicts Clmax and under-predicts post-stall hysteresis;
  - models laminar bubble bursting only approximately;
  - gives drag that can be 10-30 % lower than wind-tunnel values. Wind-tunnel drag at low Re also varies along the span (Guglielmo & Selig 1996).
- Treat absolute L/D as optimistic. The **ranking** between similar 9 % sections is more trustworthy than the absolute values.
- `analysis_confidence` was 0.96-0.98 for all leaders in the attached range, 0.58-0.80 for plates and near stall, and below 0.5 just past stall. Ignore numbers beyond about 2 deg past stall.
- For the flight comparison deliverable: the 3-D wing CLmax will be about 0.85-0.9 x the airfoil Clmax, and the measured sink rate will include fuselage, tail and propeller drag.

## 10. XFLR5 instructions for the student

1. **Load the airfoils.** File -> Open -> `dat/s7075.dat` (also `sd7037.dat`, `naca4409.dat`, `ht14.dat`). NACA 4409 can alternatively be made with Foil -> NACA Foils, "4409".
2. **Refine the panelling.** In Foil Design, use Refine Globally with 160-200 panels. Check that the TE is closed (or 0.5 % if you built it blunt).
3. **Run the batch analysis.** Use XFoil Direct Analysis -> Batch Analysis with these settings:
   - **Type 1** (fixed speed), Mach 0, **Ncrit 9**, forced transition XtrTop = XtrBot = 1.0 (free transition).
   - Re list: 20k to 200k in 10k steps (20k, 30k, 40k, 50k, 60k, 70k, 80k, 90k, 100k, 120k, 140k, 160k, 180k, 200k). XFLR5 interpolates in Re across the span; the wing tip and tail at slow speed need 20-40k, and launch or dive speeds need about 200k.
   - alpha from -6 deg to +16 deg in 0.25 deg steps, max iterations 200, "Initialize BL" ticked, sequence direction from 0 outward if the GUI offers it. Expect missing, unconverged points near stall at Re 40k and below; that is normal.
4. **Sensitivity runs.**
   - Repeat with Ncrit 7 as a rough-surface check.
   - Run Ncrit 9 with XtrTop = 0.3 to mimic a turbulator.
5. **Tail polars.** Run HT14 with the same Re list from 20k to 120k, alpha -10 to +10.
6. **Plane analysis.** In the plane analysis, set density = 1.09 kg/m^3 and kinematic viscosity = 1.70e-5 m^2/s (Hyderabad hot day: rho = 1.06).
   - Use Type 2 (fixed lift) for the trimmed operating point, with the viscous option on.
   - Use the VLM2 or 3-D panel method with the inertia and CG defined.
7. **Send back** the XFLR5 polar exports (.txt) for S7075 at Re 40k, 60k, 80k and 100k so the NeuralFoil predictions can be checked: Clmax, Cdmin, Cd at Cl 0.7 and 1.0, and Cm.

## 11. Files

- `airfoil_study.py`: candidate sweep, metrics and scoring. `postprocess.py`: ranking robustness, tables, plots and .dat export. Run them in that order with `/opt/anaconda3/bin/python3`.
- `results/`:
  - `metrics_all.csv`: every airfoil x Re x Ncrit.
  - `wing_ranking.csv`, `ranking_robustness.csv`, `ranking_table.md`.
  - `top_metrics.md`, `polar_tables.csv/.md`, `tail_table.csv`.
  - `polars.json`: full polars.
  - `re_grid.csv`.
- `plots/`:
  - `top6_polars_Re60k.png`, `top6_polars_Re100k.png`: Cl-alpha, Cd-Cl, L/D-alpha, Cl^1.5/Cd-alpha, Cm-alpha and confidence for the 7 leaders.
  - `s7075_Re_sweep.png`, `plates_vs_airfoils_Re60k.png`, `tail_polars_Re40k.png`, `tail_polars_Re60k.png`, `geometries.png`.
- `dat/` (Selig format, name on the first line, 149 points; the flat plate has 159):
  - Wing: `s7075.dat`, `sd7037.dat`, `naca4409.dat`.
  - Tail: `ht14.dat`.
  - Printable TE variants: `*_te05.dat`.
  - `flatplate-3pct.dat` (simple sheet tail).

## 12. Sources

- UIUC Airfoil Coordinates Database (coordinates fetched by AeroSandbox): https://m-selig.ae.illinois.edu/ads/coord_database.html
- P. Sharpe, R. J. Hansman, "NeuralFoil: An Airfoil Aerodynamics Analysis Tool Using Physics-Informed Machine Learning", arXiv:2503.16323 (2025): https://arxiv.org/abs/2503.16323
- Selig & McGranahan, "Wind Tunnel Aerodynamic Tests of Six Airfoils for Use on Small Wind Turbines", AIAA 2004-1188 (E387 benchmark versus NASA LTPT): https://m-selig.web.engr.illinois.edu/pubs/SeligMcGranahan-2004-AIAA-2004-1188.pdf
- Guglielmo & Selig, "Spanwise Variations in Profile Drag for Airfoils at Low Reynolds Numbers", J. Aircraft 1996: https://m-selig.web.engr.illinois.edu/pubs/GuglielmoSelig-1996-JofAC-SpanwiseDragLRN.pdf
- Selig, Guglielmo, Broeren, Giguère, "Experiments on Airfoils at Low Reynolds Numbers", AIAA 96-0062 (laminar bubbles and lift-curve hysteresis): https://m-selig.web.engr.illinois.edu/pubs/SeligGuglielmoBroerenGiguere-1996-AIAA-1996-0062.pdf
- UIUC "Summary of Low-Speed Airfoil Data" Vol. 1 (SD7037 and others): https://m-selig.web.engr.illinois.edu/uiuc_lsat/AirfoilsVolume1.pdf
- Pelletier & Mueller, "Low Reynolds Number Aerodynamics of Low-Aspect-Ratio, Thin/Flat/Cambered-Plate Wings", J. Aircraft 37(5) 2000, doi:10.2514/2.2676: https://scilit.com/publications/2fdbf8dcd7af28a2e0da4bb044ddc298
- Laitone, "Wind tunnel tests of wings at Reynolds numbers below 70 000", Exp. Fluids 1997 (a 5 % circular-arc plate is best below Re 70k), as cited in the Pelletier & Mueller search result.
- All other numbers are from NeuralFoil runs in this folder. The assumed air properties are those given in the project context.
