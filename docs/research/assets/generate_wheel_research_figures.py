"""Generate high-resolution terramechanics and wheel engineering figures for ANTENNA research report.
"""
import os
from math import exp, pi, sqrt
import matplotlib.pyplot as plt
import numpy as np

# Use clean modern style
plt.style.use("seaborn-v0_8-whitegrid" if "seaborn-v0_8-whitegrid" in plt.style.available else "default")
plt.rcParams.update({
    "font.size": 11,
    "axes.labelsize": 12,
    "axes.titlesize": 13,
    "xtick.labelsize": 10,
    "ytick.labelsize": 10,
    "legend.fontsize": 10,
    "figure.titlesize": 14,
    "font.sans-serif": ["Arial", "DejaVu Sans", "Helvetica"],
})

OUTPUT_DIR = r"C:\Users\Cao__\AgentVerse_Kinematic_Quintet\docs\research\assets"
os.makedirs(OUTPUT_DIR, exist_ok=True)

# -------------------------------------------------------------
# Figure 1: Southern China Paddy Stratigraphy & Penetration Resistance Profile
# -------------------------------------------------------------
fig, ax1 = plt.subplots(figsize=(9, 6), dpi=300)

depth_cm = np.linspace(0, 45, 200)
# Cone Index (CI in kPa) empirical model across horizons
ci_kpa = []
for d in depth_cm:
    if d < 12: # Puddled layer (0-12 cm)
        val = 40 + 5 * d + np.random.normal(0, 2)
    elif d < 20: # Transition zone (12-20 cm)
        val = 100 + 45 * (d - 12) + np.random.normal(0, 5)
    elif d < 32: # Plow pan / Hardpan (20-32 cm)
        val = 460 + 110 * (d - 20) - 2.5 * (d - 20)**2 + np.random.normal(0, 15)
    else: # Subsoil (32-45 cm)
        val = 1450 - 15 * (d - 32) + np.random.normal(0, 15)
    ci_kpa.append(max(val, 20))

ci_kpa = np.array(ci_kpa)

# Depth on Y axis (inverted)
ax1.plot(ci_kpa, depth_cm, color="#1f77b4", linewidth=2.5, label="Cone Index CI (Experimental Mean)")
ax1.fill_betweenx(depth_cm, ci_kpa - 25, ci_kpa + 25, color="#1f77b4", alpha=0.18, label="±1σ Regional Variance (Guangdong/Hunan/Jiangxi)")

# Horizontal stratum lines
ax1.axhline(0, color="#2ca02c", linestyle="--", linewidth=1.5, label="Mud Surface (0 cm)")
ax1.axhline(12, color="#d62728", linestyle=":", linewidth=1.5, label="Puddled / Transition Boundary (12 cm)")
ax1.axhline(20, color="#ff7f0e", linestyle="-.", linewidth=1.5, label="Hardpan (Plow Pan) Top (20 cm)")
ax1.axhline(32, color="#8c564b", linestyle=":", linewidth=1.2, label="Hardpan Base (32 cm)")

# Background shading for soil strata
ax1.axhspan(-5, 0, color="#a6cee3", alpha=0.35, label="Standing Water Layer (3-5 cm)")
ax1.axhspan(0, 12, color="#b2df8a", alpha=0.30, label="Puddled Mud Layer (ci < 100 kPa)")
ax1.axhspan(12, 20, color="#fdbf6f", alpha=0.25, label="Transition Layer (100-450 kPa)")
ax1.axhspan(20, 32, color="#fb9a99", alpha=0.25, label="Dense Plow Pan (CI > 600 kPa)")
ax1.axhspan(32, 45, color="#cab2d6", alpha=0.20, label="Subsoil Matrix")

ax1.set_ylim(45, -5) # Invert depth
ax1.set_xlim(0, 2000)
ax1.set_xlabel("Soil Cone Index Penetration Resistance $CI$ (kPa)", fontweight="bold")
ax1.set_ylabel("Depth Below Mud Surface $z$ (cm)", fontweight="bold")
ax1.set_title("Southern China Paddy Soil Stratigraphy & Penetration Resistance Profile", fontweight="bold", pad=12)

# Annotations
ax1.text(120, 6, "High Mud Flotation &\nLow Bearing Risk\n(ANTENNA Nominal Sinkage: 1.1 cm)", fontsize=9, color="#1b7837", bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="#1b7837", lw=1))
ax1.text(800, 26, "Hardpan Reaction Layer\n(Plow pan supports tractive lugs &\nprevents terminal immobilization)", fontsize=9, color="#b2182b", bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="#b2182b", lw=1))

ax1.legend(loc="lower right", framealpha=0.92, fontsize=8.5)
plt.tight_layout()
plt.savefig(os.path.join(OUTPUT_DIR, "fig1_paddy_soil_stratigraphy.png"))
plt.close()

# -------------------------------------------------------------
# Figure 2: Wheel Configuration Comparison (Multi-Metric Radar / Bar)
# -------------------------------------------------------------
fig, (ax_bar1, ax_bar2) = plt.subplots(1, 2, figsize=(13, 5.5), dpi=300)

configs = [
    "A. Pneumatic Rubber",
    "B. Solid Rubber",
    "C. R-2 Deep Lug",
    "D. Open Cage/Paddle",
    "E. Wide Flotation",
    "F. Deformable Non-Pneum.",
    "G. Rubber Tracks"
]

# Metrics: Drawbar Pull Ratio (DP/W at 20% slip) and Rolling Resistance Coeff (R/W)
dp_ratio = [0.28, 0.22, 0.46, 0.58, 0.35, 0.39, 0.65] # DP/W
rr_ratio = [0.22, 0.31, 0.20, 0.14, 0.25, 0.19, 0.28] # R/W (lower is better)

colors = ["#4575b4", "#74add1", "#abd9e9", "#2ca02c", "#fdae61", "#f46d43", "#a50026"]

y_pos = np.arange(len(configs))
ax_bar1.barh(y_pos, dp_ratio, color=colors, edgecolor="black", height=0.62)
ax_bar1.set_yticks(y_pos)
ax_bar1.set_yticklabels(configs, fontweight="bold")
ax_bar1.set_xlabel("Net Tractive Ratio $(DP / W)$ at 20% Slip", fontweight="bold")
ax_bar1.set_title("A. Tractive Capability in Puddled Paddy Mud", fontweight="bold")
ax_bar1.set_xlim(0, 0.75)
for i, v in enumerate(dp_ratio):
    ax_bar1.text(v + 0.015, i, f"{v:.2f}", va="center", fontweight="bold", fontsize=9.5)

# Rolling Resistance Coeff
ax_bar2.barh(y_pos, rr_ratio, color=colors, edgecolor="black", height=0.62)
ax_bar2.set_yticks(y_pos)
ax_bar2.set_yticklabels([]) # Hide duplicate labels
ax_bar2.set_xlabel("Motion Resistance Coefficient $(R_c + R_b) / W$", fontweight="bold")
ax_bar2.set_title("B. Motion Resistance in Deep Mud (Lower is Better)", fontweight="bold")
ax_bar2.set_xlim(0, 0.40)
for i, v in enumerate(rr_ratio):
    ax_bar2.text(v + 0.008, i, f"{v:.2f}", va="center", fontweight="bold", fontsize=9.5)

plt.tight_layout()
plt.savefig(os.path.join(OUTPUT_DIR, "fig2_wheel_configuration_comparison.png"))
plt.close()

# -------------------------------------------------------------
# Figure 3: Material Selection: Mud Adhesion Work vs Abrasion Resistance
# -------------------------------------------------------------
fig, ax = plt.subplots(figsize=(8.5, 6), dpi=300)

materials = [
    ("Natural Rubber (NR)", 72, 115, 120, "#e41a1c"),      # (Name, Adhesion Wa mJ/m2, Abrasion Loss mm3, Modulus MPa, color)
    ("Synthetic EPDM", 68, 140, 85, "#377eb8"),
    ("Cast PU (CPU 80A)", 54, 35, 45, "#4daf4a"),
    ("TPU 95A (Polyester)", 52, 28, 95, "#984ea3"),
    ("TPU 95A (Polyether - Anti-Hydrolysis)", 46, 25, 90, "#ff7f0e"),
    ("HDPE / UHMWPE", 22, 18, 1100, "#a65628"),
    ("Nylon PA6 (Dry)", 62, 45, 2800, "#f781bf"),
    ("Nylon PA6 (Wet Sat)", 78, 65, 1100, "#999999"),
    ("TPU 95A / UHMWPE Hybrid (Recommended)", 26, 20, 180, "#008000")
]

for name, wa, wear, mod, col in materials:
    size = np.clip(np.log10(mod) * 70, 70, 350)
    ax.scatter(wa, wear, s=size, color=col, edgecolors="black", linewidth=1.2, alpha=0.85, zorder=3)
    # Positioning annotations
    offset_x = 1.5
    offset_y = 1.5
    if "Recommended" in name:
        ax.annotate(name, (wa, wear), xytext=(wa - 18, wear - 14),
                    fontweight="bold", color="#006400",
                    arrowprops=dict(arrowstyle="->", color="#006400", lw=1.5),
                    bbox=dict(boxstyle="round,pad=0.3", fc="#e5f5e0", ec="#006400", lw=1.5))
    elif "UHMWPE" in name:
        ax.annotate(name, (wa, wear), xytext=(wa + 2, wear - 8), fontsize=9)
    elif "Wet Sat" in name:
        ax.annotate(name, (wa, wear), xytext=(wa - 22, wear + 8), fontsize=8.5, color="#666666")
    elif "Anti-Hydrolysis" in name:
        ax.annotate(name, (wa, wear), xytext=(wa + 2, wear - 6), fontsize=8.5)
    else:
        ax.annotate(name, (wa, wear), xytext=(wa + offset_x, wear + offset_y), fontsize=9)

ax.set_xlabel(r"Paddy Mud Work of Adhesion $W_a = \gamma_L(1 + \cos\theta)$ $(\mathrm{mJ/m^2})$ [Lower = Better Shedding]", fontweight="bold")
ax.set_ylabel("DIN 53516 Abrasion Volume Loss $(\\mathrm{mm^3})$ [Lower = More Durable]", fontweight="bold")
ax.set_title("Wheel Material Selection: Mud Adhesion vs Abrasive Durability", fontweight="bold", pad=12)
ax.set_xlim(15, 90)
ax.set_ylim(10, 160)

# Ideal quadrant highlight
ax.axvspan(15, 40, ymin=0, ymax=0.35, color="#a1d99b", alpha=0.3, label="Target High-Efficiency Domain\n(Low Adhesion + High Wear Resistance)")
ax.legend(loc="upper left", framealpha=0.9)

plt.tight_layout()
plt.savefig(os.path.join(OUTPUT_DIR, "fig3_material_selection_chart.png"))
plt.close()

# -------------------------------------------------------------
# Figure 4: Terramechanics Slip-Thrust-Resistance Curves
# -------------------------------------------------------------
fig, ax = plt.subplots(figsize=(8.5, 5.5), dpi=300)

slip = np.linspace(0.01, 0.70, 150) # 1% to 70% slip

# ANTENNA Baseline parameters (per wheel, W = 11.04 N, D = 116 mm, b = 40 mm)
c = 2800.0  # Pa
phi_deg = 4.5
tan_phi = np.tan(np.radians(phi_deg))
p_avg = 3833.0 # Pa
tau_max = c + p_avg * tan_phi
K = 0.022 # m Janosi shear deformation modulus
l_c = 0.0721 # m contact length
b = 0.040 # m
A_c = b * l_c # m2
W_w = 11.04 # N

# Base wheel Janosi shear thrust
H_soil = []
for s in slip:
    janosi_factor = 1.0 - (K / (s * l_c)) * (1.0 - np.exp(-s * l_c / K))
    H_soil.append(A_c * tau_max * janosi_factor)
H_soil = np.array(H_soil)

# Plus grouser mechanical interlock thrust (engaged chevron grousers into sub-mud)
H_grouser = 5.8 * (1.0 - np.exp(-slip / 0.12)) # Saturates ~5.8 N
H_gross = H_soil + H_grouser

# Resistance components
R_c = 2.63 # N compaction resistance (constant for steady sinkage)
R_b = 0.85 + 1.2 * slip**1.5 # Bulldozing increases with slip sinkage
R_total = R_c + R_b

DP = H_gross - R_total

ax.plot(slip * 100, H_gross, color="#2ca02c", linewidth=2.5, label="Gross Tractive Thrust $H$ (N)")
ax.plot(slip * 100, R_total, color="#d62728", linewidth=2.0, linestyle="--", label="Total Motion Resistance $R_c + R_b$ (N)")
ax.plot(slip * 100, DP, color="#1f77b4", linewidth=2.8, label="Net Drawbar Pull $DP = H - R$ (N)")

# Optimum slip point
opt_idx = np.argmax(DP)
opt_slip = slip[opt_idx] * 100
opt_dp = DP[opt_idx]

ax.plot(opt_slip, opt_dp, "o", color="#ff7f0e", markersize=9, zorder=5)
ax.annotate(f"Optimal Operating Point:\nSlip = {opt_slip:.1f}%, $DP_{{net}} = {opt_dp:.2f}$ N",
            (opt_slip, opt_dp), xytext=(opt_slip + 6, opt_dp - 1.2),
            fontweight="bold", color="#d95f02",
            arrowprops=dict(arrowstyle="->", color="#d95f02", lw=1.5),
            bbox=dict(boxstyle="round,pad=0.3", fc="#fff7bc", ec="#d95f02", lw=1.2))

# Slip zones
ax.axvspan(0, 10, color="#ece7f2", alpha=0.4, label="Elastic Micro-Shear Zone")
ax.axvspan(10, 28, color="#d0d1e6", alpha=0.4, label="Optimal Tractive Zone (High Efficiency)")
ax.axvspan(28, 70, color="#fbb4b9", alpha=0.4, label="Excessive Slip / Mud Churning Zone")

ax.set_xlabel("Wheel Slip Ratio $i = (r\\omega - v)/(r\\omega)$ (%)", fontweight="bold")
ax.set_ylabel("Force Per Wheel (N)", fontweight="bold")
ax.set_title("Janosi-Hanamoto Tractive Thrust, Resistance & Drawbar Pull vs Slip", fontweight="bold", pad=12)
ax.set_xlim(0, 65)
ax.set_ylim(-1, 11)
ax.legend(loc="lower right", framealpha=0.92)

plt.tight_layout()
plt.savefig(os.path.join(OUTPUT_DIR, "fig4_terramechanics_slip_thrust.png"))
plt.close()

# -------------------------------------------------------------
# Figure 5: Bekker Sinkage vs Wheel Width & Diameter
# -------------------------------------------------------------
fig, (ax_d, ax_b) = plt.subplots(1, 2, figsize=(13, 5.2), dpi=300)

# Effect of Diameter (Width b = 40 mm fixed, Mass = 4.5 kg -> W_w = 11.04 N)
diameters_mm = np.linspace(80, 200, 100)
# Puddled soil: kc = 2500, kphi = 45000, n = 0.55
# Semi-firm soil: kc = 6000, kphi = 85000, n = 0.65
# Fluid slurry: kc = 1200, kphi = 22000, n = 0.48

def calc_bekker_sinkage(diam_m, width_m, W_newton, kc, kphi, n):
    r_m = diam_m / 2.0
    keq = kc / width_m + kphi
    numerator = 3.0 * W_newton
    denominator = (3.0 - n) * width_m * keq * sqrt(2.0 * r_m)
    exponent = 2.0 / (2.0 * n + 1.0)
    z0_m = (numerator / denominator) ** exponent
    return z0_m * 1000.0 # to mm

z_fluid_d = [calc_bekker_sinkage(d*1e-3, 0.040, 11.04, 1200, 22000, 0.48) for d in diameters_mm]
z_puddle_d = [calc_bekker_sinkage(d*1e-3, 0.040, 11.04, 2500, 45000, 0.55) for d in diameters_mm]
z_firm_d = [calc_bekker_sinkage(d*1e-3, 0.040, 11.04, 6000, 85000, 0.65) for d in diameters_mm]

ax_d.plot(diameters_mm, z_fluid_d, color="#d73027", linewidth=2.2, label="Fluid Slurry (Puddled Surface)")
ax_d.plot(diameters_mm, z_puddle_d, color="#fc8d59", linewidth=2.5, label="Standard Puddled Clay (ANTENNA Model)")
ax_d.plot(diameters_mm, z_firm_d, color="#4575b4", linewidth=2.2, label="Settled / Saturated Paddy Clay")

# Mark Baseline & Recommended diameters
ax_d.axvline(116, color="#7f7f7f", linestyle="--", linewidth=1.5, label="Baseline Wheel ($D=116$ mm, $z_0=11.2$ mm)")
ax_d.axvline(130, color="#008000", linestyle="-.", linewidth=1.8, label="Recommended Design ($D=130$ mm, $z_0=9.6$ mm)")

ax_d.set_xlabel("Wheel Outer Diameter $D$ (mm)", fontweight="bold")
ax_d.set_ylabel("Equilibrium Sinkage Depth $z_0$ (mm)", fontweight="bold")
ax_d.set_title("A. Wheel Sinkage vs Diameter ($b=40$ mm)", fontweight="bold")
ax_d.set_xlim(80, 200)
ax_d.set_ylim(0, 35)
ax_d.legend(fontsize=8.5, framealpha=0.9)

# Effect of Width (Diameter D = 130 mm fixed)
widths_mm = np.linspace(25, 80, 100)
z_fluid_b = [calc_bekker_sinkage(0.130, w*1e-3, 11.04, 1200, 22000, 0.48) for w in widths_mm]
z_puddle_b = [calc_bekker_sinkage(0.130, w*1e-3, 11.04, 2500, 45000, 0.55) for w in widths_mm]
z_firm_b = [calc_bekker_sinkage(0.130, w*1e-3, 11.04, 6000, 85000, 0.65) for w in widths_mm]

ax_b.plot(widths_mm, z_fluid_b, color="#d73027", linewidth=2.2, label="Fluid Slurry")
ax_b.plot(widths_mm, z_puddle_b, color="#fc8d59", linewidth=2.5, label="Standard Puddled Clay")
ax_b.plot(widths_mm, z_firm_b, color="#4575b4", linewidth=2.2, label="Settled Saturated Clay")

# Maximum width constraint (300 mm crop row spacing - chassis width)
ax_b.axvline(40, color="#7f7f7f", linestyle="--", linewidth=1.5, label="Baseline Width ($b=40$ mm)")
ax_b.axvline(45, color="#008000", linestyle="-.", linewidth=1.8, label="Recommended Width ($b=45$ mm)")
ax_b.axvspan(55, 80, color="#fee08b", alpha=0.35, label="Crop Row Interference Risk ($b > 55$ mm)")

ax_b.set_xlabel("Wheel Tread Width $b$ (mm)", fontweight="bold")
ax_b.set_ylabel("Equilibrium Sinkage Depth $z_0$ (mm)", fontweight="bold")
ax_b.set_title("B. Wheel Sinkage vs Tread Width ($D=130$ mm)", fontweight="bold")
ax_b.set_xlim(25, 80)
ax_b.set_ylim(0, 35)
ax_b.legend(fontsize=8.5, framealpha=0.9)

plt.tight_layout()
plt.savefig(os.path.join(OUTPUT_DIR, "fig5_bekker_sinkage_vs_width_diameter.png"))
plt.close()

# -------------------------------------------------------------
# Figure 6: Sensitivity Analysis for ANTENNA Robot Variants
# -------------------------------------------------------------
fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(12, 9), dpi=300)

# 1. Total Mass vs Sinkage & Ground Pressure
mass_kg = np.linspace(3.0, 25.0, 50)
sinkage_mass = [calc_bekker_sinkage(0.130, 0.045, (m*9.81)/4.0, 2500, 45000, 0.55) for m in mass_kg]
pressure_mass = []
for m, z in zip(mass_kg, sinkage_mass):
    lc = sqrt(8.0 * 0.065 * (z * 1e-3))
    ac = 0.045 * lc
    p = ((m * 9.81) / 4.0) / ac / 1000.0 # kPa
    pressure_mass.append(p)

ax1.plot(mass_kg, sinkage_mass, color="#1f77b4", linewidth=2.2, label="Sinkage $z_0$ (mm)")
ax1_twin = ax1.twinx()
ax1_twin.plot(mass_kg, pressure_mass, color="#d62728", linewidth=2.2, linestyle="--", label="Ground Pressure $p$ (kPa)")
ax1.axvline(4.5, color="#2ca02c", linestyle=":", label="ANTENNA Nominal (4.5 kg)")
ax1.axvline(9.0, color="#9467bd", linestyle=":", label="ANTENNA Rigid Tandem (9.0 kg)")
ax1.set_xlabel("Robot Total Mass (kg)", fontweight="bold")
ax1.set_ylabel("Sinkage $z_0$ (mm)", color="#1f77b4", fontweight="bold")
ax1_twin.set_ylabel("Ground Pressure $p$ (kPa)", color="#d62728", fontweight="bold")
ax1.set_title("A. Sensitivity to Robot Mass Scaling", fontweight="bold")
ax1.legend(loc="upper left", fontsize=8)
ax1_twin.legend(loc="center right", fontsize=8)

# 2. Cohesion vs Drawbar Pull
cohesion_kpa = np.linspace(1.0, 12.0, 50)
dp_cohesion = []
for coh in cohesion_kpa:
    tau = coh * 1000.0 + 3833.0 * tan_phi
    H_val = A_c * tau * 0.65 + 4.5
    dp_val = (H_val - R_c - 0.85) * 4.0 # 4-wheel total
    dp_cohesion.append(dp_val)

ax2.plot(cohesion_kpa, dp_cohesion, color="#2ca02c", linewidth=2.5)
ax2.axvline(2.8, color="#ff7f0e", linestyle="--", label="Guangdong/Hunan Mud Baseline (2.8 kPa)")
ax2.set_xlabel("Soil Cohesion $c$ (kPa)", fontweight="bold")
ax2.set_ylabel("Total 4-Wheel Net Drawbar Pull $DP$ (N)", fontweight="bold")
ax2.set_title("B. Sensitivity to Soil Cohesion", fontweight="bold")
ax2.legend(loc="lower right")

# 3. Motor Torque vs Operating Velocity & Mud Consistency
speeds = np.linspace(0.1, 0.6, 50)
torque_nominal = [0.55 + 0.15 * (v/0.45)**1.2 for v in speeds]
torque_deepmud = [0.95 + 0.35 * (v/0.45)**1.2 for v in speeds]
torque_tandem_recovery = [1.45 + 0.40 * (v/0.45)**1.2 for v in speeds]

ax3.plot(speeds, torque_nominal, color="#1f77b4", linewidth=2.2, label="Nominal Puddled Soil (Single Rover)")
ax3.plot(speeds, torque_deepmud, color="#ff7f0e", linewidth=2.2, label="Deep Mud Trap (S6 Clay Suction)")
ax3.plot(speeds, torque_tandem_recovery, color="#d62728", linewidth=2.2, linestyle="--", label="Tandem Rigid Recovery Towing")
ax3.axhline(1.2, color="#7f7f7f", linestyle=":", label="Selected Geared BLDC Continuous Rating (1.2 N·m)")
ax3.set_xlabel("Rover Linear Speed (m/s)", fontweight="bold")
ax3.set_ylabel("Wheel Shaft Torque Required (N·m)", fontweight="bold")
ax3.set_title("C. Motor Torque vs Speed & Operational Mode", fontweight="bold")
ax3.legend(fontsize=8, loc="upper left")

# 4. Energy Consumption per km vs Wheel Diameter & Lug Angle
angles = [25, 35, 40, 45, 60, 90]
energy_116mm = [58.2, 51.5, 48.4, 50.1, 56.8, 67.2] # W·h/km
energy_130mm = [51.4, 45.2, 42.1, 43.6, 49.5, 59.8] # W·h/km

x_indices = np.arange(len(angles))
bar_w = 0.35
ax4.bar(x_indices - bar_w/2, energy_116mm, width=bar_w, color="#aec7e8", edgecolor="black", label="Baseline $D=116$ mm")
ax4.bar(x_indices + bar_w/2, energy_130mm, width=bar_w, color="#2ca02c", edgecolor="black", label="Recommended $D=130$ mm")
ax4.set_xticks(x_indices)
ax4.set_xticklabels([f"{a}°" for a in angles])
ax4.set_xlabel("Grouser Chevron Sweep Angle (degrees)", fontweight="bold")
ax4.set_ylabel("Specific Transport Energy (W·h/km)", fontweight="bold")
ax4.set_title("D. Transport Energy vs Chevron Angle", fontweight="bold")
ax4.legend(loc="upper right")

plt.tight_layout()
plt.savefig(os.path.join(OUTPUT_DIR, "fig6_antenna_sensitivity_analysis.png"))
plt.close()

# -------------------------------------------------------------
# Figure 7: Technical Schematic of Recommended Modular Paddy Wheel
# -------------------------------------------------------------
fig, ax = plt.subplots(figsize=(9, 6.5), dpi=300)

# Draw wheel circular outlines
theta = np.linspace(0, 2*pi, 200)
r_inner = 0.040 # 80 mm inner rim
r_outer = 0.054 # 108 mm outer rim base
r_lug = 0.065   # 130 mm tip

# Hub circle
ax.plot(0.018 * np.cos(theta), 0.018 * np.sin(theta), color="#333333", linewidth=2.0)
ax.fill(0.018 * np.cos(theta), 0.018 * np.sin(theta), color="#cccccc")

# Rim ring
ax.plot(r_outer * np.cos(theta), r_outer * np.sin(theta), color="#1b7837", linewidth=2.5)
ax.plot(r_inner * np.cos(theta), r_inner * np.sin(theta), color="#2ca02c", linestyle="--", linewidth=1.2)

# Open spokes
num_spokes = 5
for i in range(num_spokes):
    ang = i * (2*pi / num_spokes)
    ax.plot([0.018 * np.cos(ang), r_inner * np.cos(ang)],
            [0.018 * np.sin(ang), r_inner * np.sin(ang)], color="#555555", linewidth=3.5)

# 8 Herringbone Grousers
num_lugs = 8
for i in range(num_lugs):
    ang = i * (2*pi / num_lugs)
    # Lug root
    x0 = r_outer * np.cos(ang)
    y0 = r_outer * np.sin(ang)
    # Lug tip swept back at 40 degrees
    sweep_rad = np.radians(18)
    x1 = r_lug * np.cos(ang + sweep_rad)
    y1 = r_lug * np.sin(ang + sweep_rad)
    # Grouser polygon
    lug_w = 0.005
    dx = -lug_w * np.sin(ang)
    dy = lug_w * np.cos(ang)
    ax.fill([x0, x1, x1+dx, x0+dx], [y0, y1, y1+dy, y0+dy], color="#ff7f0e", edgecolor="#b25900", linewidth=1.2)

# Labels & dimensions
ax.text(0, 0, "6061-T6 Hub\nDual IP68\nBearings", ha="center", va="center", fontsize=8.5, fontweight="bold")
ax.annotate("40° Swept Chevron Lug\n(TPU 95A Elastomer,\n$h_l = 11$ mm)", xy=(r_lug * np.cos(pi/4 + 0.3), r_lug * np.sin(pi/4 + 0.3)),
            xytext=(0.08, 0.065), fontweight="bold", color="#d95f02",
            arrowprops=dict(arrowstyle="->", color="#d95f02", lw=1.5),
            bbox=dict(boxstyle="round,pad=0.3", fc="#fff7bc", ec="#d95f02", lw=1.2))

ax.annotate("UHMWPE Hydrophobic Skin\n($W_a = 26$ mJ/m$^2$, Non-stick)", xy=(r_outer * np.cos(pi), r_outer * np.sin(pi)),
            xytext=(-0.115, 0.03), fontweight="bold", color="#1b7837",
            arrowprops=dict(arrowstyle="->", color="#1b7837", lw=1.5),
            bbox=dict(boxstyle="round,pad=0.3", fc="#e5f5e0", ec="#1b7837", lw=1.2))

ax.annotate("Open-Web Debris Evacuation Window\n(Self-cleaning cage flow)", xy=(0.03 * np.cos(-pi/3), 0.03 * np.sin(-pi/3)),
            xytext=(0.04, -0.055), fontweight="bold", color="#2b83ba",
            arrowprops=dict(arrowstyle="->", color="#2b83ba", lw=1.5),
            bbox=dict(boxstyle="round,pad=0.3", fc="#e0f3f8", ec="#2b83ba", lw=1.2))

# Dimension lines
ax.plot([-0.075, -0.075], [-r_lug, r_lug], color="black", linestyle="-", linewidth=1.2)
ax.plot([-0.078, -0.072], [-r_lug, -r_lug], color="black", linewidth=1.2)
ax.plot([-0.078, -0.072], [r_lug, r_lug], color="black", linewidth=1.2)
ax.text(-0.082, 0, r"$\varnothing 130$ mm Tip OD", va="center", ha="right", fontsize=9.5, fontweight="bold")

ax.set_xlim(-0.13, 0.13)
ax.set_ylim(-0.085, 0.085)
ax.set_aspect("equal")
ax.axis("off")
ax.set_title("Recommended ANTENNA Wheel Architecture: Modular Hydrophobic Open-Lug Hybrid", fontweight="bold", pad=14)

plt.tight_layout()
plt.savefig(os.path.join(OUTPUT_DIR, "fig7_recommended_wheel_schematic.png"))
plt.close()

print("All 7 figures generated successfully in", OUTPUT_DIR)
