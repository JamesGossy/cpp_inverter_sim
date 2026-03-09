# visualise_data.py
# Plots simulation results from scripts/results/sim_data.csv.

import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("scripts/results/sim_data.csv")
t  = df["t"].values

# ── Mechanical speed ──────────────────────────────────────────────────────
fig, ax = plt.subplots()
ax.plot(t, df["rpm_ref"], linestyle="--", linewidth=1.2, alpha=0.7, label="ref")
ax.plot(t, df["rpm"],     linestyle="-",  linewidth=1.8,             label="actual")
ax.set_title("Mechanical speed")
ax.set_xlabel("Time (s)")
ax.set_ylabel("RPM")
ax.legend()
ax.grid(True)
fig.tight_layout()

# ── d-axis current (ref vs actual, side by side) ──────────────────────────
fig, (ax1, ax2) = plt.subplots(1, 2, sharey=True)
ax1.plot(t, df["id_ref"],  linewidth=1.8)
ax1.set_title("id ref")
ax1.set_xlabel("Time (s)")
ax1.set_ylabel("A")
ax1.grid(True)
ax2.plot(t, df["id_true"], linewidth=1.8)
ax2.set_title("id actual")
ax2.set_xlabel("Time (s)")
ax2.grid(True)
fig.suptitle("d-axis current")
fig.tight_layout()

# ── q-axis current (ref vs actual, side by side) ──────────────────────────
fig, (ax1, ax2) = plt.subplots(1, 2, sharey=True)
ax1.plot(t, df["iq_ref"],  linewidth=1.8)
ax1.set_title("iq ref")
ax1.set_xlabel("Time (s)")
ax1.set_ylabel("A")
ax1.grid(True)
ax2.plot(t, df["iq_true"], linewidth=1.8)
ax2.set_title("iq actual")
ax2.set_xlabel("Time (s)")
ax2.grid(True)
fig.suptitle("q-axis current")
fig.tight_layout()

# ── Torque ────────────────────────────────────────────────────────────────
fig, ax = plt.subplots()
ax.plot(t, df["Te"], linewidth=1.8)
ax.set_title("Torque")
ax.set_xlabel("Time (s)")
ax.set_ylabel("N·m")
ax.grid(True)
fig.tight_layout()

# ── Phase currents ────────────────────────────────────────────────────────
fig, ax = plt.subplots()
ax.plot(t, df["ia"], linewidth=1.8, label="ia")
ax.plot(t, df["ib"], linewidth=1.8, label="ib")
ax.plot(t, df["ic"], linewidth=1.8, label="ic")
ax.set_title("Phase currents")
ax.set_xlabel("Time (s)")
ax.set_ylabel("A")
ax.legend()
ax.grid(True)
fig.tight_layout()

# ── Phase voltages ────────────────────────────────────────────────────────
fig, ax = plt.subplots()
ax.plot(t, df["va"], linewidth=1.8, label="va")
ax.plot(t, df["vb"], linewidth=1.8, label="vb")
ax.plot(t, df["vc"], linewidth=1.8, label="vc")
ax.set_title("Phase voltages")
ax.set_xlabel("Time (s)")
ax.set_ylabel("V")
ax.legend()
ax.grid(True)
fig.tight_layout()

# ── Electrical angle ──────────────────────────────────────────────────────
fig, ax = plt.subplots()
ax.plot(t, df["theta_e"], linewidth=1.8)
ax.set_title("Electrical angle")
ax.set_xlabel("Time (s)")
ax.set_ylabel("rad")
ax.grid(True)
fig.tight_layout()

# ── PWM duties ────────────────────────────────────────────────────────────
fig, ax = plt.subplots()
ax.plot(t, df["duty_a"], linewidth=1.8, label="a")
ax.plot(t, df["duty_b"], linewidth=1.8, label="b")
ax.plot(t, df["duty_c"], linewidth=1.8, label="c")
ax.set_title("PWM duties")
ax.set_xlabel("Time (s)")
ax.set_ylabel("duty")
ax.legend()
ax.grid(True)
fig.tight_layout()

plt.show()