# visualise_data.py
# Plots simulation results from results/sim_data.csv.

import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("scripts/results/sim_data.csv")
t  = df["t"].values

plots = [
    ("Mechanical speed",  "RPM",   [("rpm_ref",  "RPM ref",    "--"),
                                    ("rpm",       "RPM actual",  "-")]),

    ("d-axis current",    "A",     [("id_ref",   "id ref",     "--"),
                                    ("id_true",  "id actual",   "-")]),

    ("q-axis current",    "A",     [("iq_ref",   "iq ref",     "--"),
                                    ("iq_true",  "iq actual",   "-")]),

    ("Torque",            "N·m",   [("Te",       "Te",          "-")]),

    ("Phase currents",    "A",     [("ia",       "ia",          "-"),
                                    ("ib",       "ib",          "-"),
                                    ("ic",       "ic",          "-")]),

    ("Phase voltages",    "V",     [("va",       "va",          "-"),
                                    ("vb",       "vb",          "-"),
                                    ("vc",       "vc",          "-")]),

    ("Electrical angle",  "rad",   [("theta_e_true", "theta_e_true", "-")]),

    ("PWM duties",        "duty",  [("duty_a",   "duty_a",      "-"),
                                    ("duty_b",   "duty_b",      "-"),
                                    ("duty_c",   "duty_c",      "-")]),
]

for title, ylabel, series in plots:
    fig, ax = plt.subplots()
    plotted = False
    for col, label, ls in series:
        if col in df.columns:
            # Draw reference traces thinner and slightly transparent so actuals stand out
            lw    = 1.2 if ls == "--" else 1.8
            alpha = 0.7 if ls == "--" else 1.0
            ax.plot(t, df[col], linestyle=ls, linewidth=lw, alpha=alpha, label=label)
            plotted = True
    if not plotted:
        plt.close(fig)
        continue
    ax.set_title(title)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel(ylabel)
    ax.legend()
    ax.grid(True)
    fig.tight_layout()

plt.show()