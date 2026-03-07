# visualise_data.py
# Plots simulation results from results/sim_data.csv.

import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("results/sim_data.csv")
t  = df["t"].values

# Helper: convert mechanical speed ref (rad/s) to electrical frequency (Hz)
pole_pairs = 5
if "omega_m_ref" in df.columns:
    df["f_e_ref_hz"] = df["omega_m_ref"] * pole_pairs / (2 * 3.14159265)

plots = [
    ("Mechanical speed",      "rad/s", [("omega_m_ref", "ω_m ref",   "--"),
                                        ("omega_m",     "ω_m actual", "-")]),

    ("d-axis current",        "A",     [("id_ref",      "id ref",    "--"),
                                        ("id_true",     "id actual",  "-")]),

    ("q-axis current",        "A",     [("iq_ref",      "iq ref",    "--"),
                                        ("iq_true",     "iq actual",  "-")]),

    ("Torque",                "N·m",   [("Te",          "Te",         "-")]),

    ("Phase currents",        "A",     [("ia",          "ia",         "-"),
                                        ("ib",          "ib",         "-"),
                                        ("ic",          "ic",         "-")]),

    ("Phase voltages",        "V",     [("va",          "va",         "-"),
                                        ("vb",          "vb",         "-"),
                                        ("vc",          "vc",         "-")]),

    ("Electrical angle",      "rad",   [("theta_ctrl",    "theta_ctrl",   "--"),
                                        ("theta_e_true",  "theta_e_true",  "-")]),

    ("PWM duties",            "duty",  [("duty_a",      "duty_a",     "-"),
                                        ("duty_b",      "duty_b",     "-"),
                                        ("duty_c",      "duty_c",     "-")]),
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