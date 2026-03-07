# visualise_data.py
# Plots simulation results from results/sim_data.csv.

import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("results/sim_data.csv")
t  = df["t"].values

plots = [
    ("Electrical frequency",  "Hz",       [("f_e_cmd_hz", "f_e_cmd")]),
    ("Mechanical speed",      "rad/s",    [("omega_m",    "omega_m")]),
    ("dq currents",           "A",        [("id_true",    "id"), ("iq_true", "iq")]),
    ("Torque",                "N·m",      [("Te",         "Te")]),
    ("Phase currents",        "A",        [("ia","ia"), ("ib","ib"), ("ic","ic")]),
    ("Phase voltages",        "V",        [("va","va"), ("vb","vb"), ("vc","vc")]),
    ("Electrical angle",      "rad",      [("theta_ctrl","theta_ctrl"), ("theta_e_true","theta_e_true")]),
    ("PWM duties",            "duty",     [("duty_a","duty_a"), ("duty_b","duty_b"), ("duty_c","duty_c")]),
]

for title, ylabel, series in plots:
    plt.figure()
    for col, label in series:
        if col in df.columns:
            plt.plot(t, df[col], label=label)
    plt.title(title)
    plt.xlabel("Time (s)")
    plt.ylabel(ylabel)
    plt.legend()
    plt.grid(True)

plt.show()