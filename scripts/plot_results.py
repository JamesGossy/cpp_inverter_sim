import pandas as pd
import matplotlib.pyplot as plt
import math
from pathlib import Path

def main():
    # Path to project root 
    project_root = Path(__file__).resolve().parent.parent
    results_path = project_root / "results" / "results.csv"

    print(f"Loading: {results_path}")

    df = pd.read_csv(results_path)
    df["rpm"] = df["omega_m"] * 60.0 / (2.0 * math.pi)

    plt.figure()
    plt.plot(df["t"], df["ia"], label="i_a")
    plt.plot(df["t"], df["ib"], label="i_b")
    plt.plot(df["t"], df["ic"], label="i_c")
    plt.xlabel("t [s]")
    plt.ylabel("Phase current [A]")
    plt.legend()
    plt.grid(True)

    plt.figure()
    plt.plot(df["t"], df["id"], label="i_d")
    plt.plot(df["t"], df["iq"], label="i_q")
    plt.plot(df["t"], df["id_ref"], "--", label="i_d_ref")
    plt.plot(df["t"], df["iq_ref"], "--", label="i_q_ref")
    plt.xlabel("t [s]")
    plt.ylabel("dq current [A]")
    plt.legend()
    plt.grid(True)

    plt.figure()
    plt.plot(df["t"], df["torque_e"])
    plt.xlabel("t [s]")
    plt.ylabel("Electromagnetic torque [N*m]")
    plt.grid(True)

    plt.figure()
    plt.plot(df["t"], df["rpm"])
    plt.xlabel("t [s]")
    plt.ylabel("Speed [rpm]")
    plt.grid(True)

    plt.figure()
    plt.plot(df["t"], df["va"], label="v_a")
    plt.plot(df["t"], df["vb"], label="v_b")
    plt.plot(df["t"], df["vc"], label="v_c")
    plt.xlabel("t [s]")
    plt.ylabel("Applied phase voltage [V]")
    plt.legend()
    plt.grid(True)

    plt.show()

if __name__ == "__main__":
    main()