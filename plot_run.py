# RunData.csv から 加速度・速度・ジャークの時系列グラフを作る
# 使い方:  python plot_run.py
import csv
import matplotlib
import matplotlib.pyplot as plt

# 日本語フォント（Windows）。無ければ英語ラベルに変えてください。
matplotlib.rcParams["font.family"] = ["Yu Gothic", "MS Gothic", "sans-serif"]
matplotlib.rcParams["axes.unicode_minus"] = False

t, a, v, j, timing = [], [], [], [], []
with open("RunData.csv", encoding="utf-8-sig") as f:
    reader = csv.DictReader(f)
    for row in reader:
        t.append(float(row["simTime_s"]))
        a.append(float(row["aCam_mps2"]))
        v.append(float(row["speed_mps"]))
        j.append(float(row["jerk_mps3"]))
        timing.append(int(row["timing"]))

# 計測（スタート〜ゴール）区間の範囲を薄く塗る
run_start = next((t[i] for i in range(len(timing)) if timing[i] == 1), None)
run_end = next((t[i] for i in range(len(timing) - 1, -1, -1) if timing[i] == 1), None)

fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

axs[0].plot(t, a, color="tab:blue", lw=1)
axs[0].axhline(1.0, color="red", ls="--", lw=1, label="上限 1.0 m/s²")
axs[0].set_ylabel("水平合成加速度 [m/s²]")
axs[0].set_ylim(0, 1.1)
axs[0].legend(loc="upper right")
axs[0].set_title("カメラ頂部の運動データ（時系列）")

axs[1].plot(t, v, color="tab:green", lw=1)
axs[1].set_ylabel("速度 [m/s]")

axs[2].plot(t, j, color="tab:orange", lw=1)
axs[2].set_ylabel("ジャーク [m/s³]")
axs[2].set_xlabel("時間 [s]")

# 走破区間を全グラフに薄く表示
if run_start is not None and run_end is not None:
    for ax in axs:
        ax.axvspan(run_start, run_end, color="gray", alpha=0.12)
        ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig("RunData_plot.png", dpi=150)
print("保存しました: RunData_plot.png")
