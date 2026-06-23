import numpy as np
import matplotlib.pyplot as plt
from scipy import stats

data = np.loadtxt("/code/src/crs/controls/car/pacejka_mpcc/script/data/mpcc_residuals_cm1_0555.csv",
 delimiter=",", skiprows=1)
data = data[data[:, 0] >= 0.5]

names = ["eps_vx", "eps_vy", "eps_omega"]
fig, axes = plt.subplots(2, 3, figsize=(14, 8))

for i, name in enumerate(names):
    col = data[:, 5 + i]

    # Гистограмма + нормальное распределение
    ax = axes[0, i]
    ax.hist(col, bins=100, density=True, alpha=0.7)
    x = np.linspace(col.min(), col.max(), 200)
    ax.plot(x, stats.norm.pdf(x, col.mean(), col.std()), 'r-', label='normal fit')
    q25, q75 = np.percentile(col, [25, 75])
    iqr = q75 - q25
    for k, ls in [(3, '--'), (4, ':')]:
        ax.axvline(q25 - k*iqr, color='orange', ls=ls, label=f'k={k}')
        ax.axvline(q75 + k*iqr, color='orange', ls=ls)
    ax.set_title(name)
    ax.legend(fontsize=7)

    # Q-Q plot — отклонение от нормального
    ax = axes[1, i]
    stats.probplot(col, dist="norm", plot=ax)
    ax.set_title(f"{name} Q-Q plot")

plt.tight_layout()
plt.savefig("eps_distribution.png", dpi=150)
plt.show()