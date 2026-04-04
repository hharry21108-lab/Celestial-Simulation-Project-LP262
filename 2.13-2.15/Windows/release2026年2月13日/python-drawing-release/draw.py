import matplotlib.pyplot as plt
import re

# ==========================
# 读取 N 体轨迹
# ==========================

tracks = {}
pattern = re.compile(r"x\s*=\s*([-+eE0-9\.]+)\s*y\s*=\s*([-+eE0-9\.]+)")

with open("orbit.txt", "r", encoding="utf-8") as f:
    for line in f:
        line = line.strip()
        if "x=" in line and "y=" in line:
            name = line.split(":")[0].strip()
            m = pattern.search(line)
            if m:
                x = float(m.group(1))
                y = float(m.group(2))
                if name not in tracks:
                    tracks[name] = {"x": [], "y": []}
                tracks[name]["x"].append(x)
                tracks[name]["y"].append(y)

# ==========================
# 创建图像（必须先创建 ax）
# ==========================

fig, ax = plt.subplots(figsize=(10, 10))

# ==========================
# 绘制所有天体轨迹
# ==========================

colors = [
    "tab:blue", "tab:orange", "tab:green", "tab:red",
    "tab:purple", "tab:brown", "tab:pink", "tab:gray",
    "tab:olive", "tab:cyan"
]

for i, (name, data) in enumerate(tracks.items()):
    c = colors[i % len(colors)]
    ax.plot(data["x"], data["y"], label=name, color=c, linewidth=1)
    ax.scatter([data["x"][0]], [data["y"][0]], color=c, s=20)

# ==========================
# 自动全景范围（必须在 ax 创建之后）
# ==========================

all_x = []
all_y = []

for name, data in tracks.items():
    all_x.extend(data["x"])
    all_y.extend(data["y"])

xmin, xmax = min(all_x), max(all_x)
ymin, ymax = min(all_y), max(all_y)

padding = 0.05
dx = (xmax - xmin) * padding
dy = (ymax - ymin) * padding

ax.set_xlim(xmin - dx, xmax + dx)
ax.set_ylim(ymin - dy, ymax + dy)

ax.set_aspect("equal", adjustable="box")
ax.grid(True)
ax.legend()

plt.show()
