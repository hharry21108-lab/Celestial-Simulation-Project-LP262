import matplotlib.pyplot as plt
import re

# ==========================
# 读取 N 体轨迹
# ==========================

tracks = {}  # name -> {"x": [...], "y": [...]}
pattern = re.compile(r"x\s*=\s*([-+eE0-9\.]+)\s*y\s*=\s*([-+eE0-9\.]+)")

with open("orbit.txt", "r", encoding="utf-8") as f:
    for line in f:
        line = line.strip()
        if "x=" in line and "y=" in line:
            name = line.split(":")[0].strip()
            m = pattern.search(line)
            if not m:
                continue
            x = float(m.group(1))
            y = float(m.group(2))
            if name not in tracks:
                tracks[name] = {"x": [], "y": []}
            tracks[name]["x"].append(x)
            tracks[name]["y"].append(y)

print("天体列表:", list(tracks.keys()))

# ==========================
# 创建图像和坐标轴
# ==========================

fig, ax = plt.subplots(figsize=(8, 8))

# 画轨迹
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
# 固定“全景”坐标范围
# ==========================

all_x = []
all_y = []
for data in tracks.values():
    all_x.extend(data["x"])
    all_y.extend(data["y"])

xmin, xmax = min(all_x), max(all_x)
ymin, ymax = min(all_y), max(all_y)

# 加一点边距
padding = 0.05
dx = (xmax - xmin) * padding
dy = (ymax - ymin) * padding

xmin -= dx
xmax += dx
ymin -= dy
ymax += dy

ax.set_xlim(xmin, xmax)
ax.set_ylim(ymin, ymax)

# 保持比例不变，让图像不被拉伸
ax.set_aspect("equal", adjustable="box")

ax.grid(True)
ax.legend()

# 让坐标轴区域尽量填满窗口
fig.tight_layout()

plt.show()
