import matplotlib.pyplot as plt

# ===== 可调参数 =====
filename = "orbit.txt"   # 你的输出文件
scale   = 1.0 / 1e9      # 坐标缩放（这里把米缩放到“十亿米”）
max_steps_to_load = None # 限制最多读多少个 step，None 表示全部

# name -> {"x": [...], "y": [...]}
tracks = {}

def parse_body_line(line):
    """
    解析类似：
    'Earth: x=0.298243 y=0'
    'Moon : x=3.844e+08 y=10220'
    'Sun  : x=1.496e+11 y=297800'
    'X    : x=... y=...'
    返回 (name, x, y)
    """
    # 按 ':' 分成 [name, ' x=... y=...']
    parts = line.split(":")
    if len(parts) < 2:
        return None

    name = parts[0].strip()  # 'Earth', 'Moon', 'Sun', 'X' 等
    rest = parts[1].strip()  # 'x=... y=...'

    # rest 形如 'x=3.844e+08 y=10220'
    tokens = rest.split()
    # 找到形如 'x=...' 和 'y=...'
    x_val = None
    y_val = None
    for tok in tokens:
        if tok.startswith("x="):
            x_val = float(tok.split("=", 1)[1])
        elif tok.startswith("y="):
            y_val = float(tok.split("=", 1)[1])

    if x_val is None or y_val is None:
        return None

    return name, x_val, y_val


step_count = 0
with open(filename, "r", encoding="utf-8") as f:
    for line in f:
        line = line.strip()
        if not line:
            continue

        # 只关心包含 'x=' 和 'y=' 的行
        if "x=" in line and "y=" in line:
            parsed = parse_body_line(line)
            if parsed is None:
                continue
            name, x, y = parsed

            if name not in tracks:
                tracks[name] = {"x": [], "y": []}
            tracks[name]["x"].append(x * scale)
            tracks[name]["y"].append(y * scale)

        # 统计 step 数（可选）
        if line.startswith("step "):
            step_count += 1
            if max_steps_to_load is not None and step_count > max_steps_to_load:
                break

print(f"读取到的天体：{list(tracks.keys())}")

# ===== 开始画图 =====
plt.figure(figsize=(8, 8))

colors = [
    "tab:blue", "tab:orange", "tab:green", "tab:red",
    "tab:purple", "tab:brown", "tab:pink", "tab:gray",
    "tab:olive", "tab:cyan"
]

for i, (name, data) in enumerate(tracks.items()):
    c = colors[i % len(colors)]
    plt.plot(data["x"], data["y"], label=name, color=c, linewidth=1)
    # 标出初始位置
    if data["x"] and data["y"]:
        plt.scatter([data["x"][0]], [data["y"][0]], color=c, s=20)

plt.xlabel(f"x (scaled by {scale:g})")
plt.ylabel(f"y (scaled by {scale:g})")
plt.title("N-body Orbits from orbit.txt")
plt.legend()
plt.axis("equal")
plt.grid(True)

plt.show()
