import matplotlib.pyplot as plt

# 读取你的轨道数据文件
# 文件格式要求：每一行包含 Earth x, Earth y, Moon x, Moon y
# 你可以把你的输出整理成这样的四列格式
earth_x = []
earth_y = []
moon_x = []
moon_y = []

with open("orbit.txt", "r", encoding="utf-8") as f:
    for line in f:
        line = line.strip()
        if line.startswith("Earth:"):
            # Earth: x=123 y=456
            parts = line.replace("Earth:", "").strip().split()
            ex = float(parts[0].split("=")[1])
            ey = float(parts[1].split("=")[1])
            earth_x.append(ex)
            earth_y.append(ey)

        if line.startswith("Moon"):
            # Moon : x=123 y=456
            parts = line.replace("Moon :", "").strip().split()
            mx = float(parts[0].split("=")[1])
            my = float(parts[1].split("=")[1])
            moon_x.append(mx)
            moon_y.append(my)

# 开始画图
plt.figure(figsize=(8, 8))

# 画地球轨迹（绕质心的小圈）
plt.plot(earth_x, earth_y, label="Earth", color="blue", linewidth=1)

# 画月球轨迹（大椭圆）
plt.plot(moon_x, moon_y, label="Moon", color="orange", linewidth=1)

# 画地球初始位置
plt.scatter([earth_x[0]], [earth_y[0]], color="blue", s=50)
plt.scatter([moon_x[0]], [moon_y[0]], color="orange", s=50)

plt.xlabel("x (meters)")
plt.ylabel("y (meters)")
plt.title("Earth–Moon Orbit (Velocity-Verlet Simulation)")
plt.legend()
plt.axis("equal")  # 保持比例正确
plt.grid(True)

plt.show()
