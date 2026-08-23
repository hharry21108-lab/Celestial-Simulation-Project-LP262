import matplotlib.pyplot as plt
import re

earth_x, earth_y = [], []
moon_x, moon_y = [], []
sun_x, sun_y = [], []

# 正则表达式：自动提取 x=... y=...
pattern = re.compile(r"x\s*=\s*([-+eE0-9\.]+)\s*y\s*=\s*([-+eE0-9\.]+)")

with open("orbit.txt", "r", encoding="utf-8") as f:
    for line in f:
        line = line.strip()

        # Earth
        if line.startswith("Earth"):
            m = pattern.search(line)
            if m:
                earth_x.append(float(m.group(1)))
                earth_y.append(float(m.group(2)))

        # Moon
        elif line.startswith("Moon"):
            m = pattern.search(line)
            if m:
                moon_x.append(float(m.group(1)))
                moon_y.append(float(m.group(2)))

        # Sun
        elif line.startswith("Sun"):
            m = pattern.search(line)
            if m:
                sun_x.append(float(m.group(1)))
                sun_y.append(float(m.group(2)))

# 开始画图
plt.figure(figsize=(10, 10))

if sun_x:
    plt.plot(sun_x, sun_y, label="Sun", color="gold", linewidth=2)
    plt.scatter([sun_x[0]], [sun_y[0]], color="gold", s=80)

if earth_x:
    plt.plot(earth_x, earth_y, label="Earth", color="blue", linewidth=1)
    plt.scatter([earth_x[0]], [earth_y[0]], color="blue", s=50)

if moon_x:
    plt.plot(moon_x, moon_y, label="Moon", color="orange", linewidth=1)
    plt.scatter([moon_x[0]], [moon_y[0]], color="orange", s=50)

plt.xlabel("x (meters)")
plt.ylabel("y (meters)")
plt.title("Earth–Moon–Sun Orbit (Velocity-Verlet Simulation)")
plt.legend()
plt.axis("equal")
plt.grid(True)

plt.show()
