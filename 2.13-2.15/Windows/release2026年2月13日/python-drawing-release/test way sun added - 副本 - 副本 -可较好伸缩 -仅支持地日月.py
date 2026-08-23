import matplotlib.pyplot as plt
import re
class ZoomPan:
    def __init__(self):
        self.press = None
        self.cur_xlim = None
        self.cur_ylim = None
        self.x0 = None
        self.y0 = None

    def zoom(self, event):
        ax = event.inaxes
        if ax is None:
            return

        # 缩放因子
        base_scale = 1.2
        if event.button == 'up':
            scale_factor = 1 / base_scale
        elif event.button == 'down':
            scale_factor = base_scale
        else:
            return

        xlim = ax.get_xlim()
        ylim = ax.get_ylim()

        xdata = event.xdata
        ydata = event.ydata

        new_width = (xlim[1] - xlim[0]) * scale_factor
        new_height = (ylim[1] - ylim[0]) * scale_factor

        ax.set_xlim([xdata - new_width/2, xdata + new_width/2])
        ax.set_ylim([ydata - new_height/2, ydata + new_height/2])
        ax.figure.canvas.draw_idle()

    def pan_press(self, event):
        if event.inaxes is None:
            return
        ax = event.inaxes
        self.cur_xlim = ax.get_xlim()
        self.cur_ylim = ax.get_ylim()
        self.x0 = event.xdata
        self.y0 = event.ydata
        self.press = True

    def pan_release(self, event):
        self.press = None
        event.inaxes.figure.canvas.draw_idle()

    def pan_move(self, event):
        if self.press is None or event.inaxes is None:
            return

        dx = event.xdata - self.x0
        dy = event.ydata - self.y0

        ax = event.inaxes
        ax.set_xlim(self.cur_xlim[0] - dx, self.cur_xlim[1] - dx)
        ax.set_ylim(self.cur_ylim[0] - dy, self.cur_ylim[1] - dy)
        ax.figure.canvas.draw_idle()

earth_x, earth_y = [], []
moon_x, moon_y = [], []
sun_x, sun_y = [], []

pattern = re.compile(r"x\s*=\s*([-+eE0-9\.]+)\s*y\s*=\s*([-+eE0-9\.]+)")

with open("orbit.txt", "r", encoding="utf-8") as f:
    for line in f:
        line = line.strip()

        if line.startswith("Earth"):
            m = pattern.search(line)
            if m:
                earth_x.append(float(m.group(1)))
                earth_y.append(float(m.group(2)))

        elif line.startswith("Moon"):
            m = pattern.search(line)
            if m:
                moon_x.append(float(m.group(1)))
                moon_y.append(float(m.group(2)))

        elif line.startswith("Sun"):
            m = pattern.search(line)
            if m:
                sun_x.append(float(m.group(1)))
                sun_y.append(float(m.group(2)))

# -------------------------
# 交互式缩放和平移支持
# -------------------------

class ZoomPan:
    def __init__(self):
        self.press = None

    def zoom(self, event):
        ax = event.inaxes
        if ax is None:
            return

        scale_factor = 1.2
        if event.button == 'up':      # 滚轮向上：放大
            scale = 1 / scale_factor
        elif event.button == 'down':  # 滚轮向下：缩小
            scale = scale_factor
        else:
            return

        xlim = ax.get_xlim()
        ylim = ax.get_ylim()

        xdata = event.xdata
        ydata = event.ydata

        new_width = (xlim[1] - xlim[0]) * scale
        new_height = (ylim[1] - ylim[0]) * scale

        ax.set_xlim([xdata - new_width/2, xdata + new_width/2])
        ax.set_ylim([ydata - new_height/2, ydata + new_height/2])
        ax.figure.canvas.draw_idle()

    def pan_press(self, event):
        if event.inaxes:
            self.press = (event.xdata, event.ydata, event.inaxes.get_xlim(), event.inaxes.get_ylim())

    def pan_release(self, event):
        self.press = None

    def pan_move(self, event):
        if self.press is None or event.inaxes is None:
            return

        xpress, ypress, xlim, ylim = self.press
        dx = xpress - event.xdata
        dy = ypress - event.ydata

        event.inaxes.set_xlim(xlim[0] + dx, xlim[1] + dx)
        event.inaxes.set_ylim(ylim[0] + dy, ylim[1] + dy)
        event.inaxes.figure.canvas.draw_idle()


fig, ax = plt.subplots(figsize=(10, 10))

# 画轨迹
if sun_x:
    ax.plot(sun_x, sun_y, label="Sun", color="gold", linewidth=2)
if earth_x:
    ax.plot(earth_x, earth_y, label="Earth", color="blue", linewidth=1)
if moon_x:
    ax.plot(moon_x, moon_y, label="Moon", color="orange", linewidth=1)

ax.set_aspect("equal")
ax.grid(True)
ax.legend()


zp = ZoomPan()
fig.canvas.mpl_connect('scroll_event', zp.zoom)
fig.canvas.mpl_connect('button_press_event', zp.pan_press)
fig.canvas.mpl_connect('button_release_event', zp.pan_release)
fig.canvas.mpl_connect('motion_notify_event', zp.pan_move)

plt.show()
