import pygame
import math

# 初始化 Pygame
pygame.init()

# --- 配置参数 ---
WIDTH, HEIGHT = 1000, 800
WIN = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Solar System N-Body Simulation - Voyager Alpha Mission")

# 颜色定义
BLACK = (10, 10, 15)  # 深空蓝黑
WHITE = (255, 255, 255)
YELLOW = (255, 255, 0)
BLUE = (100, 149, 237)
RED = (188, 39, 50)
DARK_GREY = (80, 78, 81)
ORANGE = (255, 165, 0)
GREEN = (0, 255, 0)
CYAN = (0, 255, 255)
MAGENTA = (255, 0, 255)
GOLD = (218, 165, 32)
TEXT_COLOR = (200, 200, 200)

# 字体
FONT_SIZE = 14
FONT = pygame.font.SysFont("Consolas", FONT_SIZE)
TITLE_FONT = pygame.font.SysFont("Consolas", 16, bold=True)

# 物理常数
AU = 149.6e6 * 1000  # 天文单位 (米)
G = 6.67428e-11      # 万有引力常数
SCALE = 200 / AU     # 默认缩放比例 (1AU = 200像素)
TIMESTEP = 3600 * 24 # 默认时间步长 (1天)

class Body:
    def __init__(self, name, x, y, radius, color, mass, is_fixed=False):
        self.name = name
        self.x = x
        self.y = y
        self.radius = radius
        self.color = color
        self.mass = mass
        self.is_fixed = is_fixed # 太阳通常视为固定中心(简化模型)或参与运算

        self.x_vel = 0
        self.y_vel = 0
        
        self.ax = 0 # 当前加速度x
        self.ay = 0 # 当前加速度y

        self.orbit = []
        self.sun = False
        self.distance_to_sun = 0

    def draw(self, win, offset_x, offset_y, scale):
        x = self.x * scale + WIDTH / 2 - offset_x
        y = self.y * scale + HEIGHT / 2 - offset_y

        # 绘制轨迹
        if len(self.orbit) > 2:
            scaled_points = []
            for point in self.orbit:
                px, py = point
                px = px * scale + WIDTH / 2 - offset_x
                py = py * scale + HEIGHT / 2 - offset_y
                scaled_points.append((px, py))
            
            # 性能优化：只绘制在屏幕内的点，或者限制轨迹长度
            if len(scaled_points) > 1:
                pygame.draw.lines(win, self.color, False, scaled_points, 1)

        # 绘制本体
        if -self.radius*2 < x < WIDTH + self.radius*2 and -self.radius*2 < y < HEIGHT + self.radius*2:
            pygame.draw.circle(win, self.color, (int(x), int(y)), max(3, int(self.radius * scale / 1e8 * 5))) # 动态调整显示大小
            
            # 绘制名字标签
            text = FONT.render(self.name, 1, self.color)
            win.blit(text, (x + 10, y - 10))

    def attraction(self, other):
        other_x, other_y = other.x, other.y
        distance_x = other_x - self.x
        distance_y = other_y - self.y
        distance = math.sqrt(distance_x ** 2 + distance_y ** 2)

        if other.sun:
            self.distance_to_sun = distance

        force = G * self.mass * other.mass / distance**2
        theta = math.atan2(distance_y, distance_x)
        force_x = math.cos(theta) * force
        force_y = math.sin(theta) * force
        return force_x, force_y

    def update_position(self, bodies, dt):
        total_fx = total_fy = 0
        for body in bodies:
            if self == body:
                continue
            fx, fy = self.attraction(body)
            total_fx += fx
            total_fy += fy

        # F = ma => a = F/m
        self.ax = total_fx / self.mass
        self.ay = total_fy / self.mass

        self.x_vel += self.ax * dt
        self.y_vel += self.ay * dt

        self.x += self.x_vel * dt
        self.y += self.y_vel * dt
        
        # 记录轨迹 (每隔几次更新记录一次，节省内存)
        # 这里简化为每次都记，实际可优化
        self.orbit.append((self.x, self.y))
        if len(self.orbit) > 500: # 限制轨迹长度
            self.orbit.pop(0)

class Spacecraft(Body):
    def __init__(self, name, x, y, color, mass, fuel):
        super().__init__(name, x, y, 5, color, mass)
        self.fuel = fuel
        self.mode = "STABLE"
        self.thrust_active = False

    def apply_thrust(self, direction_x, direction_y, thrust_amount, dt):
        # 修复：增加燃料检查，防止负燃料
        if self.fuel > 0:
            # 假设推力产生加速度 (简化物理: F=ma, 忽略质量随燃料减少的变化)
            # thrust_amount 单位可以是 N
            acc = thrust_amount / self.mass 
            
            # 归一化方向
            length = math.sqrt(direction_x**2 + direction_y**2)
            if length > 0:
                dir_x = direction_x / length
                dir_y = direction_y / length
                
                self.x_vel += dir_x * acc * dt
                self.y_vel += dir_y * acc * dt
                
                # 消耗燃料 (简化: 每秒消耗 1kg 或其他单位)
                consumption = 0.1 * dt # 假设值
                self.fuel -= consumption
                self.thrust_active = True
                
                if self.fuel < 0:
                    self.fuel = 0
        else:
            self.thrust_active = False
            self.mode = "NO FUEL"

# 辅助类：用于计算虚拟点（如拉格朗日点）
class VirtualPoint:
    def __init__(self, name, parent1, parent2, angle_offset, color):
        self.name = name
        self.p1 = parent1 # e.g. Sun
        self.p2 = parent2 # e.g. Earth
        self.angle_offset = angle_offset # +60度 for L4, -60度 for L5
        self.color = color
        self.x = 0
        self.y = 0
        self.ax = 0
        self.ay = 0
        self.x_vel = 0
        self.y_vel = 0 # 仅用于UI显示，虚拟点没有真实速度物理

    def update(self):
        # 简单的三角几何计算 L4/L5
        # 基于从 p1 到 p2 的向量，旋转 60 度
        dx = self.p2.x - self.p1.x
        dy = self.p2.y - self.p1.y
        dist = math.sqrt(dx*dx + dy*dy)
        angle = math.atan2(dy, dx)
        
        target_angle = angle + math.radians(self.angle_offset)
        
        self.x = self.p1.x + math.cos(target_angle) * dist
        self.y = self.p1.y + math.sin(target_angle) * dist
        
        # 粗略估算速度（用于UI显示），实际上L点速度等于地球角速度
        self.x_vel = self.p2.x_vel 
        self.y_vel = self.p2.y_vel

    def draw(self, win, offset_x, offset_y, scale):
        x = self.x * scale + WIDTH / 2 - offset_x
        y = self.y * scale + HEIGHT / 2 - offset_y
        if -20 < x < WIDTH + 20 and -20 < y < HEIGHT + 20:
            pygame.draw.rect(win, self.color, (int(x)-3, int(y)-3, 6, 6))
            text = FONT.render(self.name, 1, self.color)
            win.blit(text, (x + 8, y - 8))

def format_sci(val):
    """格式化科学计数法显示"""
    return "{:.2e}".format(val)

def main():
    run = True
    clock = pygame.time.Clock()
    
    # --- 创建天体 ---
    sun = Body("Sun", 0, 0, 30, YELLOW, 1.98892 * 10**30)
    sun.sun = True
    
    mercury = Body("Mercury", 0.387 * AU, 0, 8, DARK_GREY, 3.30 * 10**23)
    mercury.y_vel = -47.4 * 1000
    
    venus = Body("Venus", 0.723 * AU, 0, 14, WHITE, 4.8685 * 10**24)
    venus.y_vel = -35.02 * 1000
    
    earth = Body("Earth", -1 * AU, 0, 16, BLUE, 5.9742 * 10**24)
    earth.y_vel = 29.783 * 1000
    
    mars = Body("Mars", -1.524 * AU, 0, 12, RED, 6.39 * 10**23)
    mars.y_vel = 24.077 * 1000

    jupiter = Body("Jupiter", 5.203 * AU, 0, 25, ORANGE, 1.898 * 10**27)
    jupiter.y_vel = 13.07 * 1000

    saturn = Body("Saturn", 9.537 * AU, 0, 22, GOLD, 5.683 * 10**26)
    saturn.y_vel = 9.69 * 1000

    uranus = Body("Uranus", 19.191 * AU, 0, 18, CYAN, 8.681 * 10**25)
    uranus.y_vel = 6.81 * 1000

    neptune = Body("Neptune", 30.068 * AU, 0, 17, BLUE, 1.024 * 10**26)
    neptune.y_vel = 5.43 * 1000
    
    # 飞船 (初始位置设在地球附近或者自定义)
    # 图片显示 Voyager_Alpha 似乎在向外飞
    voyager = Spacecraft("Voyager_Alpha", -1.1 * AU, -0.1 * AU, YELLOW, 1000, 500) # 500kg Fuel
    voyager.x_vel = 0
    voyager.y_vel = 35000 # 初始高速

    bodies = [sun, mercury, venus, earth, mars, jupiter, saturn, uranus, neptune, voyager]

    # 虚拟点
    earth_l4 = VirtualPoint("Earth_L4", sun, earth, 60, GREEN)
    earth_l5 = VirtualPoint("Earth_L5", sun, earth, -60, MAGENTA)
    virtual_points = [earth_l4, earth_l5]

    # --- 视图控制 ---
    scale = 100 / AU # 初始缩放
    offset_x = 0
    offset_y = 0
    
    # 跟踪系统
    follow_index = -1 # -1 表示不跟踪，其他对应 bodies 索引
    # 默认跟踪 Voyager
    follow_index = bodies.index(voyager)

    # 时间控制
    paused = False
    timestep_mult = 1.0 # 时间倍率
    elapsed_time = 0 # 总经过秒数

    while run:
        clock.tick(60)
        
        # --- 物理步进 ---
        if not paused:
            # 可以在这里做多次子步进以提高精度
            current_dt = TIMESTEP * timestep_mult
            
            for body in bodies:
                body.update_position(bodies, current_dt)
            
            for vp in virtual_points:
                vp.update()
                
            elapsed_time += current_dt

        # --- 输入处理 ---
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
            
            # 滚轮缩放
            elif event.type == pygame.MOUSEWHEEL:
                if event.y > 0:
                    scale *= 1.1
                else:
                    scale /= 1.1

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    run = False
                if event.key == pygame.K_SPACE:
                    paused = not paused
                if event.key == pygame.K_TAB:
                    follow_index += 1
                    if follow_index >= len(bodies):
                        follow_index = -1 # 自由视角
                # 时间控制
                if event.key == pygame.K_RIGHT:
                    timestep_mult *= 2.0
                if event.key == pygame.K_LEFT:
                    timestep_mult /= 2.0
                if event.key == pygame.K_r: # 重置视角
                    scale = 100 / AU
                    offset_x = 0
                    offset_y = 0
                    follow_index = -1

        # --- 视角计算 ---
        if follow_index != -1:
            target = bodies[follow_index]
            # 平滑移动相机 (Lerp) 或者直接锁定
            # 这里直接锁定以匹配截图的 "FOLLOWING: X" 效果
            offset_x = target.x * scale
            offset_y = target.y * scale
        else:
            # 键盘控制平移 (自由模式)
            keys = pygame.key.get_pressed()
            move_speed = 10 / scale
            if keys[pygame.K_w] or keys[pygame.K_UP]: offset_y -= move_speed
            if keys[pygame.K_s] or keys[pygame.K_DOWN]: offset_y += move_speed
            if keys[pygame.K_a] or keys[pygame.K_LEFT]: offset_x -= move_speed
            if keys[pygame.K_d] or keys[pygame.K_RIGHT]: offset_x += move_speed

        # --- 渲染 ---
        WIN.fill(BLACK)

        # 1. 绘制网格 (可选，为了匹配截图的 Grid)
        # 简单画几条线表示 AU
        grid_spacing = 5 * AU * scale
        if grid_spacing > 50: # 只在间距足够大时绘制
            # 这里略去复杂的无限网格，为了性能
            pass

        # 2. 绘制所有天体
        for body in bodies:
            body.draw(WIN, offset_x, offset_y, scale)
        
        for vp in virtual_points:
            vp.draw(WIN, offset_x, offset_y, scale)

        # --- UI 覆盖层 ---
        
        # 左上角信息
        days = elapsed_time // (3600 * 24)
        hours = (elapsed_time % (3600 * 24)) // 3600
        
        status_color = YELLOW if paused else GREEN
        status_text = "PAUSED" if paused else "RUNNING"
        
        ui_y = 10
        WIN.blit(FONT.render(f"TIME: {int(days)}d {int(hours)}h", 1, GREEN), (10, ui_y))
        ui_y += 20
        WIN.blit(FONT.render(f"STATUS: {status_text}", 1, status_color), (10, ui_y))
        ui_y += 30
        
        WIN.blit(FONT.render("[ BODIES ]", 1, TEXT_COLOR), (10, ui_y))
        ui_y += 20
        for i, body in enumerate(bodies):
            prefix = "-> " if i == follow_index else "   "
            b_text = FONT.render(f"{prefix}{body.name}", 1, body.color)
            WIN.blit(b_text, (10, ui_y))
            ui_y += 18
            
        # 右侧数据面板 (模仿截图)
        right_margin = WIDTH - 250
        row_h = 0
        for body in bodies + virtual_points:
            # 标题
            name_surf = TITLE_FONT.render(f"[{body.name}]", 1, body.color)
            name_rect = name_surf.get_rect(right=WIDTH-10, top=10 + row_h)
            WIN.blit(name_surf, name_rect)
            row_h += 20
            
            # P (Position)
            p_str = f"P: {format_sci(body.x)}, {format_sci(body.y)}"
            p_surf = FONT.render(p_str, 1, TEXT_COLOR)
            p_rect = p_surf.get_rect(right=WIDTH-10, top=10 + row_h)
            WIN.blit(p_surf, p_rect)
            row_h += 15
            
            # V (Velocity)
            vel = math.sqrt(body.x_vel**2 + body.y_vel**2)
            v_str = f"V: {format_sci(vel)} m/s"
            v_surf = FONT.render(v_str, 1, TEXT_COLOR)
            v_rect = v_surf.get_rect(right=WIDTH-10, top=10 + row_h)
            WIN.blit(v_surf, v_rect)
            row_h += 15
            
            # A (Acceleration)
            acc = math.sqrt(body.ax**2 + body.ay**2)
            a_str = f"A: {format_sci(acc)} m/s2"
            a_surf = FONT.render(a_str, 1, TEXT_COLOR)
            a_rect = a_surf.get_rect(right=WIDTH-10, top=10 + row_h)
            WIN.blit(a_surf, a_rect)
            row_h += 15

            # 如果是飞船，显示燃料
            if hasattr(body, 'fuel'):
                fuel_color = RED if body.fuel < 10 else GREEN
                f_str = f"Fuel: {int(body.fuel)} kg"
                f_surf = FONT.render(f_str, 1, fuel_color)
                f_rect = f_surf.get_rect(right=WIDTH-10, top=10 + row_h)
                WIN.blit(f_surf, f_rect)
                row_h += 15
                
                m_str = f"Mode: {body.mode}"
                m_surf = FONT.render(m_str, 1, YELLOW)
                m_rect = m_surf.get_rect(right=WIDTH-10, top=10 + row_h)
                WIN.blit(m_surf, m_rect)
                row_h += 15

            row_h += 15 # 间距

        # 底部状态栏
        bottom_y = HEIGHT - 20
        # 刻度尺 (Scale)
        au_scale = WIDTH / scale / AU
        scale_text = f"Scale: Screen Width = {au_scale:.1f} AU"
        WIN.blit(FONT.render(scale_text, 1, DARK_GREY), (10, bottom_y))
        
        # 提示
        controls = "[Space] Pause | [Arrows] Speed | [Tab] Follow | [Scroll] Zoom"
        ctrl_surf = FONT.render(controls, 1, DARK_GREY)
        WIN.blit(ctrl_surf, (WIDTH/2 - ctrl_surf.get_width()/2, bottom_y))
        
        if follow_index != -1:
            f_text = f"FOLLOWING: {bodies[follow_index].name} [TAB/ESC]"
            f_surf = FONT.render(f_text, 1, YELLOW)
            WIN.blit(f_surf, (WIDTH - f_surf.get_width() - 10, bottom_y))

        pygame.display.update()

    pygame.quit()

if __name__ == "__main__":
    main()
