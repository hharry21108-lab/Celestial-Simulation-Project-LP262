# 一、 摘要（Abstract）

本文研究最初主要为了探究地日拉格朗日点（Lagrangian point）的稳定性，通过现有的公式和研究，例如基于万有引力场的加速度系统，四阶辛积分法等推演，采用c++23现代编程语言构建，进行探究，实现了L4，L5点理论稳定捕获质量与实验模拟质量的探究与互证。
随着代码的完善， 我们可以达到添加任何天体初始值以达到任何模拟效果。
# 二、引言（Introduction）

1. **已有研究**  
  当研究与计算拉格朗日点时，即为当两个质量大的天体相互作用下，在旋转参考系这一非惯性参考系下，假设有一天体绕另一天体公转，必会产生有离心力，此时处于地日L4与L5点就是分别受到两主要天体万有引力与离心力平衡的点，即太阳引力：  

   $$
   \vec{F}_{\mathrm{sun}} = G \frac{M_{\mathrm{sun}} m}{r_{\mathrm{tosun}}^2}
   $$
   
  地球引力

   $$
   \vec{F}_{\mathrm{earth}} = G \frac{M_{\mathrm{earth}} m}{r_{\mathrm{toearth}}^2}
   $$
   
  以及从公转旋转中心指该点向外的惯性力
 
   $$
   \vec{F}_{\mathrm{c}} = m \omega^2 \vec{r}
   $$

3. **不同**  
   然而，这或许在短期的计算上回相对简单，但是若是长期的推演，多天体共同作用下，该点的稳定性是否依旧呢？若是仅通过计算来获得，这是不可能的。  
   - 非惯性参考系弊端  
     当涉及到长期模拟，采用非惯性参考系下进行离心力的考究，由开普勒定律（Kepler's law）可知，两天体运转是共轴转动，若使得以一质量大的天体为中心，则另一天体运行在以该天体为焦点的椭圆轨道上。  
     由于天体运转遵循机械能守恒定律（law of conservation of mechanical energy），当一天体在椭圆轨道上运行时，由于二者相对高度在发生变化，因此动能会与势能相互转化，并不为恒定，其速度会有变化，不为匀速，这就导致惯性力的计算存在波动于困难，并不长久。  
     因此采取一个假想的绝对静止平面作为惯性系可以大幅简化推演难度，这也需要并非纯手动推演。  
   - 多体（三体及以上）运动的不可预测性  
     在当前研究下，仅有部分轨道规定好且无任何额外扰动的三体运动可以被预测，当考虑更多因素时，或者增加天体个数时，这就变得无法预测，这也便是混沌理论（Chaos theory）  

   因此计算只能作为短期的暂时的状态推断，可用于验证或被验证，真实应借助以工具解决。
# 三、方法与结果（Methodology & Results）

1. **思路**  
  由于长期的手动推算在多天体运动中不可直接推测，但我们可以通过计算出某一时刻的运动状态（位置坐标、速度），再凭借工具基于该初始值进行推衍。

2. **<第一步>计算已知初始值**  
  - 已知值  
     $M_{\mathrm{sun}} \approx 1.989 \times 10^{30}$ kg  
     $M_{\mathrm{earth}} \approx 5.972 \times 10^{24}$ kg  
     $R \approx 1$ AU  
     $G \approx 6.6743 \times 10^{-11}$ m³ kg⁻¹ s⁻²  
  - 太阳、地球初始值  
    太阳：假使太阳初始坐标为原点，所有天体均相对于这一“绝对”静止平面运动（此时不需考虑惯性力等），那么太阳初始坐标为(0,0)，初始速度为0；  
    地球：由于地日距离为那么令地球坐标为约1AU，那么地球坐标为(-AU,0)，太阳引力提供其绕日运动的向心力
       
$$
\frac{M_{\mathrm{sun}} M_{\mathrm{earth}}}{R^2} = \frac{M_{\mathrm{earth}} v^2}{R}
$$

  可以求出地球的线速度
     
$$
v = \sqrt{\frac{G M_{\mathrm{sun}}}{R}} = 29782 \, \mathrm{m/s}
$$
     
  那么就可以得到其速度的正交分解坐标(0,-29782)  
  - 地日拉格朗日点L4，L5点初始值  
    由于在惯性系中，地球和太阳都在动，很难找静止平衡点，因此我们以地球和太阳的静止为参考系，建立以地日质心为中心的旋转参考系，即一个非惯性参考系，此时L4点天体必受一个在沿质心向外的离心力，以及两天体的万有引力。由于L4点处于平衡状态，那么其所受的合引力必与离心力等大反向。即  

$$
\vec{F}_{\mathrm{sun}} + \vec{F}_{\mathrm{earth}} + \vec{F}_{\mathrm{centrifugal}} = 0
$$
      
  若使得太阳位置记作S，地球位置记作E，L4点记作P，展开可得  

$$
\frac{M_{\mathrm{sun}}}{R_{\mathrm{TOsun}}^3} \vec{PS} + \frac{M_{\mathrm{earth}}}{R_{\mathrm{TOearth}}^3} \vec{PE} + \frac{M_{\mathrm{sun}} + M_{\mathrm{earth}}}{R^3} \vec{CP} = 0
$$
 
  再结合质点（C点）的定义  

$$
M_{\mathrm{sun}} \vec{CS} + M_{\mathrm{earth}} \vec{CE} = 0
$$
     
  通过观察我们可以发现方程一解：  

$$
R_{\mathrm{TOearth}} = R_{\mathrm{TOsun}} = R
$$

  此时将展开式与质心定义结合并带入该解，可以发现刚好消去所有值，等式成立。  
  因此我们可以得知，地日L4点与地球、太阳恰好构成一个正三角形，边长为1AU，因此L4点总是先于地球1/3π rad；同时由于在地日的另一侧，同有一点同时受地日引力和沿质心向外的离心力，但方向恰与L4点相反，这便是地日拉格朗日点L5点，其总是后于地球1/3π rad。  

$$
L4 \mathrm{点位置坐标} (-\mathrm{AU} \cos \frac{\pi}{3}, -\mathrm{AU} \sin \frac{\pi}{3}); \mathrm{速度坐标} (25786.0, -14890.0)
$$
      
$$
L5 \mathrm{点位置坐标} (-\mathrm{AU} \cos \frac{\pi}{3}, \mathrm{AU} \sin \frac{\pi}{3}); \mathrm{速度坐标} (-25786.0, -14890.0)
$$
     
  至于位于两点的天体质量，先采用小质量天体进行探究（后期可以根据设定航天器轨道，探究两点捕获能力），此处令二者质量均为10kg。
3. **<第二步>实现坐标推衍**
	- 整体构建思路
		现在我们已经得知了准确的天体初始坐标，计划通过万有引力场下各天体在系统内部的相互作用自动更新位置坐标，采用c++23构建。
   - 基本常量与内容定义  
     首先需对部分基本物理常量定义，方便简化后续编程  

     ```cpp
     const double G = 6.67430e-11;
     const double AU = 1.496e11;
     ```  

     然后便是存储每个天体的基本数值{名称，质量，坐标，速度坐标，加速度坐标}  

     ```cpp
     struct Body {
         string name;
         double mass;
         double x, y, vx, vy, ax, ay;
         SDL_Color color;
     };
     ```
由于有n个天体同时存在，那么每个天体都会受到其余n-1个天体的万有引力作用，那么对于第i个天体，其所受和引力大小为

$$
F_i = \sum_{\substack{j=1 \\ j \ne i}}^{n}
G \frac{m_i m_j}{\lvert \vec{r}_j - \vec{r}_i \rvert^2}
$$

其中求和的每一项方向都不同，各自由第i个天体指向第j个天体($\sum_{\substack{j=1 \\ j \ne i}}^{n}$)
因此第i个天体合加速度大小为

$$
a_i = \sum_{\substack{j=1 \\ j \ne i}}^{n}
G \frac{m_j}{\lvert \vec{r}_j - \vec{r}_i \rvert^2}
$$

方向亦然，此时已知加速度大小，与加速度方向，我们可以再分解加速度，得出加速度的正交分解分量，即为此时刻的瞬时加速度坐标，即为

$$
a_{x,i}
= \sum_{\substack{j=1 \\ j \ne i}}^{n}
G m_j \frac{x_j - x_i}{\lvert \vec{r}_j - \vec{r}_i \rvert^3}
$$

$$
a_{y,i}
= \sum_{\substack{j=1 \\ j \ne i}}^{n}
G m_j \frac{y_j - y_i}{\lvert \vec{r}_j - \vec{r}_i \rvert^3}
$$

注意我们必须先求出万有引力大小再分解，如果直接将万有引力公式中的 ${\lvert \vec{r}_j - \vec{r}_i \rvert^2}$ 改为 ${\lvert \vec{x}_j - \vec{x}_i \rvert^2}$ 以及 ${\lvert \vec{y}_j - \vec{y}_i \rvert^2}$ ，再分别在x，y方向合求出 $a_ix$ 与 $a_iy$ ，此时 ${a_ix}^2+{a_iy}^2$ 与 ${a_i}^2$ 并不完全等价，所以只能先求出第j个天体对于第i个天体的引力，再分解为x，y两方向，再求合，不可直接在x，y两个方向计算。
我们将计算结果写至代码中，便有了

```cpp
			void compute_acceleration(int id, int n, double& ax, double& ay) {
				ax = 0.0; ay = 0.0;
				const double soft = 1e7;
				for (int j = 0; j < n; ++j) {
					if (j == id) continue;
					double dx = shared_x[j] - shared_x[id];
					double dy = shared_y[j] - shared_y[id];
					double r2 = dx * dx + dy * dy + soft;
					double r = sqrt(r2);
					double f_newton = G * b[j].mass / (r2 * r);        
					ax += f_newton * dx; ay += f_newton * dy;		
				}		
			}
```

	
- 位移的更新
我们已知了各个行星初始坐标以及此刻的加速度，为了在指定步长进行更新位移和速度，我们需要采用积分法进行。由于宇宙是一个哈密顿系统（Hamiltonian System），即存在一定的守恒性，因此当我最初采用欧拉积分法时，由于能量的漂移，地球飞离了太阳系，因此随后我决定采用幸积分法，其可以达到长期能量守恒（无能量漂移），也就是说这对于研究轨道的混沌性和稳定性至关重要。
二阶幸积分算法（二阶跳蛙法 (Leapfrog / Velocity Verlet)）通过先计算半步速度的位移，在通过重新调用加速度公式，得出新的全步速度，补偿半步速度的偏移，进而在下一步计算式补偿位移的偏移（若是位移偏离过大，后半步加速度计算得出值也会偏大，从而拉回行星速度，补偿偏移）

$$
前半步速度：v_{n+\tfrac12} = v_n + a(x_n)\,\frac{\Delta t}{2}
$$

$$
位移：x_{n+1} = x_n + v_{n+\tfrac12}\,\Delta t
$$

$$
后半步加速度：a_{n+1} = a(x_{n+1})
$$

$$
后半步/全步速度：v_{n+1} = v_{n+\tfrac12} + a_{n+1}\,\frac{\Delta t}{2}
$$

将其写入代码中，即为：

```cpp
			auto worker = [&](int id) {
				for (int step = 0; step < steps; ++step) {            
					compute_acceleration(id, steps, b[id].ax, b[id].ay); // 计算第一次加速度
					//计算半步速度/位移
					b[id].vx += 0.5 * b[id].ax * dt;
					b[id].vy += 0.5 * b[id].ay * dt;
					b[id].x += b[id].vx * dt;
					b[id].y += b[id].vy * dt;
					//计算全步速度
					compute_acceleration(id, steps, b[id].ax, b[id].ay); // 计算第二次加速度
					b[id].vx += 0.5 * b[id].ax * dt;
					b[id].vy += 0.5 * b[id].ay * dt;
					old_x[id] = b[id].x;
					old_y[id] = b[id].y;
				}
			};
```

通过两部分代码的相互调用，我们已经可以实现了模拟（附图-地日月系统-python+坐标绘图）
![天体模拟LP262-1774758428441](天体模拟LP262-1774758428441.webp)
但是，为了更高的精度，继续采用高阶幸积分，这里为四阶幸积分法

```cpp
			void physics_worker_pooled(int thread_id, int num_threads, int n, barrier<>& sync) {
			    while (!quit) {
			        for (int sub = 0; sub < 20; ++sub) {
			            for (int s = 0; s < 3; ++s) {
			                for (int i = start; i < end; ++i) {
			                    b[i].x += b[i].vx * c_y[s] * dt;
			                    b[i].y += b[i].vy * c_y[s] * dt;
			                    shared_x[i] = b[i].x; shared_y[i] = b[i].y;
			                }
			                for (int i = start; i < end; ++i) {
			                    compute_acceleration(i, n, b[i].ax, b[i].ay);
			                    b[i].vx += b[i].ax * d_y[s] * dt;
			                    b[i].vy += b[i].ay * d_y[s] * dt;
			                }
			            }
			            for (int i = start; i < end; ++i) {
			                b[i].x += b[i].vx * c_y[3] * dt;
			                b[i].y += b[i].vy * c_y[3] * dt;
			                shared_x[i] = b[i].x; shared_y[i] = b[i].y;
			            }
			        }
			    }
			}
```

此刻轨道模拟已经一定程度上的精准，在增加实时监听代码后（见后一部分），可以进行模拟。
- 补充-相对论修正
基本原理

$$
\vec{a} = \frac{GM}{r^3} \left[ \left( 1 + \frac{4GM}{c^2 r} - \frac{v^2}{c^2} \right)\vec{r} \;+\; \frac{4(\vec{r}\cdot\vec{v})}{c^2}\,\vec{v} \right]
$$

代码转译

```cpp
			if (b[j].mass > 1e29) {
			    double v2 = b[id].vx * b[id].vx + b[id].vy * b[id].vy;
			    double r_v = (dx * b[id].vx + dy * b[id].vy);
			    double prec = (4.0 * G * b[j].mass / r - v2) / C2;
			        ax += f_newton * (dx * (1.0 + prec) + (4.0 * r_v * b[id].vx) / C2);
			        ay += f_newton * (dy * (1.0 + prec) + (4.0 * r_v * b[id].vy) / C2);
			} 
			else {
			        ax += f_newton * dx; ay += f_newton * dy;
			}
```

- 代码、功能优化
监听改进：当首次验证通过后，需要大量的坐标推演验证理论，因此运行一段时间后再导出坐标进行绘制的方法过于浪费时间，不可取，因而写实时监听至c++代码，实时显示坐标、回放、暂停等，并在此基础上改进为四阶辛积分（改进辛积分算法部分见上部分）。
速率改进：始终采用多线程构建，利用c++新特性；星球颜色后期改为手动输入自定义，简化了自动生成所导致的代码复杂性，一定程度上提高了代码运行速率。
- 最终推演，验证理论（带入之前坐标）
这时我们可以增添更多的天体（太阳系八大行星&哈雷彗星），可见L4，L5点小质量行星轨道并不受影响，始终先于/后于地球60°（附图）
![天体模拟LP262-1774886221762](天体模拟LP262-1774886221762.webp)
![天体模拟LP262-1774886308451](天体模拟LP262-1774886308451.webp)

4. **结果**  
   由上文可知，地日拉格朗日点L4、L5点的确具有一定的稳定性。

# 四、结论（Conclusion）
代码目前已相对完善， 我们可以达到添加任何天体初始值以达到任何模拟效果。
至于进一步探究，有以下几方面可以进行：  
1. **实验方面**  
   可以增加行星id判断，并在时间达到一定值后摄入加速度，模拟人工卫星，探究两点捕获能力以及卫星运行的稳定性； 积分法可采用动态步长提高速率和精度；相对论修正部分可采用3PN等修正替代现有1PN后牛顿修正，提高精度。
2. **构造方面**  
   可以增设更多条件与因素，例如太阳风；也可以增设监测与探究角度，例如：势能梯度图等。
