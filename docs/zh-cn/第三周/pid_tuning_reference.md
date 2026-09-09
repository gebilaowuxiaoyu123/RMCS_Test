# 任务一 · PID 各参数意义分析（三场景）

> 三个场景：**舵轮底盘的舵向电机（外环/内环）**、**控制摩擦轮转速**、**自瞄时云台控制**
> 代码在 `rmcs_ws/src/`，配置在 `rmcs_ws/src/rmcs_bringup/config/`。

---


## 一、参考资料（本地源码为准，RMCS 官方 Wiki 作框架背景）

### A. 本地仓库（可直接引用行号，最可靠）

| 资料 | 支撑哪一点 |
|---|---|
| `rmcs_core/src/controller/pid/pid_calculator.hpp` | PID 算法本体：差分微分、条件积分（积分分离）、限幅 |
| `rmcs_core/src/controller/pid/matrix_pid_calculator.hpp` | 向量版，舵轮 4 电机一次算 |
| `rmcs_core/src/controller/pid/pid_controller.cpp` / `error_pid_controller.cpp` | 两种封装：`err=sp-pv` vs 直接吃误差 |
| `rmcs_core/src/hardware/device/dji_motor.hpp:98-127` | 扭矩常数 / 最大扭矩（算"增益强弱"的关键） |
| `rmcs_bringup/config/{omni-infantry,deformable-infantry-omni,sentry,steering-hero-little-six-friction,flight}.yaml` | 各车真实参数 |
| `docs/zh-cn/firing_mechanism_chain.md` §5 | 仓库自带调参对照表（"到位留残差→外环 ki"等） |
| `README.md` | 已完成的真机验证结论（任务二/三） |

### B. RMCS 官方 Wiki（框架背景，无 PID 整定专页）

| 页面 | 能引用什么 |
|---|---|
| [Home](https://github.com/Alliance-Algorithm/RMCS/wiki) | 框架定位："用软件工程的思维重塑嵌入式开发" |
| [Component Development](https://github.com/Alliance-Algorithm/RMCS/wiki/Component-Development) | `update()` 1kHz 单线程；`InputInterface/OutputInterface` 数据交换；参数经 yaml 的 `component_name: ros__parameters:` 注入 |

> ⚠️ RMCS Wiki 只覆盖**框架用法**（组件如何注册/接线/读参数），没有 PID 整定专页；PID 参数与整定结论均以第一节 A 的**本地仓库源码**为准。

---

## 二、总览：三个场景 × 五项要求

| | ① 舵轮舵向电机 | ② 摩擦轮转速 | ③ 自瞄云台 |
|---|---|---|---|
| **控制需求** | 角度精准+快+抗反扭矩 | 转速稳+打弹快恢复+启停平滑 | 静态精度达火控容差+快跟随+无超调 |
| **期望效果** | 稳态角误差≈0，转向不摆 | 转速贴住工作点，每发弹速一致 | 角误差 < 0.07/0.04 rad |
| **参数配合** | 角度外环 **P** + 速度内环 **P** | **PD**（步兵）/ **P**（英雄） | yaw: **P(+前馈)**；pitch: **P+I+D+重力前馈** |
| **理由** | 输出是速度指令被内环闭环；稳态静止无持续负载 | D 补打弹瞬态；I 会因打弹饱和致弹速不一致 | yaw 无重力怕噪声；pitch 有重力需阻尼 |
| **依据** | 见第一节 A：`steering_wheel_controller.cpp`、`sentry.yaml` | 见第一节 A：`friction_wheel_controller.cpp`、`omni-infantry.yaml` | 见第一节 A：`deformable_infantry_gimbal_controller.cpp`、`sentry.yaml` |

---

## 三、场景①：舵轮底盘的舵向电机（外环 / 内环）

### 3.1 先分清内外环

```mermaid
flowchart LR
    A["目标转向角"] --> B["外环：角度PID<br/>steering_angle_pid_"]
    B --> C["目标角速度"] --> D["内环：速度PID<br/>steering_velocity_pid_"]
    D --> E["扭矩"] --> F["GM6020 内部电流环"]
    F --> G["电机"]
```

- **内环 = 速度环**（快、贴近电机）：`steering_velocity_pid_`
- **外环 = 角度环**（慢、离目标远）：`steering_angle_pid_`

代码在 `steering_wheel_controller.cpp:405-414`，两个环是**嵌套调用**：

```cpp
Eigen::Vector4d steering_torques = steering_velocity_pid_.update(
    steering_control_velocities                    // 前馈：运动学算出的期望角速度
    + steering_angle_pid_.update(                  // 外环：角度误差 → 目标角速度
        (steering_control_angles - steering_status.angle)
            .unaryExpr([](double diff) { ... 折到 ±π/2 ... }))   // 就近转向（可反装轮）
    - steering_status.velocity);                   // 内环反馈：实测角速度
```

**补充说明。** 内环直接面对电机、管"速度"：输入角速度误差、输出扭矩，带宽高、响应快，负责把实际转速尽快贴住指令，并顶掉摩擦、打滑等扰动。外环离被控量（转角）近、管"角度"：输入转向角误差、输出目标角速度，自己不直接驱动电机，只给内环下达任务。之所以拆成两级，是因为既要"转得准"又要"转得稳"——外环保证方向对，内环保证跟得上，两级靠目标角速度串起来，这就是串级（cascade）控制。

### 3.2 控制需求

| 需求 | 具体含义 | 不满足的后果 |
|---|---|---|
| 静态转角精度 | 四个舵向角必须与运动学解算一致 | 平移方向歪、四轮互相"打架"、打滑 |
| 快速、低超调 | 底盘变向要跟手 | 转向慢了迟钝；超调了来回摆 |
| 抗反扭矩 | 底盘运动/打滑时轮子被带转 | 角度漂移，越走越歪 |
| 不打抖 | 4 个转向电机同时抖 = 整车抖 | 底盘高频抖动、轮胎磨损 |
| 单轮力矩足够 | 原地转向需克服轮胎侧向摩擦 | 转不到位 |

### 3.3 期望效果（量化）

- 稳态转向角误差 → 0（因为它是运动学解算的直接输入）
- 转向阶跃后一次到位、无持续振荡
- 前馈提供的角速度使**匀速转向时无滞后**

### 3.4 参数配合（yaml 原文）

哨兵 `rmcs_bringup/config/sentry.yaml`：

```yaml
steering_wheel_controller:
  ros__parameters:
    # ── 内环：速度环（本题重点）──
    steering_velocity_kp: 0.15
    steering_velocity_ki: 0.0
    steering_velocity_kd: 0.0

    # ── 外环：角度环（本题重点）──
    steering_angle_kp: 30.0
    steering_angle_ki: 0.0
    steering_angle_kd: 0.0
    # （其余物理参数 mess/惯量/半径、底盘级与轮子驱动 PID 非本题重点，略）
```

英雄 `steering-hero-little-six-friction.yaml` 只给物理参数（`mess`/`moment_of_inertia`/半径等，非本题重点），**PID 写死在 `hero_steering_wheel_controller.cpp:42-43`**：

```cpp
, steering_velocity_pid_(0.15, 0.0, 0.0)   // 与哨兵 yaml 完全一致
, steering_angle_pid_(30.0, 0.0, 0.0)      // 与哨兵 yaml 完全一致
, wheel_velocity_pid_(0.6, 0.0, 0.0)       // 哨兵是 0.5
```

> 💡 **这个不一致本身就是个结论**：两车舵向参数被验证可共用（0.15/30），只有轮子驱动环因减速比不同（英雄 `2232/169≈13.2`）而略调。

### 3.5 理由

**① 为什么外环 ki = 0？**

P 控制器的固有特性是"必须有误差才能产生输出"，即稳态留静差。但舵轮外环的输出是**目标角速度**，它由内环闭环执行——稳态时只要轮子停了、角度到位了，就没有持续误差可言。加 I 反而会在**内环已有动态**的基础上产生双环积分叠加 → 低频振荡。前馈 `steering_control_velocities` 已经补掉了大部分动态，更没必要用 I 去"试错"。

**② 为什么内环 ki = 0？**

内环是速度环。舵轮的稳态工况是**静止（速度=0）**，"稳态速度静差"对最终角度没有意义（角度是速度的积分，只要最终速度为 0）。I 项在转向过程中会持续累积，反而带来过冲。

**③ 为什么 kp 差 200 倍（30 vs 0.15）却不能说外环"增益更高"？**

量纲不同：

| 环 | 输入 | 输出 | 饱和点 |
|---|---|---|---|
| 外环 | rad（角度误差） | rad/s | `30 × 0.03 rad ≈ 0.9 rad/s` |
| 内环 | rad/s（速度误差） | N·m | `0.15 × 14.8 rad/s ≈ 2.22 N·m` = GM6020 满扭矩 |

GM6020 最大扭矩来自 `dji_motor.hpp:100-104`：`torque_constant = 0.741`、`current_max = 3.0` → `max_torque = 0.741 × 3 = 2.223 N·m`。所以 **0.15 其实很激进**（约 15 rad/s 误差就打满），而 30 对应 0.03 rad≈1.7° 就打满 0.9 rad/s，也合理。

**④ 为什么 kd 可以给 0？**

D 的作用是"阻尼"，代价是放大噪声。舵轮：惯量小、目标角由运动学解算**连续平滑**（无阶跃）、测速来自编码器差分**噪声偏大**。收益 < 风险 → 关掉。若要加，**加在内环**（`steering_velocity_kd`）比外环安全：外环 kd 会把目标角跳变放大成巨大的角速度指令。

**⑤ 注意 dt 效应**

`pid_calculator.hpp` 里微分是**差分**，没有除 dt：

```cpp
if (!std::isnan(last_err_))
    control += kd * (err - last_err_);
```

而 `sentry.yaml` 第一行 `update_rate: 1000.0` → dt = 1ms。所以 `kd` 的等效物理增益是 `kd × 1000`。**这就是为什么全仓库 kd 都很小（0.001~0.3）**——不能拿连续域 PID 的 kd 去套。

### 3.6 小结

外环（角度）纯 P 管"转得准"、内环（速度）纯 P 管"转得稳"，前馈承担大部分角速度需求，因此内外环都不用 I/D；kp 的"强"要看饱和点而非数值（30 → 0.03 rad 打满，0.15 → 15 rad/s 打满 GM6020 扭矩）。

---

## 四、场景②：控制摩擦轮转速

### 4.1 链路

```mermaid
flowchart LR
    FW["FrictionWheelController"] -- "/gimbal/left_friction/control_velocity" --> P["PidController<br/>err = setpoint - measurement"]
    P -- "control_torque" --> M["DjiMotor(M3508)"]
    M -. "velocity" .-> P
    M -. "velocity" .-> FW
```

- 上游 `friction_wheel_controller.cpp` **不管 PID**：软启停（`friction_soft_start_stop_step_ = (1/1000)/soft_start_stop_time`）、堵转检测（速度 < 50% 指令持续 200 周期）、发弹检测（主轮"速度下降积分 < -14"）
- PID 层 `pid_controller.cpp`：通用组件
- 硬件层 `omni_infantry.cpp:77-82`：M3508，`reduction_ratio(1.)` 直驱

**补充说明。** 摩擦轮没有角度外环、只有速度环，因为目标只是"稳在某转速"。上游 `FrictionWheelController` 实为带门槛的指令源：软启停斜坡防阶跃冲击、堵转检测（转速长期低于指令一半报 `friction_jammed`）、发弹检测（转速骤降报 `bullet_fired`）。真正闭环的是下方通用 `PidController`。电机为 M3508 直驱，转速即电机轴 rad/s，直接决定 kp/kd 的量级（见 4.5）。

### 4.2 控制需求

| 需求 | 含义 | 后果 |
|---|---|---|
| 稳态转速准且稳 | 转速 → 弹丸初速 → 弹道 → 自瞄能否命中 | 弹速散布、打不准 |
| 打弹后快速恢复 | 弹丸"啃"进摩擦轮瞬间掉速 | 连发时第二发弹速偏低 |
| 启停平滑 | 660 rad/s 直接给会电流冲击、打滑 | 烧电机、轮胎打滑 |
| 不因噪声抖 | 高转速下编码器量化噪声被微分放大 | 扭矩毛刺、发热 |

### 4.3 期望效果（量化）

- 稳态转速贴住工作转速：步兵 **660**、变形步兵 **580**、无人机 **620**、哨兵 **575** rad/s
- 打弹掉速后在几十 ms 内回到工作点
- 扭矩输出无高频抖动

### 4.4 参数配合（yaml 原文）

**步兵 17mm** `omni-infantry.yaml`（PD）：

```yaml
# friction_wheel_controller 段：2 轮，工作转速 660 rad/s、软启停 1s（见 4.3）
left_friction_velocity_pid_controller:   # 右轮同参数，接线 topic 见 4.1
  ros__parameters:
    kp: 0.003436926
    ki: 0.00                # ★ 无积分
    kd: 0.009373434         # ★ 微分比比例还大（打弹瞬态补偿）
```

**英雄 42mm** `steering-hero-little-six-friction.yaml`（纯 P，6 轮双档）：

```yaml
# friction_wheel_controller 段：6 轮双档 390/480（profile_0/1，见对比表）
first_front_friction_velocity_pid_controller:   # 其余 5 轮同参数
  ros__parameters:
    kp: 0.006233371
    ki: 0.00
    kd: 0.000001    # ★ ≈关闭（6 轮分担冲击、测速噪声敏感）
```

**对比表**（这是"参数配合因工况而异"的最强证据）：

| 车 | 轮数 | 工作转速 | kp | ki | kd |
|---|---|---|---|---|---|
| 步兵 17mm | 2 | 660 | 0.003436926 | 0 | **0.009373434** |
| 变形步兵 | 2 | 580 | 0.003436926 | 0 | **0.009373434** |
| 无人机 | 2 | 620 | 0.003436926 | 0 | **0.009373434** |
| 哨兵 | 2 | 575 | 0.003436926 | 0 | **0.009373434** |
| 英雄 42mm | 6 | 390/480 | 0.006233371 | 0 | **0.000001** |

### 4.5 理由

**① 为什么 ki = 0？（这是本题最关键的判断）**

摩擦轮空转**没有持续负载**，稳态静差只来自轴承摩擦，量很小（标定弹速时可补）。而加 I 的代价极大：**打弹瞬间掉速会让积分迅速累积**，弹丸打完后积分释放 → 转速过冲 → **下一发弹速不一致**。对摩擦轮来说"**每发一致**"比"绝对准"更重要。

参考前面 3.5 的结论：打弹瞬间掉速会让积分迅速累积，弹丸打完后积分释放 → 转速过冲 → **下一发弹速不一致**。代码里虽然有 `integral_split_min/max` 条件积分保护，但 yaml **没配这两个参数** → 默认 `±inf` → 条件永真 → 保护失效：

```cpp
// pid_calculator.hpp
double integral_split_min = -inf, integral_split_max = inf;   // 默认无穷 → 永远积分
```

所以工程上干脆 ki=0，而不是指望积分分离。

**② 为什么 kp 这么小（0.0034）？**

M3508 直驱时最大扭矩只有 `0.3 × 187/3591 × 20 = 0.3124 N·m`（`dji_motor.hpp:110-114`）。要饱和只需误差：

$$e_{sat} = \frac{0.3124}{0.003436926} \approx 91\ \text{rad/s}$$

工作转速 660 rad/s，**启动误差 660 → 立刻满扭矩饱和**。这正是需要 `friction_soft_start_stop_time: 1.0` 软启停的原因；稳态时误差只剩几 rad/s，输出几 mN·m 做精细调节。**kp 小是量纲问题，不代表增益弱**。

**③ 为什么步兵要 kd、英雄不要？（核心差异）**

D 的物理意义是"预测误差趋势、提供阻尼"。步兵工况：
- 只有 **2 个摩擦轮**，单轮承担的弹丸冲击大
- 转速 **660 rad/s**，掉速的**导数极陡**
- kd 项正好在"掉速那一帧"给出大扭矩做**预判式补偿**

估算：`kd × Δv = 0.009373434 × 33 ≈ 0.31 N·m` → 一帧就接近满扭矩，恢复极快。

英雄工况：
- **6 个摩擦轮**分担冲击，单轮掉速小
- 转速只有 390/480 rad/s
- 多轮并行时单轮测速噪声更容易被微分放大（这正是 D 的代价）
- → `kd = 0.000001`，等于关闭

**同一机构、不同工况 → 参数配合完全不同**，这就是"PID 三个参数不一定全上"的最有力例证。

**④ 拨弹盘（顺带对比）**：`omni-infantry.yaml` 里 `bullet_feeder_velocity_pid_controller` 是 `kp: 1.583, ki: 0, kd: 0` **纯 P**。因为拨弹盘输出轴转速低（`2π/8 × 20 = 15.7 rad/s`）、有堵转状态机（`bullet_feeder_controller_17mm.cpp` 的 `update_jam_detection()`），速度环只要"能推动"，加 I/D 反而干扰堵转判定。

### 4.6 小结

摩擦轮空转无持续负载 → ki=0，保证"每发弹速一致"；步兵 2 轮高转速用 D 补打弹瞬态，英雄 6 轮分担冲击、噪声敏感所以几乎不用 D；kp 数值小只是量纲问题（M3508 直驱 0.312 N·m，91 rad/s 误差即饱和），启动靠软启停斜坡。

---

## 五、场景③：自瞄时云台控制

### 5.1 链路（两套写法，结构相同）

**写法 A · 外置 PID 组件**（`omni-infantry.yaml`）：

```mermaid
flowchart TB
    S["SimpleGimbalController"] -- "/gimbal/{yaw,pitch}/control_angle_error" --> E["ErrorPidController<br/>yaw/pitch 角度外环"]
    E -- "/gimbal/{yaw,pitch}/control_velocity" --> P["PidController<br/>yaw/pitch 速度内环"]
    P -- "/gimbal/{yaw,pitch}/control_torque" --> M["GM6020"]
```

**写法 B · PID 内置**（`deformable_infantry_gimbal_controller.cpp:80-104`），并带**重力前馈**：

```cpp
const auto yaw_velocity_ref = yaw_angle_pid_.update(angle_error.yaw_angle_error);
*output_.yaw_control_torque =
    yaw_velocity_pid_.update(yaw_velocity_ref - *input_.yaw_velocity_imu);

const auto pitch_gravity_ff =
    pitch_gravity_ff_gain_ * std::sin(*input_.pitch_angle - pitch_gravity_ff_phase_);   // L220
*output_.pitch_control_torque =
    pitch_velocity_pid_.update(pitch_velocity_ref - *input_.pitch_velocity_imu)
    + pitch_gravity_ff;
```

哨兵还多一层**目标角速度前馈**（`eccentric_dual_yaw.cpp:23-60` `YawRateFeedforward`）：对自瞄目标方位角差分+低通，得到目标角速度直接加到外环输出，**补偿斜坡跟踪滞后**；切换目标板时用 `jump_threshold` 清零防假突变。

**补充说明。** 云台与舵轮是同一骨架：角度外环→速度内环→电流环。写法 A 拆成两个独立 PID 组件，可各自单独调参；写法 B 内聚在一个类里，话题接口一致。pitch 比 yaw 多一项重力前馈 `gain × sin(pitch - phase)`：重力力矩是随俯仰角变化的已知模型，用前馈提前补偿比靠积分慢慢"顶"更干净，且前馈不走反馈、不会引起振荡（详见 5.5）。

### 5.2 控制需求

| 需求 | 含义 | 后果 |
|---|---|---|
| 静态精度达火控容差 | 瞄准点必须落在装甲板内 | 打飞 |
| 无超调、无振荡 | 超调=打偏，振荡=散布 | 弹道散 |
| 快跟随 | 目标横移/跑动 | 追不上、开火窗口丢失 |
| 抗扰 | 开火后坐力、底盘运动、弹链冲击 | 准星跳 |
| 不放大视觉噪声 | 视觉检测有抖动/跳变 | 扭矩毛刺 |

### 5.3 期望效果（有硬指标）

火控的判据在 `auto_aim_component.fire_control` 段（各车一致）：

```yaml
    fire_control:
      bullet_speed: 22.5
      yaw_tolerance: 0.07      # ★ 4.0° 云台稳态角误差上限
      pitch_tolerance: 0.04    # ★ 2.3°
```

`fire_control.cpp:402` 就是拿 `offset <= config.yaw_tolerance` 来判定 `should_shoot` 的。**所以"期望效果"是可以量化的：稳态角误差 < 0.07 rad / 0.04 rad。**

### 5.4 参数配合（yaml 原文）

**步兵 17mm** `omni-infantry.yaml`（yaw 纯 P、pitch P+D）：

```yaml
# yaw 外环：角度误差 → 目标角速度
# （接线 measurement/setpoint/control 均为固定话题，省略，见 5.1）
yaw_angle_pid_controller:
  ros__parameters:
    output_min: -10.0      # 限目标角速度
    output_max: 10.0
    kp: 15.0
    ki: 0.0
    kd: 0.0

# yaw 内环：角速度误差 → 扭矩
yaw_velocity_pid_controller:
  ros__parameters:
    kp: 4.4
    ki: 0.00011      # ★ 极小
    kd: 0.0011       # ★ 极小

# pitch 外环：角度误差 → 目标角速度
pitch_angle_pid_controller:
  ros__parameters:
    kp: 20.0
    ki: 0.0
    kd: 0.1          # ★ pitch 外环给了 D
```

**分析。** yaw 外环带 `output ±10` 限幅，防止角速度指令打到饱和；yaw 内环 ki/kd 极小（0.00011/0.0011），只为消匀速残差、压超调，不放大陀螺噪声；pitch 外环加 `kd=0.1` 防过冲。读法：yaw 偏"跟得上"，pitch 偏"定得住"。

**变形步兵** `deformable-infantry-omni.yaml`（pitch 带 I+D+重力前馈）：

```yaml
gimbal_controller:
  ros__parameters:
    # yaw：无重力 → 纯 P
    yaw_angle_kp: 10.0
    yaw_angle_ki: 0.0
    yaw_angle_kd: 0.0
    yaw_velocity_kp: 10.0
    yaw_velocity_ki: 0.0
    yaw_velocity_kd: 0.0

    # pitch：有重力/限位 → P + 小 I + 小 D
    pitch_angle_kp: 35.0
    pitch_angle_ki: 0.02
    pitch_angle_kd: 0.3
    pitch_velocity_kp: 2.0
    pitch_velocity_ki: 0.0
    pitch_velocity_kd: 0.0

    # 重力前馈（代替大 I 顶重力）+ 走"内环速度环 + 前馈"
    pitch_gravity_ff_gain: 4.302
    pitch_gravity_ff_phase: 0.589
    pitch_torque_control: true
    # （upper/lower_limit 机械限位角，非本题重点，略）
```

**分析。** yaw 纯 P（无重力）；pitch 外环 kp 最大（35）+ 小 I/D 保精度与阻尼，而内环 kp 仅 2.0——重力已被 `4.302×sin(pitch−0.589)` 前馈补偿，内环无需高带宽顶重力。`pitch_torque_control:true` 即此模式开关。

**哨兵** `sentry.yaml`（双 yaw + 角速度前馈）：

```yaml
gimbal_controller:
  ros__parameters:
    # 双 yaw：top 管上段、bottom 管下段（均偏置小惯量，参数可共用一套思路）
    top_yaw_angle_kp: 30.0
    top_yaw_angle_ki: 0.008
    top_yaw_angle_kd: 0.005
    top_yaw_velocity_kp: 2.160
    top_yaw_velocity_ki: 0.0
    top_yaw_velocity_kd: 0.0

    bottom_yaw_angle_kp: 15.0
    bottom_yaw_angle_ki: 0.01
    bottom_yaw_angle_kd: 0.0
    bottom_yaw_velocity_kp: 2.75
    bottom_yaw_velocity_ki: 0.00125
    bottom_yaw_velocity_kd: 0.0

    pitch_angle_kp: 35.0
    pitch_angle_ki: 0.01
    pitch_angle_kd: 0.0
    pitch_velocity_kp: 2.5
    pitch_velocity_ki: 0.01
    pitch_velocity_kd: 0.0

    # 积分限幅：有 ki 才配（★ 三场景里唯一配的），其余环同模式 ±2400/±150
    top_yaw_velocity_integral_min: -2400.0
    top_yaw_velocity_integral_max: 2400.0

    # yaw 目标角速度前馈（补偿斜坡跟踪滞后）
    top_yaw_velocity_ff_gain: 1.0
    top_yaw_ff_cutoff_hz: 10.0
```

**分析。** top/bottom 惯量不同：top 惯量小 angle kp=30、bottom 惯量大 kp=15。外环保留少量 ki（0.008/0.01）是为长时间锁目标消静态残差，故配 ±2400 限幅防饱和；`top_yaw_velocity_ff` 前馈则补跟踪横移目标的滞后。

**英雄** `steering-hero-little-six-friction.yaml`（外置 pitch 双环）：

```yaml
# pitch 外环：角度误差 → 目标角速度
pitch_angle_pid_controller:
  ros__parameters:
    kp: 18.6
    ki: 0.0
    kd: 0.0

# pitch 内环：角速度误差 → 扭矩（接线固定话题，省略）
pitch_velocity_pid_controller:
  ros__parameters:
    kp: 16.05
    ki: 0.00014     # ★ 内环极小 I
    kd: 0.0
    integral_min: -2500.0
    integral_max: 2500.0
```

**分析。** pitch 外环纯 P（18.6）只管出目标角速度；内环 kp=16.05 + 极小 ki（0.00014）+ ±2500 限幅，把 IMU 角速度追上去。内环 kp 远高于步兵（16 vs 2~4.4），因 42mm 重载，参数自成一套，不可与 17mm 直接比。

### 5.5 理由

**① 为什么必须串级？**

云台惯量大、有摩擦。单环角度 PID 要同时管"到位"和"抗扰"，增益很难折中。串级把任务分开：**内环负责把实际角速度追到指令值（快、抗扰），外环负责决定该用多大角速度（慢、精准）**——内环带宽高、外环时间常数长，各自匹配自己的被控对象，总响应更好。

**② yaw 和 pitch 为什么参数不同？**

| | yaw | pitch |
|---|---|---|
| 重力负载 | 无 | **有** |
| 行程 | 大/可无限转 | 小、有机械限位 |
| 主要任务 | 跟上横移目标 | 精准俯仰+抗重力 |
| 参数选择 | **纯 P**（+前馈） | **P + 小 I + 小 D**（+重力前馈） |

- **yaw 纯 P**：加 I 在快速跟踪时会产生滞后与过冲；加 D 会放大视觉噪声。要补滞后就上**前馈**（哨兵 `top_yaw_velocity_ff_*`）而不是加大 kp/kd。
- **pitch 需要 D**：变形步兵 `pitch_angle_kd: 0.3` 提供阻尼防过冲；`pitch_angle_ki: 0.02` 消重力引起的静差。但更关键的是**用重力前馈代替大 I**：

  ```cpp
  pitch_gravity_ff_gain_ * std::sin(*input_.pitch_angle - pitch_gravity_ff_phase_)
  ```

  这是把"重力力矩随俯仰角变化"这个**已知模型**直接算出来前馈（gain 4.302、phase 0.589），而不是靠 I 去"顶"——前馈不经过反馈路径，**不会引起振荡**，可在不影响稳定性的前提下改善响应。用 I 顶重力则会在限位处积分饱和。

**③ 为什么内环 ki 极小（1e-4 量级）而不是 0？**

自瞄跟踪匀速目标是**持续过程**，内环有微小 I 能消除速度跟随残差 → 减小外环角度滞后。但必须极小，否则低频振荡。哨兵给 `top_yaw_velocity_ki: 0.00033` 并配 `integral_min/max = ±2400` 限幅，就是标准做法。英雄 `pitch_velocity_ki: 0.00014` + `±2500` 同理。

**④ 为什么内环 kd 基本为 0、步兵 yaw 给了 0.0011？**

内环反馈是 IMU 陀螺角速度，噪声小、响应快。kd 作用是抑制小超调，给大了会把陀螺噪声放大成扭矩抖动。所以"能给一点点，不给也行"。

**⑤ 积分限幅为什么只有哨兵/英雄配了？**

因为只有它们的 ki 非零。`pid_calculator.hpp` 的默认限幅是 `±inf`，**如果不配又给了 ki，就等着 windup**：

```cpp
double integral_min = -inf, integral_max = inf;   // 默认无限制
```

这解释了为什么 yaml 里 `integral_min/max` 总是跟非零 ki 成对出现。

**⑥ 期望效果如何闭环验证？**

云台稳态角误差必须 < `yaw_tolerance: 0.07` / `pitch_tolerance: 0.04`，否则 `fire_control.cpp:402` 的 `armor_on_ray` 判定不会输出 `should_shoot`。所以"云台 PID 调得好不好"**最终由火控的开火率体现**。

### 5.6 小结

云台与舵轮同为串级：内环消除速度静差并抗扰、外环保证角度进入火控容差（< 0.07/0.04 rad），yaw 用前馈补跟踪滞后、pitch 用重力前馈代替大 I；凡 ki≠0 的环都必配 `integral_min/max` 防积分饱和。依据见第一节 A 的 `deformable_infantry_gimbal_controller.cpp`、`eccentric_dual_yaw.cpp` 与对应 yaml。

---

## 六、三场景横向对比（可直接写进结论）

| 维度 | 舵轮舵向 | 摩擦轮 | 自瞄云台 |
|---|---|---|---|
| 环结构 | 角度外环 + 速度内环 | 单环速度 | 角度外环 + 速度内环（哨兵为双 yaw+pitch 三组） |
| **P** | 内外都要（30 / 0.15） | 要（0.0034 或 0.0062） | 内外都要（10~35 / 2~16） |
| **I** | **不要**（ki=0） | **不要**（ki=0） | yaw 不要；pitch 要小量（0.01~0.02）；内环极小（1e-4） |
| **D** | 不要（kd=0） | **步兵要**（0.0094）；**英雄不要** | pitch 要（0.1~0.3）；yaw 不要 |
| 前馈 | 运动学期望角速度 | — | pitch 重力前馈 / 哨兵 yaw 角速度前馈 |
| 积分限幅 | 未配（ki=0 无影响） | 未配（ki=0 无影响） | **哨兵/英雄配了**（因 ki≠0） |
| 关键量化指标 | 稳态角误差≈0 | 转速 575~660 rad/s | **角误差 < 0.07/0.04 rad** |

**三条通用结论：**

1. **I 不是必须的**：只有存在**持续不变的外部负载**且要求稳态无静差时才需要。有前馈（重力/运动学）或机构本身无静载（摩擦轮空转、舵轮静止）时，加 I 往往是引入振荡和饱和的根源。
2. **D 不是必须的，但"有冲击/大惯量/需阻尼"时很值**：摩擦轮打弹（冲击）→ 要；pitch（重力+惯量）→ 要；舵轮转向（惯量小、指令平滑）→ 不要；英雄 6 轮（冲击小、噪声敏感）→ 不要。
3. **数值不能跨环比较**：`kp=30` 与 `kp=0.0034` 不代表增益差 1 万倍。判断强弱要看**误差多大时输出饱和**——这取决于量纲、电机扭矩常数（`dji_motor.hpp`）、减速比。

---

## 七、B 站视频参考（可配合正文观看）

> 选了几条与本文最相关的教程：按"先理解 PID → 再看串级 → 再看云台/速度环实调"的顺序看即可。

| 主题 | 视频（UP 主） | 链接 |
|---|---|---|
| PID 基础（告别盲目调参） | 从本质上理解 PID 控制器（控制欲超强） | [BV1NgnfzVERK](https://www.bilibili.com/video/BV1NgnfzVERK/) |
| 串级概念（对应全文"外环/内环"） | 串级 PID 与闭环带宽（控制欲超强） | [BV1aiD5BQEsS](https://www.bilibili.com/video/BV1aiD5BQEsS/) |
| 云台调参实战（对应场景③） | 保姆级 STM32 云台 PID 调参：从乱抖调到丝滑（洛珵） | [BV16edWBkEYD](https://www.bilibili.com/video/BV16edWBkEYD/) |
| 速度环调参（对应场景②） | PID 速度环增量式（附源码）原理讲解、调参（泰霉勒ba） | [BV1Nj421U7ma](https://www.bilibili.com/video/BV1Nj421U7ma/) |
| 电机底层电流环（补充） | FOC 的 PI 参数调节（拾小白电控foc） | [BV1MC4y137CT](https://www.bilibili.com/video/BV1MC4y137CT/) |

---

# 任务二 · 底盘运动学解算 + 龙门架控制

> 接上文（任务一 · 三场景 PID 参数分析，已结束）。任务二含两小题：
> **3.1 底盘运动学解算**（全向轮 / 舵轮逆解 + GeoGebra 可视化）、**3.2 龙门架双电机同步升降**（方案 + RMCS 控制组件）。
>
> 进度：公式与思路已整理 ✅；GeoGebra 图、龙门架组件代码、真机/空跑验证 → TODO。

---

## 一、3.1 底盘运动学解算

### 1.1 全向轮逆解

**建系与编号**：底盘固连系，x 指前、y 指左、绕 z 逆时针为正（与 RMCS `BaseLink::DirectionVector` 一致）。四轮编号取 `LF/RF/LB/RB`，第 i 轮向径角 $\varphi_i$（轮心相对车心的方向，逆时针为正）。

**关键一步**：把底盘速度投影到该轮滚动方向（切向）。轮滚动切向单位向量取
$$t_i = (-\sin\varphi_i,\ \cos\varphi_i)$$
轮心线速度 = 平动投影 + 底盘自转在该轮心处的贡献 $R\omega$：

$$v_i = -v_x\sin\varphi_i + v_y\cos\varphi_i + R\,\omega$$

因此第 i 轮目标角速度：
$$\omega_i = \frac{v_i}{r} = \frac{-v_x\sin\varphi_i + v_y\cos\varphi_i + R\,\omega}{r}$$

> **校验方法**：对照仓库 `rmcs_core/src/controller/chassis/omni_wheel_controller.cpp` 的逆解实现，以及舵轮版里
> `wheel_velocity_x = vx − R·vz·sinφ`、`wheel_velocity_y = vy + R·vz·cosφ`
> 再向轮方向投影 → 应与上式一致。φ 取 0/90/180/270（前后左右布置）或 45/135/225/315（X 布置）均可推回。

**可视化（GeoGebra）**：滑动条 `vx、vy、ω、R、r`；画车体（矩形）+ 4 个轮心 + 每轮线速度矢量箭头（方向=滚动切向、长度=速率）。用"纯旋转（vx=vy=0, ω≠0）时四箭头同为切向"检查公式正确性。TODO：插入 GeoGebra 截图/动图。

### 1.2 舵轮逆解（Steering / Swerve）

舵轮要同时给两个量：**转向角 θᵢ**（轮子朝哪）与**驱动角速度 ωᵢ**（轮子转多快）。

轮心线速度矢量（平动 + 自转在轮心处的切向速度）：
$$\vec c_i = \begin{pmatrix} v_x - R\,\omega\sin\varphi_i \\ v_y + R\,\omega\cos\varphi_i \end{pmatrix}$$

- 转向角 = 线速度方向：
$$\theta_i = \mathrm{atan2}\big(v_y + R\,\omega\cos\varphi_i,\ \ v_x - R\,\omega\sin\varphi_i\big)$$
- 驱动转速 = 速率除以半径（方向已由 θ 对准，无需负号）：
$$\omega_i = |\vec c_i|\,/\,r$$

**工程细节（对应仓库 `steering_wheel_controller.cpp` 的做法）**
- θ 可折进 $\pm\pi/2$：舵轮可反装驱动，避免转向电机绕远路（仓库 `.unaryExpr` 那段）；
- 死区：$|\vec c_i|$ 很小时 atan2 无意义，仓库改用**加速度方向**定角；加速度也≈0 时输出 NaN/0 扭矩——这是"公式之外"的工程点，值得写进报告。

**可视化**：每个轮画两个量——转向箭头（方向 θᵢ）与驱动箭头（沿 θᵢ、长度 $\propto \omega_i r$）。纯平移时四轮转向一致、纯旋转时四轮切向——正确性判据。TODO：插图。

---

## 二、3.2 龙门架运动（双电机同步升降）

### 2.1 问题本质与控制需求

两侧丝杆电机负载不同 → 升降不同步 → 横梁倾斜/卡死。因此不只是"各自跟目标"，还要**主动消除同步误差**
$$e_{sync} = h_L - h_R$$

| 需求 | 含义 | 后果 |
|---|---|---|
| 平缓 | 目标轨迹限速/限加速度（梯形或 S 形 profile） | 阶跃冲击、结构晃动 |
| 两侧同步 | 横梁保持水平 | 卡死、倾斜、飞镖 Pitch 不准 |
| 左右解耦抗偏载 | 负载重一侧不落后 | 一侧爬不动另一侧空转 |

### 2.2 反馈选择 / 无外部传感器时如何估计姿态

1. **电机自带编码器（首选，RMCS 的 DJI/LK 电机都有多圈角）**：丝杆高度由轴转角换算
$$h = \frac{\theta_{motor}}{2\pi} \cdot \frac{lead}{ratio} + h_0$$
（θ：电机多圈角，lead：丝杆导程 mm/rev，ratio：减速比）。左右各算各的 → **用两侧编码器差即可估计横梁倾斜**，这是"无外部传感器"的正解（以轴代梁）。
2. 要抗丝杆打滑/背隙才加外部传感器：拉线编码器 / 磁栅尺直接测梁高，接近开关做零点归位。
3. 完全没有编码器只能开环 → 无法做同步闭环，报告中应点明这一点。

### 2.3 控制方案：目标 profile + 双位置环 + 交叉耦合

```
目标 h*(t) ── S形/梯形 profile（限速、限加速度）──►
   左位置环: vL_ref = posPID(h* - hL)         # 高度→目标速度
   右位置环: vR_ref = posPID(h* - hR)
   同步纠偏: vL_ref += k_sync · (hL - hR)      # 交叉耦合
            vR_ref -= k_sync · (hL - hR)
   左右速度内环（各一）→ 扭矩 → 电机
```

- 平缓靠 profile：`max_velocity / max_acceleration` 做成可配参数；
- 同步靠交叉耦合：重的一侧落后 → $e_{sync}\ne 0$ → 自动"多推重侧、少推轻侧"；$k_{sync}$ 相当于横梁的虚拟刚性，别贪大（会振荡）。

### 2.4 RMCS 组件划分（照仓库套路）

| 层 | 文件 / 类 | 职责 |
|---|---|---|
| 目标源 | `GantryTargetController`（仿 AngleTargetController） | 给 h*(t)：固定/斜坡/方波，yaml 切换 |
| 控制器 | `GantryController` | profile + 左右位置环 + 交叉耦合 → 出左右目标速度 |
| PID 块 | 复用 `ErrorPidController` / `PidController`（yaml 装配，外置可独立调参） | 位置环→速度环→扭矩 |
| 硬件 | `GantryHardware`（仿 steering-hero 的 Board 写法） | C 板 CAN，两电机注册 `/gantry/left`、`/gantry/right` 的 angle/velocity/torque |
| 配置 | `config/gantry.yaml` | 组件列表 + 左右 PID + 导程/减速比 + profile 限幅 |

**"接入 C 板能跑（无实物）"**：保证 `colcon build` + `ros2 launch rmcs_bringup rmcs.launch.py robot:=gantry` 能起、话题通；无实物时用 `value_broadcaster` + Foxglove 看指令链；接 C 板后先空载低速验证编码器方向/零位，再上 profile 闭环。组件内用 `.ready()` 兜底（没接电机/NaN → 输出 0 扭矩）才能安全空跑。

TODO：龙门架组件骨架代码 + `gantry.yaml` + 跑通验证记录。

---

## 三、任务二 · B 站参考

| 主题 | 视频（UP 主） | 链接 |
|---|---|---|
| 全向轮底盘解算（RM 战队培训，最贴题） | 【26赛季电控培训】P6 全向轮底盘解算（青岛科大 YKTKEMIAO） | [BV1FpnfzNEFc](https://www.bilibili.com/video/BV1FpnfzNEFc/) |
| 全向轮解算 + 代码生成 | 全向轮运动学解算与 GPT 代码生成（西格螺） | [BV1rX4y1p7n7](https://www.bilibili.com/video/BV1rX4y1p7n7/) |
| 麦轮解算（对照） | 麦克纳姆轮解算·麦轮解算（bili我最烦取名字了） | [BV13yBuYEEMf](https://www.bilibili.com/video/BV13yBuYEEMf/) |
| 麦轮系统教程（对照） | 底盘运动学解析系统性教程（三）｜麦克纳姆轮小车（WHEELTEC） | [BV1Tyw9e8EK5](https://www.bilibili.com/video/BV1Tyw9e8EK5/) |
| 舵轮运动学（演示） | 4 舵轮正运动学解算（_无往而不胜_） | [BV1LE42157oM](https://www.bilibili.com/video/BV1LE42157oM/) |
| 舵轮可视化 | 舵轮运动学可视化（bilibili_duhuo） | [BV1Akjjz2ESh](https://www.bilibili.com/video/BV1Akjjz2ESh/) |
| 龙门同步控制 | 汇川伺服：龙门同步控制的 2 种方案（工控小黄人） | [BV1cC4y1M7Uh](https://www.bilibili.com/video/BV1cC4y1M7Uh/) |
