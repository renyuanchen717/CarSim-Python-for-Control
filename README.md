# CarSim-Python-for-Control

这份文档写给刚开始接触 CarSim 与 Python 联合仿真的读者。你不需要一开始就理解 CarSim 的所有配置，也不需要先会搭建复杂的神经网络。本文只关注一个问题：

**如何让 Python 在 CarSim 仿真过程中读取车辆状态，并把油门、制动控制量传回 CarSim。**

在这个基础上，我们再说明如何把 Python 中的 MoE 模型放在“功能选择器”的位置，让它根据车辆和环境状态选择调用 ACC、AEB 等 ADAS 功能。

## 1. 先建立直觉

CarSim 可以理解为一个车辆仿真器。它负责计算车辆动力学，比如车辆速度、位置、横摆角、障碍物相对位置等。

Python 可以理解为外部控制器。它负责根据 CarSim 返回的状态做判断，然后输出控制量。

整个过程像这样循环：

```text
CarSim 当前状态
    ↓
Python 读取状态
    ↓
Python 决定油门和刹车
    ↓
CarSim 接收控制量并前进一步
    ↓
CarSim 返回新的状态
```

这就是 CarSim 与 Python 联合仿真的核心。

## 2. 本文使用的简化场景

为了让流程更容易理解，本文只讨论纵向控制 (不涉及左右转向)。

Python 输出两个量：

```text
throttle  油门
brake     制动
```

本文的重点是：

- 如何读取车速；
- 如何读取前方目标车或障碍物位置；
- 如何判断该加速、跟车还是制动；
- 如何把 `throttle` 和 `brake` 传给 CarSim。

## 3. CarSim 和 Python 怎么交换数据

CarSim 和 Python 之间不是通过图片、传感器流或者网络消息交互，而是通过两个数组交换数据。

一个数组叫 `export_vars`，表示 CarSim 输出给 Python 的状态。

一个数组叫 `import_vars`，表示 Python 输入给 CarSim 的控制量。

可以这样理解：

```text
export_vars: CarSim -> Python
import_vars: Python -> CarSim
```

每一个仿真步，Python 做的事情就是：

```python
return_code, export_vars = solver.integrate_io(
    t_current,
    import_vars,
    export_vars,
)
```

这行代码的意思是：

1. 把当前的 `import_vars` 交给 CarSim；
2. 让 CarSim 前进一步；
3. 从 CarSim 取回新的 `export_vars`。

## 4. Python 给 CarSim 的输入

在纵向控制示例中，CarSim 接收 4 个输入：

```text
import_vars = [
    throttle,
    brake
]
```

含义如下：

| 名称 | 含义 | 本文怎么使用 |
| --- | --- | --- |
| `throttle` | 油门开度 | Python 输出 |
| `brake` | 制动输入 | Python 输出 |

因此，对本文的纵向控制任务来说，真正需要模型或控制器决定的只有：

```python
throttle = 0.2
brake = 0.0
```

## 5. CarSim 返回给 Python 的状态

CarSim 会返回车辆和目标物体的状态。常见输出如下：

```text
export_vars = [
    Vx,
    Vy,
    Yaw,
    MuX_L1,
    Xo,
    Yo,
    X_Obj_1,
    Y_Obj_1,
    AVz,
]
```

这些变量可以理解为：

| 名称 | 含义 | 对 ADAS 有什么用 |
| --- | --- | --- |
| `Vx` | 自车纵向速度 | 判断当前车速 |
| `Vy` | 自车侧向速度 | 判断车辆是否稳定 |
| `Yaw` | 自车横摆角 | 判断车辆姿态 |
| `MuX_L1` | 路面附着条件 | 判断湿滑程度 |
| `Xo` | 自车 X 坐标 | 计算相对距离 |
| `Yo` | 自车 Y 坐标 | 判断横向位置 |
| `X_Obj_1` | 前方目标 X 坐标 | 计算前方距离 |
| `Y_Obj_1` | 前方目标 Y 坐标 | 判断是否同车道 |
| `AVz` | 横摆角速度 | 判断车辆稳定性 |

原始输出通常需要做单位转换。例如速度可能是 `km/h`，而 Python 控制器中更常用 `m/s`。

一个常见转换函数如下：

```python
import math

def parse_obs(raw):
    ego_x = raw[4]
    ego_y = raw[5]
    obj_x = raw[6]
    obj_y = raw[7]

    return {
        "vx": raw[0] / 3.6,
        "vy": raw[1] / 3.6,
        "yaw": math.radians(raw[2]),
        "mu": raw[3],
        "ego_x": ego_x,
        "ego_y": ego_y,
        "obj_x": obj_x,
        "obj_y": obj_y,
        "rel_x": obj_x - ego_x,
        "rel_y": obj_y - ego_y,
        "yaw_rate": math.radians(raw[8]),
    }
```

对 ACC 和 AEB 来说，最重要的通常是：

```text
vx      当前自车速度
rel_x   前方目标与自车的纵向距离
rel_y   前方目标与自车的横向距离
mu      路面附着条件
```

## 6. 简单的速度控制例子

PID 速度控制器的目标很简单：

- 如果当前车速低于目标车速，就给油门；
- 如果当前车速高于目标车速，就给制动；
- 如果速度接近目标，就保持较小控制量。

简化后的逻辑如下：

```python
target_speed = 30.0  # m/s
current_speed = obs["vx"]

error = target_speed - current_speed

if error > 0:
    throttle = min(error * 0.1, 1.0)
    brake = 0.0
else:
    throttle = 0.0
    brake = min(-error * 0.1, 1.0)

import_vars = [throttle, brake, 0.0, 0.0]
```

这个例子虽然简单，但它已经包含了 CarSim-Python 交互的全部关键步骤：

1. 从 CarSim 读取速度；
2. Python 根据速度计算控制量；
3. 把油门和刹车传回 CarSim；
4. CarSim 更新车辆状态。

## 7. 一个完整的交互循环长什么样

下面是一个完整但仍然简化的流程。

```python
import ctypes
from vs_solver import vs_solver

sim_file = "baseline1/simfile.sim"

solver = vs_solver()
dll_path = solver.get_dll_path(sim_file)
carsim_dll = ctypes.CDLL(dll_path)

if not solver.get_api(carsim_dll):
    raise RuntimeError("CarSim solver API 加载失败")

config = solver.read_configuration(sim_file)

t_current = config["t_start"]
t_step = config["t_step"]
n_export = config["n_export"]

export_vars = solver.copy_export_vars(n_export)

target_speed = 30.0

for step in range(100):
    obs = parse_obs(export_vars)

    error = target_speed - obs["vx"]

    if error > 0:
        throttle = min(error * 0.1, 1.0)
        brake = 0.0
    else:
        throttle = 0.0
        brake = min(-error * 0.1, 1.0)

    import_vars = [throttle, brake, 0.0, 0.0]

    return_code, export_vars = solver.integrate_io(
        t_current,
        import_vars,
        export_vars,
    )

    if return_code != 0:
        break

    t_current += t_step

solver.terminate_run(t_current)
```

这段代码的作用是确认：

- CarSim DLL 能被 Python 正常加载；
- Python 能拿到 CarSim 输出；
- Python 写入的油门和刹车能影响车辆运动；
- 仿真可以正常开始和结束。
- 

## 8. MoE 在这里扮演什么角色

当前目标不是让 MoE 直接学习底层车辆动力学，也不是让 MoE 直接输出复杂的所有控制量。

更清晰的定位是：

**MoE 是 ADAS 功能选择器。**

也就是说，MoE 根据当前状态选择调用哪个功能：

```text
当前距离很远，速度正常       -> Cruise
前方有车，需要保持距离       -> ACC
前方距离过近，有碰撞风险     -> AEB
```

然后，被选中的 ADAS 功能再输出具体控制量：

```text
ACC -> throttle / brake
AEB -> throttle / brake
```

数据流可以写成：

```text
CarSim 状态
    ↓
MoE 选择 ADAS 功能
    ↓
ADAS 功能计算 throttle / brake
    ↓
CarSim 执行控制
```

## 9. MoE + ADAS 的闭环例子

下面是一个简化的伪代码。这里不讨论 MoE 怎么训练，只说明它在闭环里放在哪里。

```python
obs = env.reset()

for step in range(max_steps):
    selected_adas = moe_selector.select(obs)

    if selected_adas == "ACC":
        cmd = acc.run(obs)
    elif selected_adas == "AEB":
        cmd = aeb.run(obs)
    else:
        cmd = cruise.run(obs)

    obs, done, info = env.step(
        throttle=cmd["throttle"],
        brake=cmd["brake"],
    )

    if done:
        break

env.close()
```

这个结构有几个好处：

- MoE 只负责“选功能”，任务边界清晰；
- ACC、AEB 等功能可以先用规则或传统控制器实现；
- 后续可以替换 MoE，而不用改 CarSim 接口；
- 记录 `selected_adas` 后，可以分析模型在什么场景下选择了什么功能。

## 10. 建议记录哪些数据

如果要训练或分析 MoE，建议每个控制周期记录一行数据：

```text
time
vx
mu
ego_x
ego_y
obj_x
obj_y
rel_x
rel_y
yaw_rate
selected_adas
throttle
brake
```

这些数据可以回答很多问题：

- 当前车速是多少；
- 前方目标距离是多少；
- MoE 选择了 ACC 还是 AEB；
- 选择 AEB 时是否真的存在危险；
- 油门和制动是否平滑；
- 不同路面附着条件下选择是否合理。

如果后续需要更复杂分析，还可以额外记录：

```text
relative_speed
ttc
safe_distance
adas_confidence
expert_scores
```

## 11. 仿真的开始和结束

CarSim 的每一轮仿真都要有明确的开始和结束。

开始时调用：

```python
config = solver.read_configuration(sim_file)
```

这一步会读取 CarSim 的配置并初始化仿真。

结束时必须调用：

```python
solver.terminate_run(t_current)
```

不要省略结束步骤。CarSim Solver DLL 会在进程中保存当前 run 的状态。如果上一轮仿真没有正常结束，下一轮可能初始化失败。
