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

为了让流程更容易理解，本文只讨论纵向控制，也就是车辆沿道路前后方向的控制。

Python 只需要输出两个量：

```text
throttle  油门
brake     制动
```

暂时不考虑转向控制。即使 CarSim 的接口中还包含转向输入，我们也先把转向固定为 0，让车辆直行。

所以本文的重点不是“如何让车换道”，而是：

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
    brake,
    steer_L1,
    steer_R1,
]
```

含义如下：

| 名称 | 含义 | 本文怎么使用 |
| --- | --- | --- |
| `throttle` | 油门开度 | Python 输出 |
| `brake` | 制动输入 | Python 输出 |
| `steer_L1` | 左前轮转向 | 固定为 0 |
| `steer_R1` | 右前轮转向 | 固定为 0 |

因此，对本文的纵向控制任务来说，真正需要模型或控制器决定的只有：

```python
throttle = 0.2
brake = 0.0
```

送入 CarSim 时再补齐为：

```python
import_vars = [throttle, brake, 0.0, 0.0]
```

这一步非常重要。它说明了一个原则：

**模型不一定要直接输出 CarSim 需要的全部输入，环境封装层可以帮模型补齐固定项。**

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

## 6. 最简单的纵向控制例子

在还没有 MoE 模型之前，可以先用一个简单的纵向 PID 控制器验证接口是否通畅。

PID 控制器的目标很简单：

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

这段代码可以先不用 MoE。它的作用是确认：

- CarSim DLL 能被 Python 正常加载；
- Python 能拿到 CarSim 输出；
- Python 写入的油门和刹车能影响车辆运动；
- 仿真可以正常开始和结束。

## 8. 为什么要封装成环境

如果后面要训练模型，最好不要在训练代码里到处直接调用 CarSim DLL。更好的做法是把 CarSim 包装成一个环境。

这个环境只暴露三个动作：

```python
obs = env.reset()
obs, done, info = env.step(throttle, brake)
env.close()
```

对模型来说，它不需要知道 CarSim DLL 怎么加载，也不需要知道 `import_vars` 里还有两个转向输入。

模型只需要关心：

```text
输入：当前车辆和环境状态
输出：throttle / brake
```

环境内部负责把它变成：

```python
import_vars = [throttle, brake, 0.0, 0.0]
```

这种封装可以让后续代码更清晰，也方便替换控制器或模型。

## 9. MoE 在这里扮演什么角色

当前目标不是让 MoE 直接学习底层车辆动力学，也不是让 MoE 直接输出复杂的所有控制量。

更清晰的定位是：

**MoE 是 ADAS 功能选择器。**

也就是说，MoE 根据当前状态选择调用哪个功能：

```text
当前距离很远，速度正常       -> Cruise
前方有车，需要保持距离       -> ACC
前方距离过近，有碰撞风险     -> AEB
状态异常或不确定             -> Safe Stop
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

## 10. MoE + ADAS 的闭环例子

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

## 11. 建议记录哪些数据

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

## 12. 仿真的开始和结束

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

如果你把 CarSim 包装成环境，建议在 `reset()` 时检查上一轮是否结束；如果没有结束，先调用 `close()`。

## 13. Python 能不能修改 CarSim 场景

这一点很容易误解。

Python 在仿真过程中能实时修改的，是 CarSim 已经声明为 `import_vars` 的输入。

在本文的纵向控制例子里，Python 可以实时修改：

```text
throttle
brake
steer_L1
steer_R1
```

但我们只使用前两个：

```text
throttle
brake
```

Python 不能通过普通 `import_vars` 直接实时修改：

- 道路线形；
- 车道宽度；
- 障碍物长宽高；
- Moving Object 的三维形状；
- CarSim 车辆参数；
- 已经初始化完成的道路结构。

这些通常是 CarSim run 初始化时读入的配置。

如果你想做多场景实验，更稳妥的方式是准备多个 CarSim run 或多个 `simfile.sim`：

```text
场景 1：正常路面 + 远距离前车
场景 2：低附着路面 + 近距离前车
场景 3：高速接近静止障碍物
```

然后每一轮 episode 选择一个场景，初始化对应 run，仿真结束后再切换下一个场景。

## 14. 推荐上手顺序

建议按这个顺序推进：

1. 先跑通 CarSim + Python 最小循环；
2. 打印 `export_vars`，确认 Python 能读取 CarSim 状态；
3. 用固定 `throttle` 和 `brake` 测试车辆是否响应；
4. 加入简单纵向 PID，验证速度控制；
5. 封装成 `reset()`、`step()`、`close()` 环境；
6. 实现 Cruise、ACC、AEB 等功能接口；
7. 加入 MoE，让它选择调用哪个 ADAS 功能；
8. 记录每一步状态、功能选择和控制量；
9. 做多场景评估。

## 15. 核心结论

- CarSim 与 Python 的交互本质是两个数组：`export_vars` 和 `import_vars`。
- CarSim 把车辆和环境状态放进 `export_vars`，Python 读取后做决策。
- Python 把油门和制动放进 `import_vars`，CarSim 读取后推进仿真。
- 对入门纵向控制来说，模型只输出 `throttle` 和 `brake` 就够了。
- 如果 CarSim 还要求转向输入，可以在环境封装层补 `0.0`。
- MoE 更适合作为 ADAS 功能选择器：它选择 ACC、AEB 等功能，而不是一开始直接控制所有底层变量。
- 修改道路、障碍物尺寸等 CarSim 场景信息，通常需要切换或重新生成 CarSim run，而不是在仿真过程中直接改 `import_vars`。

