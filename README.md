# CarSim 2019.0 + Python 纵向速度跟踪控制 Demo

这份文档面向刚开始接触 CarSim 与 Python 联合仿真的读者。目标很简单：

**让 Python 在 CarSim 仿真过程中读取当前车速，通过PID控制调节 `throttle` 和 `brake` 两个控制量传回 CarSim，使车辆能够跟踪目标速度。**

## 1. 交互逻辑

CarSim 和 Python 的交互可以理解成一个循环：

```text
CarSim 输出车辆状态
    ↓
Python 读取状态，例如 Vx
    ↓
Python 计算 throttle / brake
    ↓
CarSim 接收 throttle / brake
    ↓
CarSim 前进一步并输出新状态
```

这里有两个数组最重要：

```text
export_vars: CarSim -> Python
import_vars: Python -> CarSim
```

在本文的 Demo 中：

```text
import_vars = [throttle, brake]
```

## 2. 本仓库的 Python 文件

4 个文件：

```text
carsim_vs_solver.py
carsim_io.py
carsim_longitudinal_env.py
example_longitudinal_pid.py
```

它们的作用如下。

| 文件 | 作用 |
| --- | --- |
| `carsim_vs_solver.py` | 负责加载 CarSim Solver DLL，并封装 `read_configuration()`、`integrate_io()`、`terminate_run()` 等底层接口。 |
| `carsim_io.py` | 负责把 CarSim 输出数组 `export_vars` 转成 Python 字典，并完成常见单位转换。 |
| `carsim_longitudinal_env.py` | 把 CarSim 包装成只接收 `throttle/brake` 的环境。它会检查 CarSim import 数量必须为 2。 |
| `example_longitudinal_pid.py` | 一个最小可运行 Demo：读取当前速度，用简单 PID 输出 `throttle/brake`，让车辆跟踪目标速度。 |

用 `example_longitudinal_pid.py` 确认 CarSim 和 Python 的数据交换是通的。

## 3. CarSim 侧需要创建什么 simfile

你需要在本地 CarSim 中创建一个用于纵向控制的 run，并导出或生成对应的 `simfile.sim`。

这个 run 必须满足两个要求。

### 要求 1：CarSim 只接收两个 import

Python Demo 只会向 CarSim 发送：

```text
[throttle, brake]
```

因此 CarSim 侧的 import 顺序必须是：

```text
IMPORT throttle
IMPORT brake
```

在 CarSim 展开的参数文件中，类似下面这样：

```text
IMPORT IMP_THROTTLE_ENGINE Replace 0.0 ! 1
IMPORT IMP_BK_STAT         Replace 0.0 ! 1
PORTS_IMP 1,2
```

不同 CarSim 配置中，制动变量名可能不是 `IMP_BK_STAT`，也可能是 `IMP_PCON_BK` 等。变量名可以不同，但顺序必须一致：

```text
第 1 个 import = throttle
第 2 个 import = brake
```

### 要求 2：CarSim 至少输出 Vx

Demo 里的 PID 控制器需要读取当前纵向车速，所以 CarSim 至少需要 export：

```text
EXPORT Vx
```

你也可以输出更多变量，例如：

```text
EXPORT Vx
EXPORT Ax
EXPORT Xo
EXPORT DisS1_1
EXPORT SpdS1_1
EXPORT V_Obj_1
PORTS_EXP 1,6
```

这些变量不是全部必需的。本 Demo 需要的是纵向车速 `Vx`。

如果你只输出 `Vx`，运行命令里就写：

```text
--export-names Vx
```

**注意：`--export-names` 的顺序必须和 CarSim 中 EXPORT 的顺序完全一致。**

## 4. simfile.sim 中应看到的关键信息

你的 `simfile.sim` 应该类似：

```text
PRODUCT_ID CarSim
PRODUCT_VER 2019.0
DLLFILE D:\CarSim2019.0\CarSim2019.0_Prog\Programs\solvers\carsim_64.dll
PORTS_IMP 1,2
PORTS_EXP 1,N
```

其中：

- `PRODUCT_VER 2019.0` 表示使用 CarSim 2019.0；
- `DLLFILE` 是 Python 要加载的求解器 DLL；
- `PORTS_IMP 1,2` 表示 Python 只输入两个量；
- `PORTS_EXP 1,N` 表示 CarSim 输出 N 个量。

`PORTS_EXP` 的数量由你自己决定，只要运行 Demo 时提供相同数量的 `--export-names` 即可。

## 5. Python 文件怎么运行

假设你已经把这 4 个 `.py` 文件放在同一个目录下，并且你已经创建好了simfile.sim：

```text
D:\xxx\simfile.sim
```

### 运行 Demo

运行：

```powershell
python .\example_longitudinal_pid.py `
  --sim-file ".\simfile.sim" `
  --target-speed 25 `
  --duration 10 `
  --log-csv ".\longitudinal_pid_log.csv"
```

含义：

- `--sim-file`：你的 CarSim simfile 路径；
- `--target-speed 25`：目标速度为 `25 m/s`；
- `--duration 10`：运行 10 秒；
- `--log-csv`：保存日志。

## 6. Demo 内部发生了什么

`example_longitudinal_pid.py` 做了下面几件事。

### 1. 加载 CarSim DLL

```python
env = CarSimLongitudinalEnv(sim_file=args.sim_file, ...)
```

环境内部会读取 `simfile.sim`，找到 `DLLFILE`，然后加载 `carsim_64.dll`。

### 2. 初始化 CarSim run

```python
obs = env.reset()
```

这一步会调用 CarSim 的 `read_configuration()`，读取：

```text
n_import
n_export
t_start
t_stop
t_step
```

如果 `n_import` 不是 2，程序会报错并停止。

### 3. 读取当前车速

```python
obs["vx"]
```

`carsim_io.py` 会把 CarSim 的 `Vx` 从 `km/h` 转成 `m/s`。

### 4. PID 计算 throttle / brake

```python
throttle, brake = controller.step(target_speed, obs["vx"])
```

如果当前速度低于目标速度，输出油门；如果当前速度高于目标速度，输出制动。

### 5. 把 throttle / brake 传给 CarSim

```python
obs, done, info = env.step(throttle, brake)
```

环境内部会构造：

```python
import_vars = [throttle, brake]
```

然后调用：

```python
solver.integrate_io(t_current, import_vars, export_vars)
```

### 6. 结束仿真

运行结束后调用：

```python
env.close()
```

它会调用 CarSim 的：

```python
terminate_run(t_current)
```

这一步不能省略。否则下一次初始化 CarSim run 时可能出现状态冲突。

## 7. 如果运行失败，先检查这些点

### 1. n_import 不是 2

错误类似：

```text
This demo expects exactly 2 CarSim imports: [throttle, brake].
```

说明你的 CarSim run 不是只输入 throttle 和 brake。请在 CarSim 中调整 import 配置，确保：

```text
PORTS_IMP 1,2
```

### 2. export 数量和 --export-names 数量不一致

例如 CarSim 输出 6 个变量，但你只写了：

```text
--export-names Vx
```

程序会报错。解决方法是让 `--export-names` 和 CarSim 的 `EXPORT` 顺序完全一致。

### 3. Vx 没有输出

如果没有 `EXPORT Vx`，PID 控制器无法知道当前速度。请至少添加：

```text
EXPORT Vx
```

### 4. DLL 路径不对

确认 `simfile.sim` 中存在：

```text
DLLFILE D:\CarSim2019.0\CarSim2019.0_Prog\Programs\solvers\carsim_64.dll
```

并且该文件真实存在。

### 5. 上一次 run 没有正常结束

如果 Python 中断或异常退出，CarSim Solver 可能残留状态。重新运行前可以重启 Python 进程；代码中正常结束时会自动调用 `terminate_run()`。

## 8. 各文件简要说明

### `carsim_vs_solver.py`

这是最底层封装。它不关心控制逻辑，只负责和 CarSim DLL 通信。

主要函数：

```python
load(sim_file)
read_configuration(sim_file)
copy_export_vars(n_export)
integrate_io(t_current, import_vars, export_vars)
terminate_run(t_current)
```

### `carsim_io.py`

负责把 CarSim 输出数组转成更容易读的字典。

例如：

```python
obs = parse_observation(export_vars, export_names)
```

输出：

```python
{
    "vx": 24.8,
    "ego_x": 120.5,
    "rel_x": 35.0,
    ...
}
```

### `carsim_longitudinal_env.py`

这是最适合后续扩展的文件。它把 CarSim 包装成：

```python
obs = env.reset()
obs, done, info = env.step(throttle, brake)
env.close()
```

它强制要求 CarSim 只有两个 import，因此可以避免控制通道顺序混乱。

### `example_longitudinal_pid.py`

这是最小 Demo。它用 PID 计算油门和制动，让车辆跟踪目标速度。

你应该先跑通这个文件，再考虑加入更复杂的控制器或学习模型。

## 9. 后续可以怎么扩展

跑通 Demo 后，可以逐步扩展：

1. 增加更多 export，例如前车距离、前车速度、加速度；
2. 把 PID 换成你自己的控制策略；
3. 把 `target_speed` 改成随场景变化；
4. 增加日志字段；
5. 批量运行多个 `simfile.sim`；
6. 再接入更复杂的模型。

但在扩展前，请保持一个原则：

```text
CarSim import 顺序必须和 Python import_vars 顺序一致。
```

本文最小 Demo 的顺序固定为：

```text
[throttle, brake]
```

## 10. 核心结论

- 最小 CarSim-Python 纵向控制只需要两个输入：`throttle` 和 `brake`。
- CarSim 侧必须配置 `PORTS_IMP 1,2`。
- CarSim 至少需要输出 `Vx`，这样 Python 才能做速度控制。
- `--export-names` 必须和 CarSim 的 `EXPORT` 顺序完全一致。
- `example_longitudinal_pid.py` 是第一个应该运行的 Demo。
- 每次仿真结束都必须调用 `terminate_run()`，代码中的 `env.close()` 已经处理这件事。
