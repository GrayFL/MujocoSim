# MujocoSim

基于 MuJoCo 的机器人、传感器与场景仿真工具，通过可选的 RSim 集成提供 Signal / CommandSink 与设备接口。目前 demo 是 0.6 m × 0.6 m 差速机器人、二维雷达、前向 RGBD，以及 HM3D 场景和可推动的桌椅。

**仿真实现全部位于本项目。** RSim 只提供通用组件运行、通信与设备接口；MuJoCo 的线程、采样、命令安全和生命周期由本项目的 `mujocosim.rsim` 负责。项目结构和扩展方法见 [ARCHITECTURE.md](ARCHITECTURE.md)，本机路径、开发约束与验证记录见 [PROJECT.md](PROJECT.md)。

## 安装

使用 Python ≥ 3.12。四个项目须使用兼容的本地源码版本，不要用公共索引上的同名包替换。源码链接 / `.pth` 开发模式可直接使用；安装元数据按能力区分：

```sh
uv pip install --dry-run -e '.[server]'   # 先检查解析结果
uv pip install -e '.[engine]'            # 独立同步物理，无 RSim
uv pip install -e '.[server]'            # 引擎 + RSim/DDS 设备提供端，无 pyrobo
uv pip install -e '.[rsim,dds]'          # 专有设备客户端，无 MuJoCo
uv pip install -e '.[server,apps]'       # 点云/像素导航应用
```

基础包仅依赖 NumPy；`engine` 包含 MuJoCo、SciPy、trimesh 和 Pillow，`scene` 另含大网格减面使用的 fast-simplification，`apps` 包含导航与绘图依赖。仅需要公共底盘接口的消费者不必安装 MujocoSim，可使用 `rsim.devices.ChassisPortsDevice`。本轮不要求构建 wheel。

RGBD 和预览需要 OpenGL。`--headless` 关闭交互窗口，仍会离屏渲染最终图像。macOS 被动 viewer 使用 `mjpython` 启动。原始数据单独准备，不随仓库分发。

先把外部数据目录配置为 `MUJOCO_SCENE_ROOT`。它统一存放场景、模型资产、转换缓存和建图结果；仓库的 `assets/` 只存可视化、日志及小型临时实验输出。

```powershell
# PowerShell，替换为实际的项目外数据目录
$env:MUJOCO_SCENE_ROOT = 'E:/Datasets/mujoco'
```

```sh
# Linux / macOS
export MUJOCO_SCENE_ROOT=/path/to/mujoco
```

下文 `/path/to/mujoco` 均指这个外部目录。迁移时复制数据目录并设置该变量；本机默认位置见 [PROJECT.md](PROJECT.md)。在启动 Python / Notebook 内核前设置环境变量。

Linux / WSL 必须使用 Linux 路径；例如 Windows 数据盘 `D:\Datasets\mujoco` 在 WSL 中通常写为 `/mnt/d/Datasets/mujoco`。未设置变量时，Linux / macOS 默认使用 `~/Storage/Datasets/mujoco`。在 Linux 上误填 Windows 盘符路径会明确报错，避免在项目内创建同名相对目录。

## 运行 demo

```sh
python -m mujocosim --scene /path/to/mujoco/hm3d/scene.glb --preview
python -m mujocosim --scene /path/to/mujoco/hm3d/scene.glb --headless --mode demo --seconds 12 --output assets/runs/demo
python -m mujocosim.apps.motion --scene /path/to/mujoco/hm3d/scene.glb --output assets/runs/motion
```

统一使用 `python -m mujocosim`；安装后也可运行 `mujocosim`。其他应用位于 `mujocosim.apps`。直接运行 `python -m mujocosim.apps.demo` 时显式传入场景。

| 参数 | 用途 |
|---|---|
| `--scene` | GLB/GLTF、组合 JSON 或已转换目录 |
| `--up-axis auto/y/z`、`--floor-height H` | GLB/GLTF 朝上轴、源坐标中的楼层高度 |
| `--resolution R` | GLB/GLTF 碰撞体素尺寸，默认 0.08 m |
| `--spawn X Y YAW` | 世界坐标中的 m / m / rad |
| `--cache-dir`、`--output` | 默认 `<data-root>/cache`、`assets/runs/latest` |
| `--prepare-only`、`--rebuild` | 仅准备场景、强制重建 |
| `--mode teleop/demo/idle` | 键盘控制、避障演示、静止 |
| `--seconds`、`--record-every` | 仿真时长、快照间隔 |
| `--width`、`--height`、`--preview` | 相机分辨率、实时图像窗口 |

MuJoCo 窗口中 ↑/↓ 前后，←/→ 转向，空格停止，R 重置机器人及动态物体，P 快照，Q 退出。方向键设置持续速度，按空格停车；WASD 保留给 viewer 原有快捷键。`--mode demo` 使用真值、静态净空和雷达，是功能演示；点云全局导航见下节。

## 场景与资产

```text
<data-root>/                   # 仓库外的数据目录
  test/chair.glb
  test/table.glb
  hm3d/4ok3usBNeis.glb
  scenes/simple_indoor.scene.json
  scenes/4ok3usBNeis_furnished.scene.json
  templates/                  # 布景模板；随数据一起迁移
  cache/                      # OBJ/PNG、MJCF、导航缓存和 scene_index.json
  maps/                       # 建图点云、拓扑、Notebook 地图导出
assets/
  runs/            # 可视化、日志、结果 JSON、小型传感器快照
  inspection/      # 原始资产检查图
```

把两个物体以及 `scenes/*.scene.json` 或 `templates/*.scene.json` 放入数据目录，再构建。模板属于外部数据，不随代码仓库分发；迁移时须一起复制。

```sh
python -m examples.build_scenes --scene-root /path/to/mujoco --hm3d /path/to/source/4ok3usBNeis.glb
python -m mujocosim --scene /path/to/mujoco/scenes/simple_indoor.scene.json --preview
python -m mujocosim --scene /path/to/mujoco/scenes/4ok3usBNeis_furnished.scene.json --preview
python -m examples.check_scenes --output assets/runs/scene_checks
```

构建脚本保留已安装 JSON；`--refresh-recipes` 明确覆盖为数据目录下的模板，`--rebuild` 重建缓存，`--only simple_indoor` 仅构建规则房间。指定 `--scene-root` 时，缓存默认写入该目录的 `cache/`。没有模板时可直接使用已有 `scenes/*.scene.json`；两处都没有配置时明确报错。JSON 的源资产路径相对 JSON 本身，格式为 `mujocosim-scene-v1`。组合的朝上轴、楼层、分辨率与碰撞高度以 JSON 为准。

省略 `--scene` 时加载数据目录下的 `scenes/4ok3usBNeis_furnished.scene.json`。`prepare_scene`、各应用和 RSim 后端默认共用外部缓存。可用 `--cache-dir` 指定其他外部缓存目录。轻量输出的相对路径以工作目录为准。场景索引中的 prepared 路径相对索引目录；数据位置变化后重跑构建脚本，源路径签名变化会生成新缓存。

规则房间大小 6 × 5 × 2.6 m。圆桌保持约 1 × 1 × 0.742 m，质量假设 18 kg；椅子等比缩至 0.763 × 0.830 × 0.9 m，质量假设 9 kg。两者有自由刚体、摩擦、分部碰撞，可推动或倾倒。JSON 支持 `position`、`yaw`、`scale`、`target_height`、`mass`、`friction`、`colliders`；尺寸为米、角度为弧度。collider 坐标先转 Z-up、底部归零、XY 居中，再随物体统一缩放；box 使用半尺寸，capsule 使用半径和 fromto。

## 同步物理 API

```python
from mujocosim import Simulation, SensorConfig, PhysicsConfig, prepare_scene
from mujocosim.robots import DifferentialDrive, DriveCommand

scene = prepare_scene("/path/to/mujoco/hm3d/scene.glb")
with Simulation(scene, robot=DifferentialDrive(), sensors=SensorConfig(),
                physics=PhysicsConfig(timestep=.002)) as sim:
    sim.command(DriveCommand(.2, .1))
    sim.step(.5)
    scan, frame = sim.lidar(), sim.rgbd()
    sim.save_snapshot("assets/runs/snapshot")
    sim.export_model()  # 按需导出与资产放在一起的 world.xml
```

核心 Simulation 必须显式选择机器人；`sensors=None` 可不安装雷达/RGBD。同步命令持续有效到下次命令或停止，没有 RSim TTL。正常推进只操作执行器；复位使用所选机器人自己的 reset 方法。新机器人实现见 [扩展说明](ARCHITECTURE.md)。

## RSim 集成

```python
from rsim import Runtime, VelocityCommand
from rsim.devices import ChassisPortsDevice
from mujocosim.rsim import MujocoChassisFactory

factory = MujocoChassisFactory(scene_dir="/path/to/prepared-scene", motion_enabled=True)
source = factory.build()                         # 未启动，不创建引擎或线程
chassis = ChassisPortsDevice.from_component(source)
async with Runtime(chassis.pose, chassis.state, chassis.velocity):
    frame = await chassis.pose.get(timeout=10)
    await chassis.velocity.set(VelocityCommand(.2, .0), ttl=.2)
```

RSim 组合要求 `rsim>=0.4.0`（包含端口校验、episode历史隔离与通用客户端）。完整的 16 个端口只在 `mujocosim.rsim.drivers.chassis.MujocoChassisFactory.port_specs()` 声明。`build/shared` 捕获场景内容指纹，并在 owner 创建资源前复核；文件变化后须重新创建描述。显式 `scene_revision` 同样检查实际资源内容，不能替代摘要。生成缓存的位置 `cache_root` 和运行日志不影响指纹，物理、传感器、推进模式和安全配置会影响指纹。`build()` 构造本地组件，`shared(key=...)` 构造显式启动描述，`device(key=...)` 只连接已运行实例。`INTERFACE_VERSION='chassis-ports-v1'` 与 pyrobo 内存设备一致；共同端口为 `pose/state/velocity`。`pose` 明确是 ground truth，地图和导航输入由应用注入。

`factory.service(source=shared, ...)` 可将同一个 World 包装成原 `rsim.devices.Chassis` 的 RPC 服务；Shared Ports 与 RPC 是不同接口，不能交换客户端。速度默认 TTL 上限 .5 s，运动默认关闭。一个 World 的物理、传感器、reset 和 inspect 始终由同一 owner 线程串行处理；数据读取不推进物理。

专有轻量客户端无需场景文件或 MuJoCo：

```python
from mujocosim.rsim import SimulationDevice
view = SimulationDevice(key="mujoco:my_world", clock="simulation", transport=dds)
async with Runtime(view.state, view.reset_request):
    acknowledgement = await view.reset()  # 完成 reset 和首帧发布后返回 episode
```

reset 采用原生物理时间（重新从零开始），增加 episode，清除旧历史、推进回执缓存并退休旧命令 epoch；控制器需使用新 epoch 才能恢复。已开始的 reset 会完成 episode 事务后再传播调用方取消，关闭会等待事务结束。若物理 reset 后采样失败，清空历史并保持暂停，必须先成功 reset 再显式 resume。暂停同样清除执行命令，恢复不会续命旧命令。`inspect(callback)` 是本地 owner 线程工具入口，回调应返回快照，不能把活动 MjData 交给其他线程；远端只提供固定 JSON 管理操作。新组件不提供 `.simulation` 公共可变视图。

### 三种推进模式

`MujocoChassisFactory(scheduling='realtime', speedup=1.)` 默认持续推进。`clock` 是源时间的语义域，与推进模式独立。所有模式始终使用 `physics.timestep`，负载不会扩大 dt；`1/hz` 必须是物理 dt 的整数倍。实时路径保持 host metronome，加速路径使用可调 host 节奏并跳过错过的唤醒，避免无界追赶。请求倍率是节奏配置，实际 RTF 需要测量。

| 模式 | 物理推进 | 管理操作 |
| --- | --- | --- |
| `realtime` | 默认连续运行 | pause、resume、reset、describe |
| `accelerated` | 按 `speedup` 调整 wall-time 节奏 | 同上，增加 `set_rate(rate)` |
| `controlled` | 不注册自主推进任务 | 显式 advance、describe/synchronize、abort、reset |

pause 立即退休命令，在下一完整物理子步前阻止继续推进，不发布新 tick。resume 和运行中的有效倍率变化增加 episode，清空历史/旧回执，发布新的边界快照；保留物理时间、控制 step_index 和累计物理子步数。暂停时修改倍率，在 resume 后生效。reset 才将 step_index 和累计子步数归零。帧元数据包含 `session/episode/step_index/source_rate/physical_substeps`。

加速源可在打开后调用 `source.subscribe(policy='latest')` 或有限容量的 `policy='every', capacity=...`。每个消费者有独立订阅；`every` 在队列满时产生背压，暂停/重置会解除旧投递的等待。实时模式不接受 every。事件送达只是节奏通知，不能当作计算完成或命令接纳回执。跨进程消费者可使用 RSim 的 `SignalPacer` 订阅现有状态信号。

### 受控推进和回执

`SimulationDevice` 的 `advance_request / inspect_request / control_request` 与 reset 一起构成专用管理端口。通用 Chassis 客户端仍只需要 `pose/state/velocity`，不因此订阅图像或载入引擎。以下连接一个已经运行的 controlled 提供端：

```python
async with Runtime(view.state, view.velocity, view.advance_request, view.inspect_request):
    status = await view.describe()  # synchronize() 同义，返回已提交状态
    accepted = await view.velocity.set(VelocityCommand(.1), ttl=.3)
    advanced = await view.advance(accepted['context'], accepted['command_id'], 10)
    assert accepted['kind'] == 'CommandAccepted'
    assert advanced['kind'] == 'PhysicsAdvanced'
```

`context` 是 `{session, episode, step_index}`，`command_id` 是 `{controller_id, controller_epoch, sequence}`。命令接纳回执还包含原始主机 `deadline_ns`、`source_deadline_ns` 和 `port`；只有 owner 实际执行源期限时 `source_deadline_ns` 才非空，无显式源期限的 realtime/controlled 命令返回 null。advance 请求包含 `context/command_id/requested_substeps`；`requested_substeps` 计固定物理 dt 次数，成功只增加一个控制 step_index。默认最多 1000 个子步，可通过 `max_substeps` 配置。

只有全部积分和快照发布完成才返回 `PhysicsAdvanced`：其中 `request_context` 是 k，`context` 是 k+1，同时给出命令身份、请求/实际子步数、累计 `physical_substeps`、源 `stamp_ns`、`phase='READY'` 和状态摘要。相同请求重试返回原回执，即使原命令 TTL 已过期；同一 context 换命令或子步数会被拒绝。默认缓存最近 32 个请求，可用 `ack_capacity` 配置；被淘汰的旧步只能拒绝，不能重新执行。管理端口对旧或未知 source context 返回 `CommandRejected(reason='stale_context')`，同一管理连接仍可发送新 context；运动控制器 epoch 的退役仍返回 `retired`。

每个子步前同时复核主机 monotonic TTL 和可选的 `SourceCommandBound`。后者保留提交时的源 context/clock/deadline，接纳时不重新锚定；实时模式同样检查。没有显式源期限的加速命令，还保留按接纳时剩余主机预算和倍率计算的固定源期限。部分积分、请求过期、命令身份不匹配或引擎失败会进入 `ABORTED`，停止后续 advance，恢复需要 reset。`describe/synchronize` 返回真实已知的 stamp、累计子步和 `last_result.kind='PhysicsAborted'`；原生状态无法读取时 `state_known=false`，未知数值为 null，不能视为零步。

调用方取消会等正在进行的物理事务完成或 ABORTED 后再传播。ACK 丢失不代表没有推进；先 synchronize，再用完全相同的请求恢复缓存结果或显式 reset。`abort(reason)` 可主动终止会话。ABORTED 状态的远端 resume 返回普通 `CommandRejected`，同一连接仍可 reset 后恢复；本地 `source.resume()` 保留 `RuntimeError` 契约，其他 owner 故障不会被当成普通状态拒绝隐藏。完整 `SimulationDevice` 需要新管理端口；旧 Provider 必须停止并用新工厂重建，通用 Chassis 子集保持兼容。

服务 CLI 同样接受 `--scheduling realtime|accelerated|controlled` 和 `--speedup N`。这些接口提供受控物理事务；计算阶段之间的锁步顺序由应用的 RSim stepper 显式协调。

旧 `mujocosim.integrations.rsim.create_source` 保留到 0.4 的过渡入口，委托同一工厂并显式采用 `legacy_continuous` 时间策略；旧直接 `.simulation` 访问仅存在于该兼容 facade。停止 Provider 后整体更新 RSim 与 MujocoSim，再按新工厂重建，不恢复旧进程的 pickle。 点云导航共享视图同步升级为 `mujocosim-navigation-v2`，旧客户端须更新后重新连接。

跨进程保持相同应用接口（安装 DDS extra，同机、同一用户）：

```sh
# 提供者终端；先构建场景，避免首次转换耗时触发服务启动超时
python -m mujocosim.apps.service --scene /path/to/mujoco/scenes/4ok3usBNeis_furnished.scene.json --name mujoco --enable-motion
# 客户端终端
python -m mujocosim.apps.motion --connect mujoco --output assets/runs/client
```

RSim 负责描述信息和共享数组、进程生命周期及命令租约；不修改其平台实现。`inspect` 仅供本地 viewer/调试在物理线程串行执行，不是远程控制 API。速度截止时间使用主机单调时钟，仿真加速不延长租约。

服务可用 `--hz` 设置发布频率（默认 50 Hz，周期须为物理步长 0.002 s 的整数倍）。曾出现的 DDS 命令确认超时已修复：RSim 的常规连接检查改为轻量状态查询，不再高频扫描共享文件目录。迁移时也需使用更新后的 RSim；命令 TTL、控制频率与共享文件机制保持原值。诊断记录见 PROJECT.md。

可选的真实跨进程验收会启动服务和物理 worker，依次运行独立客户端，并在结束后正常关闭服务：

```sh
python -m examples.check_dds_motion --scene /path/to/mujoco/scenes/simple_indoor.scene.json --rounds 3
python -m examples.check_dds_motion --scene /path/to/mujoco/scenes/4ok3usBNeis_furnished.scene.json --rounds 3
```

该检查保持 640×480 RGBD、360 束雷达和 50 Hz 发布，每轮前进、转向、后退、停止；需要 DDS 和离屏 OpenGL，结果保存在 `assets/runs/dds_motion/<run-id>`。它是本项目应用验收，不调用 RSim 的其他测试套件。

### WSL 内跨进程通信

服务端、客户端和 Notebook 内核应运行在同一个 WSL 发行版、同一用户下，并使用相同的服务名称、DDS domain 和 RSim 存储配置。这里的共享数组接口要求两端能访问同一共享目录。

WSL 的虚拟网络可能丢失较大 UDP 报文，表现为端点已发现、小消息可达，但位姿停更、图像读取或命令确认超时。本项目的 DDS 服务、运动客户端、导航服务以及 `NavigationClient` / `NavigationMap` 会在创建 DDS participant 前自动设置 `MaxMessageSize=1200 B`、`MaxRexmitMessageSize=1200 B`、`FragmentSize=1025 B`，使较大消息由 DDS 分片。参数含义见 [Cyclone DDS 配置文档](https://cyclonedds.io/docs/cyclonedds/latest/config/config_file_reference.html#cyclonedds-domain-general-maxmessagesize)。网卡与发现仍由 Cyclone DDS 自动选择。

这个默认配置仅在 WSL 且未设置 `CYCLONEDDS_URI` 时生效。已有自定义配置会原样保留；出现上述症状时，可将三个参数合并到自定义配置的 `Domain/General` 中，或在新终端中 `unset CYCLONEDDS_URI` 后使用项目默认值。更新后重新启动服务端和 Notebook 内核；已创建的 DDS participant 不会重新读取配置。

自编写的底层 RSim 客户端若绕过本项目应用，在每个独立进程的第一个 `Runtime` 之前调用同一配置入口：

```python
from rsim.transport.descriptor import TransportConfig
from mujocosim.integrations.transport import configure_transport

transport = configure_transport(TransportConfig('cyclonedds', 0))
# 将 transport 传入 SharedProvider / SharedComponent / Chassis 等组件。
```

## 点云建图、Notebook 导航与 pygame 控制

此示例额外需要兼容的 **graphmap、pyrobo 源码库**可导入；scipykit 仅提供可选绘图样式。graphmap 必须有 `Environment(enable_2d_map=False)` 和 `adapters.rgbd_observation`，pyrobo 必须有 `Navigator`、`RRTConnectConfig`、`VoronoiConfig`、`DWAConfig`。不要用不相关的同名 PyPI 包代替。按各库说明安装到同一环境，再安装 Notebook/pygame 工具（已有依赖无需重装）：

```sh
uv pip install -e ".[server,apps,notebook]"
# 终端 1：保持仿真运行；--keyboard 打开 RSim 原有 pygame 控制界面
python -m mujocosim.apps.navigation --scene /path/to/mujoco/scenes/simple_indoor.scene.json --enable-motion --keyboard
# 终端 2：从项目根目录启动，打开 Notebook
python -m jupyterlab examples/navigation.ipynb
```

选择安装了上述依赖的 Python 内核。Notebook 连接已有服务，内核进程与物理 worker 不同，不会另建世界。两端默认 `name=navigation`、DDS domain 0；更改服务端 `--name` / `--domain` 后同步修改 Notebook 的 `NAME` / `DOMAIN`，也可通过 `MUJOCOSIM_NAV_NAME` / `MUJOCOSIM_NAV_DOMAIN` 环境变量设置。当前跨进程接口限同一主机、同一用户和 RSim 存储配置。

场景可换成 `/path/to/mujoco/scenes/4ok3usBNeis_furnished.scene.json` 或其他已准备场景。`--headless` 隐藏 MuJoCo 窗口；`--planner voronoi` 切换全局规划器，默认 `rrt_connect`。`--bounds XMIN XMAX YMIN YMAX` 设置导航边界；省略时从首次点云范围推导，边界外不可导航。未传 `--enable-motion` 时仍建图，但拒绝运动。也可单独启动面板：

```sh
python -m mujocosim.apps.keyboard --name navigation --domain 0
```

pygame 使用 WASD，松键、失焦或输入过期时停车；空格取消导航并制动，Esc 关闭面板。面板空闲不会取消 Notebook 的任务；按方向键才抢占，原目标不会自动恢复。MuJoCo viewer 也可用 ↑↓←→，空格取消任务，R 复位并重建地图。导航应用的 viewer 不提供 demo 的 P 快照。服务退出时将点云和拓扑保存到 `<data-root>/maps/navigation/<run-id>/`，可用 `--map-output` 指定其他外部目录。`--output` 保存日志和 `ready.json` / `maps.json`，后者给出实际地图路径。每次运行独立保存，避免覆盖上次地图。

Notebook 中的核心接口也可以用于独立 Python 客户端：

```python
from rsim.runtime import Runtime
from rsim.transport.descriptor import TransportConfig
from mujocosim.integrations.navigation import NavigationClient, NavigationMap

transport = TransportConfig('cyclonedds', 0)
nav = NavigationClient('navigation', transport=transport)
maps = NavigationMap('navigation', transport=transport)
async with Runtime(nav, maps.map, maps.path, maps.pose):
    cloud = (await maps.map.get(timeout=15)).data  # world XYZ/RGB，只读数组
    action = await nav.go_to(-.8, 1.6, 1.5708)     # 世界坐标 m/m/rad
    result = await nav.wait_for_arrival(action, timeout=120)
    print(result)  # 检查 status == 'arrived'；也可能被手动抢占、取消或拒绝
    await nav.cancel()
```

该目标是规则房间的绕行示例，不能直接套用到其他场景。`await nav.navigate_to(x,y,yaw)` 合并发送与等待；`yaw=None` 时取从当前位置指向目标的角度。Jupyter 可跨单元格保留 Runtime，示例提供显式关闭单元格。关闭客户端或内核、停止续租、取消等待均会停止其任务；`client_lost` 表示租约中断，需要重新发目标。

`PointCloudMapper` 将 RGBD 光学 Z 深度和 360° 雷达通过仿真真值变换融合到 graphmap 三维点云（5 cm 采样），不启用二维地图，也不要求 ROS2、FAST-LIO、RTAB-Map 或 Open3D。导航选择 z=0.08–1.16 m 的障碍点并补充最新雷达，再按 XY 2 cm 间距选择原始 XYZ 点，减少竖直表面产生的重复路网节点；不生成占据/自由空间栅格。pyrobo 使用 0.6 m 方形足迹、0.10 m 点半径、RRT-Connect/Voronoi 全局路径及 DWA 局部控制。点半径必须覆盖两次采样的 XY 距离损失。导航组件不读取场景的 `navigation.npz`；该缓存仍用于原仿真的出生点检查。

这是已观测点云上的单层导航示例，**点云空白不代表已知可通行**。先手动观察目标和沿途空间，再发送坐标。低矮、细小或遮挡物可能漏观测。地图按静态环境累积，推动家具后需 R 复位或重启服务重建，当前不删除旧位置残影。目标在障碍内返回 `invalid_goal`；暂时无路时停车重试，可取消后重新选点。

导航应用默认 RGBD 3 Hz，与建图频率对齐；基本仿真源仍默认 5 Hz。速度上限 0.35 m/s、0.7 rad/s。规划结果保留采样时刻的 250 ms 命令期限，500 ms 客户端租约与其独立。负载高时可能间歇停车，不延长期限来维持运动。到达等待默认 180 s，可通过 API 的 `timeout` 或 Notebook 的 `GOAL_TIMEOUT` 调整；它只限制整次任务等待，与单条速度有效期不同。

可选验收使用真实独立 Jupyter kernel；`--pygame` 会打开真实窗口，用窗口进程内部 SDL 事件检验控制，不向其他桌面应用注入按键：

```sh
python -m examples.check_navigation --scene /path/to/mujoco/scenes/simple_indoor.scene.json --pygame
python -m examples.check_navigation --scene /path/to/mujoco/scenes/4ok3usBNeis_furnished.scene.json
```

轻量输出为 `assets/runs/navigation_checks/<run-id>`：执行后的 Notebook、点云/轨迹图、结果 JSON、pygame 截图、进程退出检查及日志。Notebook 的点云数组另存 `<data-root>/maps/notebook/<run-id>/navigation.npz`，位置写入 `result.json` 的 `map_file`；可用 `MUJOCOSIM_NOTEBOOK_MAP_OUTPUT` 指定其他外部目录。`--target X Y YAW` 改目标，`--goal-timeout` 改整次任务的等待秒数，`--planner voronoi` 验收另一个规划器。它只运行本项目应用，不运行 RSim 的其他测试套件。

## 从图像像素导航与生成航点拓扑

同一个导航服务内已包含 `PixelTarget` 组件。完整交互示例为 [examples/pixel_navigation.ipynb](examples/pixel_navigation.ipynb)：显示 RGBD，输入图中的像素，连续到达两个可达位置并画出实际航点拓扑。

```python
from rsim.runtime import Runtime
from rsim.transport.descriptor import TransportConfig
from mujocosim.integrations.navigation import NavigationClient, NavigationMap

transport = TransportConfig('cyclonedds', 0)
nav = NavigationClient('navigation', transport=transport)
maps = NavigationMap('navigation', transport=transport)
async with Runtime(nav, maps.map, maps.observation, maps.topology):
    observation = await nav.observe(maps)  # 固定并返回这一帧的 rgb/depth/标定/位姿
    # 显示 observation['rgb']，从图中选择 (u, v) = (列, 行)。
    point = await nav.pixel_to_world(320, 312)
    target = await nav.resolve_pixel(320, 312)  # 查询，不移动
    print(point['world_point'], target['goal'])
    result = await nav.navigate_to_pixel(320, 312)
    print(result['status'])                    # 到达为 arrived
    topology = await nav.get_topology()
```

`world_point` 保留像素对应的真实世界三维点，包括桌面或墙面高度。`goal=[x,y,yaw]` 是地面上的可达车体中心，朝向观测点；两者通常不同。反投影使用 graphmap `CameraIntrinsic`、`Pose` 与原始光学 Z 深度，不使用机器人查询时的新位姿。浮点像素取最近像素，越界、无效深度不会被补值。

`observe(maps)` 或 `capture()` 固定的帧会成为该客户端后续像素调用的默认帧；首次未指定帧的查询也会自动固定一帧。换画面时再次 `observe`，旧帧可显式传 `frame_id`。服务最多保留 16 帧，淘汰或仿真复位后明确报错。独立客户端各自记住选择的帧，不能把随时间更新的画面与另一个时刻的深度/位姿混用。

可达搜索使用观测障碍点和独立的 MuJoCo 物理模型：检查静态地面盒体的联合覆盖、车体净空、从当前位置连续连通的路线，不改变真实机器人位姿，也不读 `navigation.npz`。目标附近候选点间隔默认 5 cm，全局连续点路网间距约 28 cm；返回采样集合中最近的已验证目标及路径，`search.globally_optimal=False`，不保证连续空间的精确全局最优。车体使用能包住 0.6 m 方车任意朝向的圆形包络和余量，因此可能拒绝实际可挤过的窄通道。当前支持世界 z=0 的轴对齐有限盒体平地，不支持楼梯/多层导航。参数位于 `PixelTargetConfig`。

路线内部另保留 6 cm 跟踪余量，在起终点附近渐变收窄，避免生成刚好贴障碍的切线路径；目标的基本净空约束保持一致。像素目标按验证拐点使用 pyrobo DWA 控制，并按当前动态物体姿态复查预测轨迹和制动段；路线失效或持续无进展时停止并返回 `route_blocked`。`resolve_pixel` 在独立线程中求解，不阻塞手动抢占和续租；完成查询本身不会启动运动。若查询后机器人已移动过远、复位或任务被取消，旧结果不能启动导航，需重新求解。普通 `go_to(x,y,yaw)` 保留原 RRT-Connect/Voronoi 行为。

拓扑订阅为 `maps.topology`，或调用 `get_topology()`：`nodes` 含世界位姿 `pose=[x,y,yaw]` 和到达时的像素观测，`edges` 含实际走过的真值轨迹 `trajectory=[[x,y,yaw],...]`、方向和长度，单位 m/rad，地面 z=0。默认每 0.8 m、明显转弯和像素目标到达时产生航点，附近节点可合并。仅计划的路径不会创建边；缺帧、跳变和 reset 不会跨段连边。手动运动也参与轨迹记录，复位开始新图。服务退出保存 `<map-output>/topology.json`，路径见轻量输出目录的 `maps.json`。

```sh
python -m examples.check_navigation --notebook pixel_navigation --scene /path/to/mujoco/scenes/simple_indoor.scene.json
python -m examples.check_navigation --notebook pixel_navigation --scene /path/to/mujoco/scenes/4ok3usBNeis_furnished.scene.json
# 可覆盖示例像素：--pixels 320 312 448 312
```

同进程可直接使用 `world.pixel_target.pixel_to_world(u,v)`、`await world.pixel_target.resolve_pixel(u,v)`；构建组件时注入 source、mapper、navigation，标定、帧和当前物理状态均从这些上下文取得。

## 数据、输出与边界

单位 m/s/rad，机器人 X 前、Y 左、Z 上。差速 demo 的 `base_footprint` 为名义地面中心；雷达位于 `(0,0,.4)`，相机位于 `(0,0,1)`。光学相机 X 右、Y 下、Z 前。RGB 为 uint8 RGB；深度为 float32 米制光学 Z，范围外 NaN；雷达无回波为 +inf。默认 640×480、360 束。

RSim pose/truth 是世界真值；odom 来自真实轮关节转角，imu 使用 MuJoCo gyro/accelerometer。state 的 `linear_x/angular_z` 是 base_footprint 中的真值前向速度/偏航速度，原 `linear_velocity/angular_velocity` 保留别名。`acceleration/yaw_acceleration` 使用相邻物理样本求导；`frame/episode/step_index/pose_kind` 与 Frame 元数据一致。`collided` 标识平地场景中机器人非竖直支撑接触，不把正常车轮触地计为撞障。图像和数组只读。新工厂的 RSim Frame 时间戳与 `physics_time`、传感器 header 均来自本次 episode 的原生物理时间；不同 episode 的时间戳不可直接相减。只有过渡 `create_source` 的 `legacy_continuous` 模式保留跨 reset 连续偏移。

RSim 元数据中的小型变换矩阵和相机内参以不可变嵌套 tuple 保存，数值处理时使用 `np.asarray(...)`；大尺寸图像、点云与雷达数组保留数组表示。

输出包括 `rgb.png`、`depth_m.npy`、`sensors.npz`、`metadata.json`、`trajectory.csv`、`lidar_history.npz`、`diagnostic.png`、`robot_overview.png`、`run_summary.json`。同目录运行会覆盖同名结果。`world.xml` 是显式导出文件，不再作为运行时共享中间文件。

扫描碰撞仍是选定楼层的平面/表面体素近似；`collision_height` 默认 1.16 m，可在 JSON 或 prepare_scene API 调整。尚不支持楼梯或完整多层导航。物体显示网格减面、颜色采样，物理参数未做实物标定。导航图记录物体初始占地，移动后不在线更新；碰撞和传感器随当前姿态更新。geom group 3 可检查代理，group 2 控制上部场景剖视。

缓存是生成数据，无并发预处理锁；同一个未生成缓存应先单独准备。迁移数据源后重建缓存即可；GLTF 外部纹理变更需要 `--rebuild`。旧 artifacts 目录仅为历史结果。

## 验证范围

```sh
# 无窗口 Linux 的真实物理、RGBD、工厂、命令安全、共享进程与旧路径回归
MUJOCO_GL=egl python -m pytest tests -q
# 明确选取两个独立解释器，证明 provider 无 pyrobo、专有客户端无 mujoco
python tests/check_environments.py --provider-python /path/to/provider/bin/python \
  --client-python /path/to/client/bin/python --output assets/environment-check --scheduling controlled
```

跨环境测试会启动显式 Provider，连接同一个 World，验证 reset 完成回执、新 episode、历史隔离和实际 worker 解释器；controlled 模式另验证暂存、推进、查询、过期重试、暂停/恢复。全量测试在装有 pyrobo 时还实际运行 1/2/10 倍率的同一环境导航控制器，验证到达、停止和无碰撞；无 pyrobo 的 engine/provider 环境明确跳过这项可选应用测试。真实设备/ROS2 硬件、GUI 交互、跨主机控制和跨进程严格锁步不在此提供端测试范围。开发机配置、实际运行结果、兼容组合与阶段未完成项记录在本地 `PROJECT.md`。

## 扩展场景和传感器

普通 GLB 可用 `prepare_scene(path, collision_scope="scene")` 生成全高度碰撞体。默认的 `"level"` 使用选定地面附近的碰撞高度。两种模式都生成一个平面的出生点和净空网格。自定义场景构造器可将已解码的 `trimesh.Scene` 交给 `mujocosim.scenes.write_mesh_scene(scene, directory, ...)`，得到 MJCF、纹理、导航栅格和包含 `world_from_source` 的几何元数据。构造器负责数据源、缓存和写入最终的 `metadata.json`。整目录部署时，RSim 工厂按 MJCF、元数据、导航栅格及运行时纹理/网格计算指纹，源文件路径仅用于溯源。

第三方机器人可继承 `DifferentialDrive` 并扩展 `build(root)`。传感器可继承 `SensorConfig`，覆盖 `attach(body)` 来安装 MJCF 传感器，或在 `super().attach(body)` 后调整标准安装架。将这两个实例传给 `MujocoChassisFactory(robot=..., sensors=...)` 即可使用原有底盘和 RGB-D 端口。
