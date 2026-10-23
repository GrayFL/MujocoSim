# MujocoSim

基于 MuJoCo 的机器人、传感器与场景仿真工具，通过可选的 RSim 集成提供 Signal / CommandSink 与设备接口。目前 demo 是 0.6 m × 0.6 m 差速机器人、二维雷达、前向 RGBD，以及 HM3D 场景和可推动的桌椅。

**仿真实现全部位于本项目。** RSim 只提供协议适配、组件运行与设备接口。项目结构和扩展方法见 [ARCHITECTURE.md](ARCHITECTURE.md)，本机路径、开发约束与验证记录见 [PROJECT.md](PROJECT.md)。

## 安装

使用 Python ≥ 3.12 和已有 Conda/venv 环境。在本项目根目录安装：

```sh
# RSim 源码路径按实际位置调整，必须包含新的注入式 MuJoCo adapter
python -m pip install -e ../RSim
python -m pip install -e ".[rsim]"
# 可选：预览窗口 / DDS 跨进程 / 测试
python -m pip install -e ".[preview,dds,test]"
```

仅使用同步物理 API 时 `pip install -e .` 即可，不要求 RSim。依赖包括 MuJoCo、NumPy、SciPy、trimesh、Pillow、Matplotlib、fast-simplification；RSim 集成另外需要兼容的 graphmap.pose.Pose。可选 scipykit 绘图样式不可用时回退 Matplotlib。不要仅凭 RSim 的版本号使用旧发行包：adapter 必须提供 `SimulationBackend`、`Observation`、`CommandSpec`。

RGBD 和预览需要 OpenGL。`--headless` 关闭交互窗口，仍会离屏渲染最终图像。macOS 被动 viewer 使用 `mjpython` 启动。原始数据单独准备，不随仓库分发。

## 运行 demo

```sh
python -m mujocosim --scene data/scene.glb --preview
python -m mujocosim --scene data/scene.glb --headless --mode demo --seconds 12 --output assets/runs/demo
python -m mujocosim.apps.motion --scene data/scene.glb --output assets/runs/motion
```

`run_sim.py` 与 `python -m mujocosim` 等价；安装后也可运行 `mujocosim`。旧的 `python -m mujoco_diffdrive` 保留为本地兼容入口。直接运行 `python -m mujocosim.apps.demo` 时显式传入场景。

| 参数 | 用途 |
|---|---|
| `--scene` | GLB/GLTF、组合 JSON 或已转换目录 |
| `--up-axis auto/y/z`、`--floor-height H` | GLB/GLTF 朝上轴、源坐标中的楼层高度 |
| `--resolution R` | GLB/GLTF 碰撞体素尺寸，默认 0.08 m |
| `--spawn X Y YAW` | 世界坐标中的 m / m / rad |
| `--cache-dir`、`--output` | 默认 `assets/cache`、`assets/runs/latest` |
| `--prepare-only`、`--rebuild` | 仅准备场景、强制重建 |
| `--mode teleop/demo/idle` | 键盘控制、避障演示、静止 |
| `--seconds`、`--record-every` | 仿真时长、快照间隔 |
| `--width`、`--height`、`--preview` | 相机分辨率、实时图像窗口 |

W/S 前后，A/D 转向，空格停止，R 重置机器人及动态物体，P 快照，Q 退出。切换至对应窗口使其获得键盘焦点。demo 使用真值、静态净空和雷达，是功能演示，不是 SLAM 或全局导航。

## 场景与资产

```text
<scene-root>/
  test/chair.glb
  test/table.glb
  hm3d/4ok3usBNeis.glb
  scenes/simple_indoor.scene.json
  scenes/4ok3usBNeis_furnished.scene.json
assets/
  templates/       # 版本管理的布景配置
  cache/           # OBJ/PNG、场景 MJCF、导航数据与 scene_index.json
  runs/            # 传感器、轨迹、预览、物理检查
  inspection/      # 原始资产检查图
```

准备两个物体后，安装模板并构建：

```sh
python -m examples.build_scenes --scene-root data/mujoco --hm3d data/4ok3usBNeis.glb
python -m mujocosim --scene data/mujoco/scenes/simple_indoor.scene.json --preview
python -m mujocosim --scene data/mujoco/scenes/4ok3usBNeis_furnished.scene.json --preview
python -m examples.check_scenes --output assets/runs/scene_checks
```

构建脚本保留已安装 JSON；`--refresh-recipes` 明确覆盖为仓库模板，`--rebuild` 重建缓存，`--only simple_indoor` 仅构建规则房间。JSON 的源资产路径相对 JSON 本身；新格式为 `mujocosim-scene-v1`，兼容旧 `rsim-scene-v1`。组合的朝上轴、楼层、分辨率与碰撞高度以 JSON 为准。

`MUJOCO_SCENE_ROOT` 指定默认数据根目录；省略 `--scene` 时加载其下的 `scenes/4ok3usBNeis_furnished.scene.json`。迁移后设置该变量或显式传路径；文中的 `data/` 是示例。相对输出路径以工作目录为准。场景索引中的 prepared 路径相对索引目录，移动项目后可重跑构建脚本更新索引，不需要编辑缓存文件。

规则房间大小 6 × 5 × 2.6 m。圆桌保持约 1 × 1 × 0.742 m，质量假设 18 kg；椅子等比缩至 0.763 × 0.830 × 0.9 m，质量假设 9 kg。两者有自由刚体、摩擦、分部碰撞，可推动或倾倒。JSON 支持 `position`、`yaw`、`scale`、`target_height`、`mass`、`friction`、`colliders`；尺寸为米、角度为弧度。collider 坐标先转 Z-up、底部归零、XY 居中，再随物体统一缩放；box 使用半尺寸，capsule 使用半径和 fromto。

## 同步物理 API

```python
from mujocosim import Simulation, SensorConfig, PhysicsConfig, prepare_scene
from mujocosim.robots import DifferentialDrive, DriveCommand

scene = prepare_scene("data/scene.glb")
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
from mujocosim.integrations.rsim import create_source
source = create_source("data/scene.glb", motion_enabled=True)
```

这里的 `create_source` 是本项目的差速 demo 组装。它把本地轮编码器、IMU、图像和雷达转换为 RSim 端口，再注入 RSim adapter。业务应用仍使用 `rsim.devices.Chassis`，完整组装位于 `mujocosim/apps/demo.py`、`motion.py`。底层 `rsim.drivers.MuJoCo` 现在接受后端工厂，**不再接受 scene 并自动构造差速机器人**。

跨进程保持相同应用接口（安装 DDS extra，同机、同一用户）：

```sh
# 提供者终端；先构建场景，避免首次转换耗时触发服务启动超时
python -m mujocosim.apps.service --scene data/mujoco/scenes/4ok3usBNeis_furnished.scene.json --cache-dir assets/cache --name mujoco --enable-motion
# 客户端终端
python -m mujocosim.apps.motion --connect mujoco --output assets/runs/client
```

RSim 负责描述信息和共享数组、进程生命周期及命令租约；不修改其平台实现。`inspect` 仅供本地 viewer/调试在物理线程串行执行，不是远程控制 API。速度截止时间使用主机单调时钟，仿真加速不延长租约。

服务可用 `--hz` 设置发布频率（默认 50 Hz，周期须为物理步长 0.002 s 的整数倍）。本轮本地运动验收通过，但 DDS 完整运动试跑出现命令确认超时，尚未定位；25 Hz 也未完整通过，降低频率不能视为修复。跨进程运行目前有此限制，诊断记录见 PROJECT.md。

## 数据、输出与边界

单位 m/s/rad，机器人 X 前、Y 左、Z 上。差速 demo 的 `base_footprint` 为名义地面中心；雷达位于 `(0,0,.4)`，相机位于 `(0,0,1)`。光学相机 X 右、Y 下、Z 前。RGB 为 uint8 RGB；深度为 float32 米制光学 Z，范围外 NaN；雷达无回波为 +inf。默认 640×480、360 束。

RSim pose/truth 是世界真值；odom 来自真实轮关节转角，imu 使用 MuJoCo gyro/accelerometer。图像和数组只读。RSim Frame 时间戳跨 reset 单调，数据中的 physics_time 和传感器 header 使用本次 episode 的物理时间；不同 episode 的 header 不应直接相减。

RSim 元数据中的小型变换矩阵和相机内参以不可变嵌套 tuple 保存，数值处理时使用 `np.asarray(...)`；大尺寸图像、点云与雷达数组保留数组表示。

输出包括 `rgb.png`、`depth_m.npy`、`sensors.npz`、`metadata.json`、`trajectory.csv`、`lidar_history.npz`、`diagnostic.png`、`robot_overview.png`、`run_summary.json`。同目录运行会覆盖同名结果。`world.xml` 是显式导出文件，不再作为运行时共享中间文件。

扫描碰撞仍是选定楼层的平面/表面体素近似；`collision_height` 默认 1.16 m，可在 JSON 或 prepare_scene API 调整。尚不支持楼梯或完整多层导航。物体显示网格减面、颜色采样，物理参数未做实物标定。导航图记录物体初始占地，移动后不在线更新；碰撞和传感器随当前姿态更新。geom group 3 可检查代理，group 2 控制上部场景剖视。

缓存是生成数据，无并发预处理锁；同一个未生成缓存应先单独准备。迁移数据源后重建缓存即可；GLTF 外部纹理变更需要 `--rebuild`。旧 artifacts 目录仅为历史结果。

## 验证范围

```sh
# 本项目：物理、场景、传感器、扩展接口、本地 RSim 绑定
python -m pytest tests -q
# 在 RSim 根目录：只验证 MuJoCo adapter
python -m pytest tests/test_mujoco_adapter.py -q
```

不运行 RSim 的平台、共享存储、ROS、通用设备或控制器测试套件。真实场景检查用 `examples.check_scenes`；迁移安装或接口变更后再运行相关本地 demo。
