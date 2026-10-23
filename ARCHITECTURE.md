# 结构与扩展约定

## 归属和依赖方向

MujocoSim 拥有仿真世界的具体内容，RSim 拥有组件和协议机制。场景、机器人、传感器彼此组合；RSim 由应用层接入，不作为物理实现的宿主。

```text
mujocosim/
  simulation.py             # 组装、步进、复位、资源释放
  model.py                  # scene + robot + 可选 sensor rig -> 内存 MJCF
  config.py                 # PhysicsConfig / SensorConfig
  deployment.py             # 数据根目录和本地默认值
  robots/
    base.py                 # Robot / RobotState 小接口
    differential_drive.py   # 当前构型、轮速控制、编码器与 IMU
  sensors/
    rig.py                  # 当前雷达/RGBD 安装几何
    suite.py                # LaserScan / RGBDFrame 与采样
  scenes/
    prepare.py              # SceneBuilder 与格式分派
    mesh.py                 # GLB/GLTF，含 HM3D，选层/体素碰撞
    composition.py          # 房间或扫描场景 + 动态物体
    objects.py              # 物体定向、缩放、减面、显示与碰撞
  integrations/rsim.py       # 本项目 demo 的端口、消息和命令映射
  apps/                     # demo / motion / service / recording / plotting
assets/templates/           # 可版本管理的布景配置
examples/                   # 构建与验证项目场景、设备 API 示例
mujoco_diffdrive/            # 单向兼容转发；新功能不放这里
```

核心的 simulation、robots、sensors、scenes 不依赖 RSim，也不回调兼容包。integrations 和 apps 可以依赖两者。RSim adapter 不导入 MujocoSim；项目通过工厂注入后端。保留直接 Python 组合，不引入插件扫描器、全局注册中心、通用实体树或复杂配置继承。

## 机器人接口

`Robot.build(root)` 向 MJCF 添加 body/joint/actuator/sensor，返回传感器安装 body；`bind(model,data)` 得到运行时 RobotState。模型构建与编译后的索引绑定分开，避免把关节顺序假设写进引擎。

RobotState 负责 reset、apply、stop、after_reset、after_step 和 telemetry；body_id 用于查询根刚体位姿。apply 的命令类型由构型决定。差速机器人使用 DriveCommand，轮半径、轮距、限速、里程计积分全在 differential_drive.py。引擎只调用 apply，不解释轮速或机械臂关节。

新增构型时：在 robots 中实现上述接口，用名字绑定关节和执行器；将实例传给 Simulation。若接 RSim，在本项目 integrations 中选择该构型的端口和命令格式。不要往 RSim adapter 增加 robot_type 分支。`tests/test_extensibility.py` 用单轴滑台、标量力命令和无传感器配置实际验证了该入口。

当前 spawn 为单层地面上的 `[x,y,yaw]`，Robot.clearance_radius 决定出生点净空；机械臂等固定模型可自行忽略无关分量。需要空中/多楼层任意 SE(3) 初态时再扩展 spawn 契约，不把当前平地契约声称为已支持全部机器人。

## 场景接口

场景准备产物保持简单：目录内含 `scene.xml`（asset/worldbody）、`metadata.json`（spawn、world_from_source 等）和 `navigation.npz`（floor、obstacles、clearance、origin、resolution）。MuJoCo 引擎只消费这个结果，不区分 HM3D 与手工场景。

`prepare_scene` 支持 GLB/GLTF、组合 JSON、已有产物目录和自定义 `SceneBuilder.prepare(cache_root, **options)`。新增输入格式时实现独立 builder，生成同一产物；需要 CLI 后缀支持时在入口增加显式分派。未来接原生 MJCF/USD 或其他数据集，转换和地面/导航适配都在 scenes，不进入 RSim。当前没有宣称已支持这些格式。

场景不按固定 0.6 m 机器人拒绝准备；机器人净空校验在 Simulation.reset。扫描碰撞高度是场景参数，修改机器人高度时需相应配置。JSON 组合使用初始物体占地检查放置冲突，真实物体运动交给 MuJoCo。

## 传感器和运行器

传感器采样与 MJCF 构型分开，默认 rig 是当前 demo 的线雷达/RGBD。新传感器在 sensors 中实现安装、采样与数据结构，再由本地运行/集成组装；本次没有为每种传感器建立插件层。支持无 rig 的 Simulation，未安装的传感器调用报明确错误。

Simulation 的 command 只转交 RobotState。step 推进 MuJoCo，再更新机器人内部测量状态并刷新正向运动学。reset 恢复全场景，包括动态家具，再让机器人执行其初态设置。核心不负责异步调度；离屏 GL 必须在使用它的线程创建和释放。运行时直接编译内存 XML，显式 export_model 才写 world.xml。

## RSim 协议边界

RSim 的 `SimulationBackend` 只有 open、step(duration,commands)、sample(demand,force)、reset、stop、close。Observation 描述物理时间、输出值和元数据；输出名、容量和 CommandSpec 在构造 adapter 时显式给出。CommandSpec 指定安全回退、验证和安全值判断，不限定为 VelocityCommand。

adapter 负责一个工作线程、RSim 需求快照、Signal 发布、独立命令截止时间、motion_enabled 和跨 reset 单调时钟。后端的所有物理/GL 方法在同一线程执行。后端输出拥有独立快照，发布后不可修改；端口类型的语义与 readonly 数组由项目绑定负责。

本地 DifferentialDriveBackend 负责生成 world、转换 DriveCommand、映射 Pose/odom/imu/scan/image/depth 等数据。SimulationSource 只是 viewer/检查工具访问本地 Simulation 的便捷层。业务 demo 继续通过 devices.Chassis 操作，其参数和服务组装放在本地 apps。

RSim `examples/mujoco_adapter.py` 是独立的单关节接入教程，用来说明如何自己实现协议；它不导入本项目或包含房间/桌椅/差速车。旧 rsim.apps.mujoco* 已移除，不能反向依赖本项目做隐藏转发。

## 验证与后续扩展

每次按改动跑本项目相关测试，必要时跑 RSim 的 test_mujoco_adapter。不得把平台、传输、共享存储或通用控制器回归并入这里的测试范围。应用 demo 是对本项目接入链的验收，不替代 RSim 自己的库验证。

优先顺序：有第二个实际构型时添加 Robot；有第二种实际场景格式时添加 SceneBuilder；有新的传感器时扩展本地 sensors 和端口绑定。只有两种实现确实出现共同逻辑时再抽取基类或配置模式。保持现有默认 demo 和物理数据可复现，不为了未来假想接口扩大抽象层数。
