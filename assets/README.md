# 轻量输出目录

- `runs/`：可视化、日志、结果 JSON、小型传感器快照与临时实验输出，Git 忽略。
- `inspection/`：源模型尺寸、朝向与场地检查图，Git 忽略。

所有场景 JSON、布景模板、模型资产、转换缓存和完整地图均放在 `MUJOCO_SCENE_ROOT` 指定的外部数据目录，本机为 `D:/Storage/Datasets/mujoco`。对应子目录为 `scenes/`、`templates/`、`test/`、`hm3d/`、`cache/`、`maps/`。

导航服务在这里保存 `ready.json` / `maps.json`，记录外部点云与拓扑的位置。已迁移的历史实验通过 `data_locations.json` 查找数据，新 Notebook 的 `result.json` 记录 `map_file`。迁移清单见 `runs/data_migration/manifest.json`。

通用使用方法见 [README](../README.md)，本机开发上下文见 [PROJECT](../PROJECT.md)。
