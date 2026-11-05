# `fusion_framework.launch` 与 `radar_livox_fusion_node` 逻辑说明

本文依据以下两个文件整理：

- `src/FAST_LIO-main/launch/fusion_framework.launch`
- `src/FAST_LIO-main/src/radar_livox_fusion_node.cpp`

配置文件为 `src/FAST_LIO-main/config/fusion_framework.yaml`。

## 1. launch 文件做了什么

```xml
<arg name="config" default="$(find fast_lio)/config/fusion_framework.yaml" />
<rosparam command="load" file="$(arg config)" ns="radar_livox_fusion_node" />
<node pkg="fast_lio" type="radar_livox_fusion_node"
      name="radar_livox_fusion_node" output="screen" />
```

启动过程只有两步：

1. 声明 `config` 参数，默认指向 `fast_lio/config/fusion_framework.yaml`，也可以在 `roslaunch` 时用 `config:=...` 替换。
2. 将 YAML 加载到私有命名空间 `radar_livox_fusion_node`，再启动同名可执行文件。

节点构造函数使用私有 `ros::NodeHandle nh("~")` 读取参数，因此 YAML 中的 `input/...`、`sync/...`、`extrinsic/...` 等路径正好对应节点的参数名。launch 本身不进行同步或融合，也没有启动 Livox、雷达驱动；这两个传感器驱动必须另行运行并发布对应话题。

## 2. 话题和主要配置

当前 YAML 的默认连接关系如下：

| 作用 | 话题/参数 | 类型或含义 |
|---|---|---|
| Livox 输入 | `/livox/lidar` | `livox_ros_driver/CustomMsg` |
| 毫米波雷达输入 | `/radar_enhanced_pcl` | `sensor_msgs/PointCloud` |
| 未融合的 Livox 输出 | `/livox/lidar_pcl2_raw` | `sensor_msgs/PointCloud2`，含 XYZ 和反射率 |
| 融合输出 | `/fusion/points` | `sensor_msgs/PointCloud2`，含 `x,y,z,intensity,time,ring` |
| 输出坐标系 | `frame_id: livox` | 融合结果统一标为 Livox 坐标系 |
| 时间匹配阈值 | `max_sync_dt: 0.06 s` | 雷达与 Livox 时间差超过该值则不融合 |
| Livox 延迟 | `livox_delay: 0.12 s` | 收到 Livox 后等待一段墙钟时间再处理 |
| 雷达过期窗口 | `stale_time: 0.20 s` | 比当前 Livox 早太多的雷达帧丢弃 |
| 雷达 ring | `31` | 与 Livox 的 ring 0--5 区分 |
| 外参 | `mmwave_to_livox_t/r` | 雷达到 Livox 的平移和旋转，默认单位阵/零平移 |

`filter/is_filter` 当前配置为 `false`，所以默认不启用 1--120 m 的距离裁剪；若设为 `true`，Livox 和雷达都会应用 `min_range`、`max_range`。`filter/voxel_leaf` 虽然被读入成员变量，但当前源码没有用它进行体素降采样。

## 3. 节点总体数据流

```text
Livox CustomMsg ──回调──> Livox 队列 ──定时器(等待 livox_delay)──┐
                                                               ├─> 选取最近雷达帧
毫米波 PointCloud ─回调──> 雷达队列 ────────────────────────────┘
      │
      ├─> 发布原始 Livox PointCloud2
      ├─> Livox 点解析、抽样、距离检查、质量统计
      ├─> 雷达点解析、Power 筛选、外参变换
      ├─> 烟雾状态机决定两类点的保留规则
      └─> 按点时间排序，发布融合 PointCloud2
```

节点由两个订阅回调和一个 `ros::WallTimer` 驱动。回调只负责加锁后把消息放入队列，实际处理集中在定时器中，避免 Livox 到达过早时雷达尚未到达。

## 4. Livox 点云处理

每个 Livox 帧的处理顺序是：

1. `buildLivoxRawCloud` 遍历全部 `point_num` 个点，复制 `x/y/z`，并把 Livox `reflectivity` 写入 `intensity`，立即发布到 `/livox/lidar_pcl2_raw`。这一路不做有效点筛选。
2. `buildLivoxCandidates` 从索引 `1` 开始遍历（第 0 个点被跳过），只保留 `line < scan_line` 的点，并要求 `tag` 的质量位满足 `0x10` 或 `0x00`。
3. 对通过质量检查的点按 `point_filter_num` 做计数抽样；默认值 2，即每两个有效点保留一个。
4. 保存 Livox 原有的每点时间 `offset_time * 1e-9`（秒）和线号 `line`，再按距离过滤（仅当 `is_filter=true`）。
5. 同时统计烟雾检测所需的有效点数、远距离点数、远距离低反射率点数和远距离体素分布。

Livox 点最终保留原坐标、反射率、相对时间和 ring，因此可以被 FAST-LIO 按带时间的点云使用。

## 5. 雷达与 Livox 的时间同步

### 队列维护

- 雷达回调把 `sensor_msgs::PointCloud` 放入雷达队列；时间戳回退时清空队列，队列超过 `radar_queue_size` 时从队首丢弃。
- Livox 回调把消息和接收墙钟时间放入 Livox 队列；时间戳回退或队列溢出时采取同样的保护措施。

### 延迟处理

定时器周期默认为 5 ms。它检查 Livox 队首消息在墙钟上是否已经等待至少 `livox_delay`（默认 120 ms），未达到则继续等待；达到后取出并处理。因此这里是“可配置延迟 + 最近邻匹配”，不是 ROS `message_filters` 的严格同步。

### 选择雷达帧

`takeBestMmwave` 以 Livox 帧时间戳为基准：

1. 先删除早于 `livox_stamp - stale_time` 的雷达帧。
2. 在剩余队列中寻找绝对时间差最小的帧。
3. 若最小差值大于 `max_sync_dt`，本帧放弃雷达，继续发布 Livox-only；否则取出该雷达帧（一个雷达帧只匹配一次）。

若队列为空、雷达过期或没有满足时间阈值的帧，融合流程不会阻塞，而是退化为仅发布 Livox 点云。

## 6. 雷达点处理与坐标融合

雷达输入是 `sensor_msgs/PointCloud`。节点复制每个点的 XYZ，并查找名为 `Power` 或 `intensity` 的 channel 作为点强度；没有该 channel 时强度默认为 1。雷达点没有逐点时间，因此整帧使用同一个相对时间：

```text
radar_point_time = clamp(radar_stamp - livox_stamp, 0, livox_scan_duration)
```

Livox 扫描时长取该帧所有保留点的最大 `time`；若无法得到则使用配置的 `scan_duration`（默认 0.1 s）。

雷达点先进行距离检查（仅在 `is_filter=true` 时），再根据烟雾模式筛选 Power 和数量。保留下来的每个点执行刚体变换：

```text
p_livox = R_mmwave_to_livox * p_mmwave + t_mmwave_to_livox
```

配置中的旋转矩阵按行展开为 9 个数。默认外参是单位阵和零平移，因此只有在两个传感器坐标轴已经对齐时才可直接使用默认值。雷达点的 `ring` 固定为 31，Livox 点为 0--5，便于下游区分数据来源。

这里的“融合”是前端点云级拼接：节点不会进行点到点关联、目标跟踪、Doppler 速度融合或 EKF 更新，而是把筛选后的 Livox 点和变换后的雷达点放入同一个 `PointCloud2`。

## 7. 烟雾自适应方法

YAML 开启了 `smoke_adaptive`。节点为每个 Livox 帧计算三个可解释的指标：

1. **有效点比例**：当前有效 Livox 点数除以正常状态下的慢变基线。比例低于 `min_valid_points_ratio=0.5` 时，低点数分数升高。
2. **远距离低反射率比例**：距离不小于 `far=20 m` 的点中，反射率低于 8.0 的比例。
3. **远距离孤立体素比例**：把远距离点按 0.5 m 体素分组，体素内点数不超过 2 的视为孤立体素。

三者归一化到 `[0,1]` 后按配置权重计算：

```text
smoke_score = 0.35 * low_valid_point_score
            + 0.35 * far_low_reflectivity_score
            + 0.30 * far_isolated_voxel_score
```

状态机有 `NORMAL`、`SMOKE_SUSPECTED`、`SMOKE` 三态，并用帧计数防抖：连续 5 帧分数达到 0.65 才进入 `SMOKE`；连续 15 帧分数不高于 0.35 才回到 `NORMAL`。阈值之间保持当前稳定状态；从正常状态开始积累时会短暂进入 `SMOKE_SUSPECTED`。

### 不同状态的点保留策略

- **NORMAL**：不删除 Livox 远点；雷达 Power 阈值为 7.0，最多保留 300 个最高 Power 点。
- **SMOKE_SUSPECTED**：远距离 Livox 点仅在“低反射率且孤立”时删除；雷达阈值取 7.0 和 5.5 的中值，数量上限取两种上限的中间值。
- **SMOKE**：远距离 Livox 点只要低反射率或孤立就删除（近距离点不按烟雾规则删除）；雷达阈值降到 5.5，最多保留 1000 个最高 Power 点。

因此正常环境优先保持 Livox 的精细几何，疑似或确认烟雾时抑制远处不稳定的 Livox 噪声，并让更多较弱但可能穿透烟雾的雷达回波进入融合结果。

## 8. 输出与边界行为

- 融合点按 `time` 稳定排序后发布，消息时间戳使用 Livox 帧时间戳，坐标系使用配置的 `frame_id`。
- 若 Livox 帧没有候选点，直接跳过，不发布融合帧。
- 若烟雾过滤后两类点全部被删除，也跳过发布并给出节流警告。
- 节点维护匹配、Livox-only、过期丢弃和队列溢出计数，并通过 ROS 日志周期性输出匹配状态。

## 9. 实现上的注意事项

1. 当前配置的 `filter/is_filter=false`，所以 `min_range/max_range` 实际不生效；需要距离裁剪时应显式打开它。
2. `filter/voxel_leaf` 在当前源码中只被读取，没有调用 PCL 体素滤波器，不能据此认为输出已降采样。
3. `near_range` 用于约束 `far_range` 的合法范围，但当前点过滤逻辑只判断“是否达到 far_range”，并未单独使用 near/far 分层。
4. 雷达没有逐点时间，整帧点被赋予同一相对时间；这适合当前 `sensor_msgs/PointCloud` 接口，但不表达雷达扫描内部的运动畸变。
5. 外参必须由实际标定结果填写。坐标变换只完成几何对齐，不会自动估计外参。

综上，该节点是一个带软时间同步、外参变换、质量自适应筛选和来源标记的前端点云拼接器：Livox 提供高密度精细几何，毫米波雷达在时间匹配后以补充点的形式加入，烟雾状态只改变两类点的筛选强度，并不改变 FAST-LIO 后端的状态估计算法。
