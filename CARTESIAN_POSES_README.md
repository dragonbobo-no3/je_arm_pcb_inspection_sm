# 修改Pick和Place坐标说明

## 快速修改坐标（无需重新编译）

### 📂 文件位置
- **Pick坐标**：`config/move_group_client/cartesian_states/pick.yaml`
- **Place坐标**：`config/move_group_client/cartesian_states/place.yaml`

### 📝 YAML格式

每个文件包含以下字段：
```yaml
x: <X坐标, 单位: 米>
y: <Y坐标, 单位: 米>
z: <Z坐标, 单位: 米>

qx: <四元数X分量>
qy: <四元数Y分量>
qz: <四元数Z分量>
qw: <四元数W分量>

frame_id: "base_link"  # 参考坐标系
```

### 🔄 修改流程

**第1步** - 编辑YAML文件：
```bash
# 编辑Pick位置
nano src/SMACC2/smacc2_sm_reference_library/my_robot_arm_sm/config/move_group_client/cartesian_states/pick.yaml

# 编辑Place位置
nano src/SMACC2/smacc2_sm_reference_library/my_robot_arm_sm/config/move_group_client/cartesian_states/place.yaml
```

**第2步** - 重新启动节点（无需编译）：
```bash
# 关闭当前运行
pkill -f my_robot_arm_sm_node

# 重新启动
source ~/ros2_ws/install/setup.bash
ros2 launch my_robot_arm_sm my_robot_arm_sm.launch.py
```

### 📐 四元数说明

常见的四元数值：

| 方向 | qx | qy | qz | qw | 说明 |
|------|----|----|----|----|------|
| 0, 0, 0, 1 | 0.0 | 0.0 | 0.0 | 1.0 | 默认/水平 |
| 向下 | 0.0 | 0.707 | 0.0 | 0.707 | 与Y轴成90° |
| 向上 | 0.0 | -0.707 | 0.0 | 0.707 | 与Y轴成-90° |
| 向前 | 0.707 | 0.0 | 0.0 | 0.707 | 绕X轴旋转 |

可以在RViz中用交互式控制器测量实际方向，然后填入对应的四元数。

### 🎯 当前数据源

当前数据来自 `calibration_points_example.json`：
- **Pick (R0_D02)**：X=-0.273, Y≈0, Z=0.412
- **Place (R0_D05)**：X=-0.367, Y=0.331, Z=-0.009

