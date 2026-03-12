 <h2>Build Instructions</h2>

First, source your ros2 installation.
```
source /opt/ros/humble/setup.bash
```

Before you build, make sure you've installed all the dependencies...

```
rosdep install --ignore-src --from-paths src -y -r
```

Then build with colcon build...

```
colcon build
```
  <h2>Operating Instructions</h2>

Then source the proper workspace...

```
source ~/workspace/humble_ws/install/setup.sh
```

And then run the launch file...

```
ros2 launch sm_panda_cl_moveit2z_cb_inventory sm_panda_cl_moveit2z_cb_inventory.launch.py
```


 <h2>Viewer Instructions</h2>
If you have the SMACC2 Runtime Analyzer installed then type...

```
ros2 run smacc2_rta smacc2_rta
```

If you don't have the SMACC2 Runtime Analyzer click <a href="https://robosoft.ai/product-category/smacc2-runtime-analyzer/">here</a>

处理外部事件队列 → 执行状态转移 → 对当前激活状态调用 update()（如果该状态实现了）→ 再睡眠到下一周期

je_arm_pcb_inspection_sm_node.cpp
      └─ smacc2::run<SmJeArmPcbInspection>()
            │
            └─ SignalDetector 线程  ← 这就是"主循环"
                    │
                    以 20Hz (config.yaml 里 signal_detector_loop_freq) 轮询：
                    │  ① rclcpp::spin_some()      ← 处理 ROS 话题/服务
                    │  ② 检查 pending events 队列  ← CB success/failure/键盘等
                    │  ③ 触发 Boost.Statechart 状态转移
                    │  ④ 调 update() (如果实现了)

SM 进入 StLPregrasp
    │
    ├─ staticConfigure()         [编译期/初始化期，声明 CB]
    ├─ StLPregrasp::onEntry()    [同步: 写黑板、打日志]
    └─ CbMoveKnownState::onEntry() [异步线程: 读yaml → plan → execute]
                │
                │  ...轨迹执行中...
                │
                └─ 完成 → postEvent<EvCbSuccess>
                                │
                          SignalDetector 主循环消费
                                │
                          Transition → StGripperOpen