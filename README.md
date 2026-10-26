# vrpn_client_instant

这是基于 `vrpn_client_ros` 的 ROS1 事件驱动客户端，包名为
`vrpn_client_instant`，可执行文件和默认 ROS 节点名分别为
`vrpn_client_instant_node`、`vrpn_tracker_instant_node`。

C++ 头文件为 `vrpn_client_instant/vrpn_client_instant.h`，命名空间为
`vrpn_client_instant`。

## 按服务端事件发布

移除了 `update_frequency` 和用于收包的 ROS 定时器。独立接收线程调用
`vrpn_Connection::mainloop(&timeout)`，由 VRPN 内部的 `select()` 等待 TCP/UDP
数据；收到报告即加入 ROS 发布队列，每条报告分别发布，不按周期采样或合并。
100 ms 超时用于空闲时检查退出和维护任务，数据到达会立即唤醒等待，
不会将发布频率限制为 10 Hz。断线时有可中断的短暂等待，避免重连循环空转。

`vrpn_client_instant_node` 和 `vrpn_tracker_instant_node` 均使用该机制。
`update_frequency` 参数保留在旧配置中不会限速，只会输出一次弃用提示。
`refresh_tracker_frequency` 仍控制自动发现 tracker 的间隔，使用单调时钟，
与已有 tracker 的数据收发频率无关。已知名称时建议直接设置 `trackers`，
这样无需等待自动发现。

## 线程与时间戳

- VRPN 回调只复制报告并入队；声明话题、发布消息和发送 TF 均在 ROS
  回调队列中执行。应用仍需调用 `ros::spin()` 或启动 spinner。
- 每个 tracker 共用一个 FIFO，将 pose/twist/accel 串行发布；即使使用
  多线程 `AsyncSpinner`，也不会并发修改 Publisher、传感器映射或 TF 状态。
- `use_server_time: false` 在 VRPN 回调收到报告时记录 ROS 时间，避免将
  发布队列等待时间计入时间戳；`true` 沿用服务端时间。对应 pose 和 TF 使用同一时间戳。
- 退出时先停止并等待接收线程，再注销 VRPN 回调、取消待发布任务并等待
  正在执行的发布。队列任务持有独立发布状态，不保留 tracker 的裸指针。
- 使用共享 connection 构造 `VrpnTrackerRos` 时，外部 connection 的拥有者
  仍需串行调用 VRPN API，并在销毁 tracker 前停止收包。

ROS Publisher 的发送队列沿用上游大小 `1`；慢订阅者、网络拥塞及 VRPN UDP
丢包仍可能导致接收端丢帧。内部待发布 FIFO 不合并报告，持续处理不过来时会积压，
应通过订阅端日志检查实际吞吐和时延。

`accel` 话题的声明类型同时修正为 `geometry_msgs/AccelStamped`。

## 编译与验证

在安装 ROS1、catkin 和 VRPN 的 Linux 工作空间中编译：

```bash
catkin_make
source devel/setup.bash
roslaunch vrpn_client_instant sample.launch server:=192.168.1.100
```

记录服务端时间间隔时设置 `use_server_time: true`。可将服务端频率设为
120/240 Hz，使用 `rostopic hz /vrpn_client_instant_node/<tracker>/pose` 验证接收频率；
停止服务端报告后，客户端不应重复发布旧帧。

发布队列回归测试覆盖突发报告、并发回调、多线程 spinner、时间戳和退出清理：

```bash
catkin_make run_tests_vrpn_client_instant
catkin_test_results
```

实现依据：VRPN 的
[网络等待及回调分发](https://github.com/vrpn/vrpn/blob/master/vrpn_Connection.C)，
以及 roscpp 的
[线程安全回调入队接口](https://github.com/ros/ros_comm/blob/noetic-devel/clients/roscpp/src/libros/callback_queue.cpp)。
