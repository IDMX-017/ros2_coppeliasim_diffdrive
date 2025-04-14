Video Demonstration of Sphere-Following Diff-Robot [click the image]

[![Watch the video](https://img.youtube.com/vi/rvQTukPbEp0/maxresdefault.jpg)](https://youtu.be/rvQTukPbEp0 )


## 🤖 ROS 2 Interface + Low-Level Velocity Control

This node implements a **ROS 2 subscriber** that listens to the topic `/cmd_vel` of type `geometry_msgs/msg/Twist` to control a differential-drive robot inside **CoppeliaSim**.

---

### 📡 ROS 2 Interface

The node subscribes to velocity commands published on `/cmd_vel`:

```python
self.subscription = self.create_subscription(
    Twist, cmd_vel, self.callback, 10
)


