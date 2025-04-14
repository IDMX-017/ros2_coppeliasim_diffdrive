Video Demonstration of Sphere-Following Diff-Robot [click the image]

[![Watch the video](https://img.youtube.com/vi/rvQTukPbEp0/maxresdefault.jpg)](https://youtu.be/rvQTukPbEp0 )


## ROS 2 Interface + Low-Level Velocity Control

This node implements a **ROS 2 subscriber** that listens to the topic `/cmd_vel` of type `geometry_msgs/msg/Twist` to control a differential-drive robot inside **CoppeliaSim**.

---

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class Subscriber(Node):
    def __init__(self, cmd_vel):
        super().__init__('subscriber_cmd_vel')
        self.subscription = self.create_subscription(Twist, cmd_vel, self.callback, 10)
        self.leftMotor = sim.getObject('./leftMotor')   # Handle of the left motor
        self.rightMotor = sim.getObject('./rightMotor') # Handle of the right motor

    def callback(self, msg):
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        leftVel, rightVel = self.obtainVelocities(linear_vel, angular_vel)
        sim.setJointTargetVelocity(self.leftMotor, leftVel)
        sim.setJointTargetVelocity(self.rightMotor, rightVel)
        print(f'Linear velocity: {linear_vel:.2f}, Angular velocity: {angular_vel:.2f}')

    def obtainVelocities(self, linear, angular):
        wheel_radius = 0.097
        wheel_base = 0.1155
        v_left = linear - (angular * wheel_base / 2.0)
        v_right = linear + (angular * wheel_base / 2.0)
        leftVel = v_left / wheel_radius
        rightVel = v_right / wheel_radius
        return leftVel, rightVel

def sysCall_init():
    sim = require('sim')
    simROS2 = require('simROS2')

    rclpy.init()
    self.subscriber_node = Subscriber('cmd_vel')

def sysCall_sensing():
    rclpy.spin_once(self.subscriber_node, timeout_sec=0)

def sysCall_cleanup():
    self.subscriber_node.destroy_node()
    rclpy.shutdown()
```



