#!/usr/bin/env python3
# Wheel + IMU odometry for the Webots AgileX LIMO.
#
# Publishes /odom and the odom -> base_link transform from
#   * the wheel encoders (/joint_states) for the distance travelled, and
#   * the IMU (/imu) for the heading.
#
# Why not diff_drive_controller's own odometry?
#
# 1. Heading. The LIMO is a four-wheel skid-steer robot: its wheels slip
#    sideways whenever it turns, so the yaw integrated from the encoders is
#    only right for one turning speed. Measured in this simulation it
#    over-reports by 6 % at 1.0 rad/s on the spot and by 14 % at 1.0 rad/s
#    while driving, which is what Nav2 commands. The real LIMO firmware takes
#    its heading from the IMU for the same reason.
#
# 2. Velocity. webots_ros2_control steps the controllers on Webots time but
#    stamps them with the /clock topic, which arrives late now and then. The
#    encoder displacement per update is always exact, but when it is divided
#    by the jittering stamp interval the reported speed comes out 10-15 % too
#    high. Here the velocity is displacement over the known controller period.

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from tf2_ros import TransformBroadcaster


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class LimoOdometry(Node):
    def __init__(self):
        super().__init__('limo_odometry')
        self.declare_parameter('wheel_radius', 0.045)
        self.declare_parameter('left_wheel_names', ['front_left_wheel', 'rear_left_wheel'])
        self.declare_parameter('right_wheel_names', ['front_right_wheel', 'rear_right_wheel'])
        # 1 / controller_manager.update_rate (resource/ros2control.yaml)
        self.declare_parameter('controller_period', 0.04)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('publish_tf', True)

        self.radius = self.get_parameter('wheel_radius').value
        self.left = list(self.get_parameter('left_wheel_names').value)
        self.right = list(self.get_parameter('right_wheel_names').value)
        self.period = self.get_parameter('controller_period').value
        self.odom_frame = self.get_parameter('odom_frame_id').value
        self.base_frame = self.get_parameter('base_frame_id').value
        self.publish_tf = self.get_parameter('publish_tf').value

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.prev_positions = None
        self.prev_yaw = None
        self.imu_yaw = None      # latest IMU heading, relative to the first one
        self.imu_yaw0 = None
        self.imu_yaw_rate = 0.0

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Imu, 'imu', self.on_imu, sensor_qos)
        self.create_subscription(JointState, 'joint_states', self.on_joint_states, 10)

    def on_imu(self, msg):
        yaw = yaw_from_quaternion(msg.orientation)
        if self.imu_yaw0 is None:
            self.imu_yaw0 = yaw
        self.imu_yaw = wrap_angle(yaw - self.imu_yaw0)
        self.imu_yaw_rate = msg.angular_velocity.z

    def on_joint_states(self, msg):
        if self.imu_yaw is None:
            return
        positions = dict(zip(msg.name, msg.position))
        if any(name not in positions for name in self.left + self.right):
            return
        if self.prev_positions is None:
            self.prev_positions = positions
            self.prev_yaw = self.imu_yaw
            return

        # Distance travelled since the last update: mean of all wheels.
        wheels = self.left + self.right
        delta = sum(positions[w] - self.prev_positions[w] for w in wheels) / len(wheels)
        distance = delta * self.radius
        self.prev_positions = positions

        # Integrate along the mean heading over the interval.
        dyaw = wrap_angle(self.imu_yaw - self.prev_yaw)
        mid_yaw = self.prev_yaw + 0.5 * dyaw
        self.x += distance * math.cos(mid_yaw)
        self.y += distance * math.sin(mid_yaw)
        self.yaw = self.imu_yaw
        self.prev_yaw = self.imu_yaw

        q = Quaternion(x=0.0, y=0.0, z=math.sin(0.5 * self.yaw), w=math.cos(0.5 * self.yaw))

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = distance / self.period
        odom.twist.twist.angular.z = self.imu_yaw_rate
        for i, v in ((0, 1e-3), (7, 1e-3), (14, 1e6), (21, 1e6), (28, 1e6), (35, 1e-3)):
            odom.pose.covariance[i] = v
            odom.twist.covariance[i] = v
        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header = odom.header
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation = q
            self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = LimoOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
