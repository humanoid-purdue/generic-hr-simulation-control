from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from tf2_ros import TransformBroadcaster, TransformStamped
import numpy as np
import time

JOINT_LIST = ['left_hip_yaw_joint', 'left_hip_roll_joint', 'left_hip_pitch_joint', 'left_knee_joint', 'left_ankle_joint', 'right_hip_yaw_joint', 'right_hip_roll_joint', 'right_hip_pitch_joint', 'right_knee_joint', 'right_ankle_joint', 'torso_joint', 'left_shoulder_pitch_joint', 'left_shoulder_roll_joint', 'left_shoulder_yaw_joint', 'left_elbow_joint', 'right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 'right_shoulder_yaw_joint', 'right_elbow_joint']

class StatePublisher(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('state_publisher')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.joint_traj_pub = self.create_publisher(JointTrajectory, 'joint_trajectories', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        loop_rate = self.create_rate(30)
        joint_state = JointState()
        joint_traj = JointTrajectory()
        jtp = JointTrajectoryPoint()
        duration = Duration()
        st = time.time()
        try:
            while rclpy.ok():
                rclpy.spin_once(self)

                # update joint_state
                now = self.get_clock().now()
                joint_state.header.stamp = now.to_msg()
                joint_state.name = JOINT_LIST
                joint_state.position = [0.] * len(JOINT_LIST)

                joint_traj.header.stamp = now.to_msg()
                joint_traj.joint_names = JOINT_LIST

                jtp.effort = [5.0 * np.sin((time.time() - st) * 4) ] * len(JOINT_LIST)
                duration.sec = 9999
                duration.nanosec  = 0
                jtp.time_from_start = duration

                joint_traj.points = [jtp]

                # send the joint state and transform
                self.joint_pub.publish(joint_state)
                #self.joint_traj_pub.publish(joint_traj)

                loop_rate.sleep()
                self.get_logger().info("Successfully published joint trajectory")

        except KeyboardInterrupt:
            pass

def main():
    node = StatePublisher()

if __name__ == '__main__':
    main()