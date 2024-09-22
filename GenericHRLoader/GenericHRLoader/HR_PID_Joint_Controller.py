import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from tf2_ros import TransformBroadcaster, TransformStamped
from tf2_msgs.msg import TFMessage
import numpy as np
import time

JOINT_LIST = ['left_hip_yaw_joint', 'left_hip_roll_joint', 'left_hip_pitch_joint', 'left_knee_joint', 'left_ankle_joint', 'right_hip_yaw_joint', 'right_hip_roll_joint', 'right_hip_pitch_joint', 'right_knee_joint', 'right_ankle_joint', 'torso_joint', 'left_shoulder_pitch_joint', 'left_shoulder_roll_joint', 'left_shoulder_yaw_joint', 'left_elbow_joint', 'right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 'right_shoulder_yaw_joint', 'right_elbow_joint']

class HRPIDJointController(Node):

    def __init__(self):
        super().__init__('hr_pid_controller')
        qos_profile = QoSProfile(depth=10)
        self.joint_traj_pub = self.create_publisher(JointTrajectory, 'joint_trajectories', qos_profile)
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)
        self.subscription_pos = self.create_subscription(
            TFMessage,
            'robot_position',
            self.pos_callback,
            10
        )

    def pos_callback(self, msg):
        transforms = msg.transforms
        v1 = {}
        for i in transforms:
            v1[i.child_frame_id] = [i.transform.translation.x, i.transform.translation.y, i.transform.translation.z]
        self.get_logger().info(str(v1))

    def listener_callback(self, msg):
        joint_state = JointState()
        joint_traj = JointTrajectory()
        jtp = JointTrajectoryPoint()
        duration = Duration()
        st = time.time()
        now = self.get_clock().now()
        joint_state.header.stamp = now.to_msg()
        joint_state.name = JOINT_LIST

        joint_traj.header.stamp = now.to_msg()
        joint_traj.joint_names = JOINT_LIST

        efforts = []
        for c in range(len(JOINT_LIST)):
            joint_pos = msg.position[c]
            efforts += [-200 * (joint_pos - 0)]

        jtp.effort = efforts
        duration.sec = 9999
        duration.nanosec = 0
        jtp.time_from_start = duration

        joint_traj.points = [jtp]

        # send the joint state and transform
        # self.joint_pub.publish(joint_state)
        self.joint_traj_pub.publish(joint_traj)



def main(args=None):
    rclpy.init(args=args)

    hrpid = HRPIDJointController()

    rclpy.spin(hrpid)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    hrpid.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()