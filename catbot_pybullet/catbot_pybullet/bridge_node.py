import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Float64MultiArray
import pybullet as p
import pybullet_data
import time

class PyBulletBridge(Node):
    def __init__(self):
        super().__init__('pybullet_bridge')
        
        # Initialize PyBullet in GUI mode
        self.physics_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        
        # Load a simple plane and a box
        self.plane_id = p.loadURDF("plane.urdf")
        self.startPos = [0,0,1]
        self.startOrientation = p.getQuaternionFromEuler([0,0,0])
        self.robotId = p.loadURDF("catbot_pybullet/meshes/catbot.urdf", self.startPos, self.startOrientation)
        
        # ROS 2 Publishers & Subscribers
        self.pose_publisher = self.create_publisher(Pose, 'pybullet_pose', 10)
        self.joints_cmd_subscriber = self.create_subscription(Float64MultiArray, "/servo_angles", self.joints_cmd_callback, 10)
        
        # Timer for updating the simulation
        self.timer = self.create_timer(0.01, self.update_simulation)
    
    def joints_cmd_callback(self, msg: Float64MultiArray):
        """Receives joint commands and applies them to the PyBullet object."""
        jnt_cmds = np.array(msg.data) / (360 / (2 * math.pi))
        idx = [11, 10, 0, 7, 8, 9, 4, 5, 6, 1, 2, 3]
        adjusted_joint_cmds = jnt_cmds[idx]
        # self.get_logger().info(f"input position: {adjusted_joint_cmds}")
        p.setJointMotorControlArray(bodyUniqueId=self.robotId,
                                        jointIndices=range(12),
                                        controlMode=p.POSITION_CONTROL,
                                        targetPositions=adjusted_joint_cmds)
        for i in range(p.getNumJoints(self.robotId)):
            jnt = p.getJointInfo(bodyUniqueId=self.robotId, jointIndex=i)
            self.get_logger().info(f"joints to pybullet: {jnt[0:2]} \n")
        

    
    def update_simulation(self):
        """Step simulation and publish object state."""
        p.stepSimulation()
        time.sleep(0.01)  # Simulate real time
        
        # Get object position and orientation
        pos, orn = p.getBasePositionAndOrientation(self.robotId)
        
        # Publish as a ROS 2 Pose message
        pose_msg = Pose()
        pose_msg.position.x, pose_msg.position.y, pose_msg.position.z = pos
        pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w = orn
        
        self.pose_publisher.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PyBulletBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    p.disconnect()

if __name__ == '__main__':
    main()
