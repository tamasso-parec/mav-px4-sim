import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseArray
from std_msgs.msg import Header

class DronePoseBridge(Node):
    def __init__(self):
        super().__init__('drone_pose_bridge')
        self.sub = self.create_subscription(
            PoseArray,  
            '/gazebo_world/object_poses',
            self.pose_callback,
            10)
        

        self.pub = self.create_publisher(PoseStamped, '/x500/ground_truth', 10)

    def pose_callback(self, msg):
        # print("Received pose message")
        pose_msg = PoseStamped()
        pose_msg.header = Header()
        pose_msg.header.frame_id = "world"
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        drone_msg = msg.poses[2]
        pose_msg.pose.position = drone_msg.position
        pose_msg.pose.orientation = drone_msg.orientation
        self.pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DronePoseBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
