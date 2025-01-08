import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from mavros_msgs.msg import CommandBool, SetMode
from mavros_msgs.srv import CommandTOL

class TakeoffCommand(Node):
    def __init__(self):
        super().__init__('takeoff_command')
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        
        self.timer = self.create_timer(5.0, self.send_takeoff_command)

    def send_takeoff_command(self):
        if not self.arming_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Arming service not available')
            return

        if not self.set_mode_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Set mode service not available')
            return

        if not self.takeoff_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Takeoff service not available')
            return

        # Arm the drone
        arm_cmd = CommandBool.Request()
        arm_cmd.value = True
        self.arming_client.call_async(arm_cmd)
        self.get_logger().info('Arming drone')

        # Set mode to OFFBOARD
        set_mode_cmd = SetMode.Request()
        set_mode_cmd.custom_mode = 'OFFBOARD'
        self.set_mode_client.call_async(set_mode_cmd)
        self.get_logger().info('Setting mode to OFFBOARD')

        # Send takeoff command
        takeoff_cmd = CommandTOL.Request()
        takeoff_cmd.altitude = 10.0  # Set desired takeoff altitude
        self.takeoff_client.call_async(takeoff_cmd)
        self.get_logger().info('Sending takeoff command')

def main(args=None):
    rclpy.init(args=args)
    node = TakeoffCommand()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

