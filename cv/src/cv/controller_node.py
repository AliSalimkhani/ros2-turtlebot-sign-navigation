import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time

class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        # --- State Machine ---
        self.state = 'READY'  # States: READY, WAITING, TURNING, STOPPING
        self.pending_command = None
        self.command_received_time = 0.0
        self.turn_start_time = 0.0

        # --- Timing Parameters ---
        self.maneuver_delay = 3.0  # Seconds to wait after seeing a sign
        self.turn_duration = 1.0    # Seconds to perform the actual turn

        # --- Subscriber and Publisher ---
        self.subscription = self.create_subscription(
            String,
            '/command',
            self.control_callback,
            10
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- Timer for the main logic ---
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info("Controller node started. State: READY")

    def control_callback(self, msg):
        """Receives a command and processes it based on current state."""
        # Always accept STOP commands regardless of state
        if msg.data == "STOP":
            self.get_logger().info(f"STOP command received. Moving to STOPPING state.")
            self.state = 'STOPPING'
            self.pending_command = msg.data
            return
            
        # Only accept other commands when in READY state
        if self.state != 'READY':
            self.get_logger().info(f"Ignoring command '{msg.data}', not in READY state.")
            return

        self.get_logger().info(f"Command '{msg.data}' received. Starting WAITING state.")
        self.state = 'WAITING'
        self.pending_command = msg.data
        self.command_received_time = time.time()

    def timer_callback(self):
        """The main state machine logic."""
        twist = Twist()

        if self.state == 'READY':
            # Default behavior: move forward
            twist.linear.x = 0.3
            self.publisher.publish(twist)

        elif self.state == 'WAITING':
            # Wait for the delay period, continuing to move
            twist.linear.x = 0.1
            self.publisher.publish(twist)

            if (time.time() - self.command_received_time) >= self.maneuver_delay:
                self.get_logger().info("Delay finished. Starting TURNING state.")
                self.state = 'TURNING'
                self.turn_start_time = time.time()

        elif self.state == 'TURNING':
            # Execute the turn for a specific duration
            if self.pending_command == "LEFT":
                twist.angular.z = 1.5
            elif self.pending_command == "RIGHT":
                twist.angular.z = -1.5
            
            self.publisher.publish(twist)

            if (time.time() - self.turn_start_time) >= self.turn_duration:
                self.get_logger().info("Turn duration finished. Starting STOPPING state.")
                self.state = 'STOPPING'

        elif self.state == 'STOPPING':
            # Send a stop command and reset to READY
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher.publish(twist)
            
            self.get_logger().info("Maneuver complete. Resetting to READY state.")
            self.state = 'READY'
            self.pending_command = None

    def destroy_node(self):
        # Ensure robot is stopped on shutdown
        stop_twist = Twist()
        self.publisher.publish(stop_twist)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()