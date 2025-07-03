import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute
import math
import random

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')

        # Publisher to send movement commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        # Subscriber to read current position and angle
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        # Pose will be updated in callback
        self.pose = None
        self.state = 'init'
        self.target_theta = None
        self.escape_start_time = None
        self.last_wall = None  # remember which wall we hit last

        self.linear_speed = 2.0
        self.escape_duration = 0.5  # time to drive forward after turning

        self.timer = self.create_timer(0.1, self.control_loop)

        # Client for teleporting the turtle and setting direction instantly
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')

    def pose_callback(self, msg):
        self.pose = msg

    def control_loop(self):
        # wait for pose info and teleport service
        if self.pose is None or not self.teleport_client.service_is_ready():
            return

        twist = Twist()

        # randomize initial direction using teleport
        if self.state == 'init':
            self.target_theta = random.uniform(-math.pi, math.pi)
            self.teleport_to(self.pose.x, self.pose.y, self.target_theta)
            self.get_logger().info(f"Initial heading set to {math.degrees(self.target_theta):.1f}°")
            self.state = 'move'

        elif self.state == 'move':
            x, y = self.pose.x, self.pose.y
            margin = 0.5  # safe margin before edge

            # if close to any wall, rotate
            if x < margin or x > 11.08 - margin or y < margin or y > 11.08 - margin:
                self.get_logger().info(f"Edge detected at x = {x:.3f}, y = {y:.3f}")
                self.last_wall = self.detect_wall(x, y, margin)

                # calculate two 90° rotation options
                cw = self.normalize_angle(self.pose.theta - math.pi / 2)
                ccw = self.normalize_angle(self.pose.theta + math.pi / 2)

                # pick direction that should point away from wall
                if self.last_wall == 'left':
                    self.target_theta = ccw if abs(self.normalize_angle(ccw)) < abs(self.normalize_angle(cw)) else cw
                elif self.last_wall == 'right':
                    self.target_theta = ccw if abs(self.normalize_angle(ccw - math.pi)) < abs(self.normalize_angle(cw - math.pi)) else cw
                elif self.last_wall == 'bottom':
                    self.target_theta = ccw if abs(self.normalize_angle(ccw - math.pi/2)) < abs(self.normalize_angle(cw - math.pi/2)) else cw
                elif self.last_wall == 'top':
                    self.target_theta = ccw if abs(self.normalize_angle(ccw + math.pi/2)) < abs(self.normalize_angle(cw + math.pi/2)) else cw
                else:
                    self.target_theta = ccw  # fallback

                # rotate using teleport
                self.teleport_to(self.pose.x, self.pose.y, self.target_theta)
                self.get_logger().info(f"Rotated to {math.degrees(self.target_theta):.1f}°, escaping...")
                self.escape_start_time = self.get_clock().now()
                self.state = 'escape'

            else:
                # just move forward if everything's fine
                twist.linear.x = self.linear_speed

        elif self.state == 'escape':
            elapsed = (self.get_clock().now() - self.escape_start_time).nanoseconds / 1e9

            if elapsed < self.escape_duration:
                twist.linear.x = self.linear_speed
            else:
                # check if we're heading back toward the wall again
                if self.should_turn_again():
                    self.get_logger().info("Still facing wall after escape. Turning again.")
                    # rotate another 90° away
                    self.teleport_to(self.pose.x, self.pose.y, self.normalize_angle(self.pose.theta + math.pi / 2))
                    self.escape_start_time = self.get_clock().now()
                else:
                    self.get_logger().info("Escape complete and facing away. Resuming movement.")
                    self.state = 'move'

        self.cmd_vel_pub.publish(twist)

   def should_turn_again(self):
        if self.last_wall is None:
            return False
    
        theta = self.pose.theta
        x, y = self.pose.x, self.pose.y
        margin = 0.5  # reuse same margin as in control_loop
        safe_distance = 0.7  # slightly beyond the margin
    
        if self.last_wall == 'left':
            facing_wall = math.cos(theta) <= 0
            too_close = x < safe_distance
        elif self.last_wall == 'right':
            facing_wall = math.cos(theta) >= 0
            too_close = x > 11.08 - safe_distance
        elif self.last_wall == 'bottom':
            facing_wall = math.sin(theta) <= 0
            too_close = y < safe_distance
        elif self.last_wall == 'top':
            facing_wall = math.sin(theta) >= 0
            too_close = y > 11.08 - safe_distance
        else:
            return False
    
        return too_close and facing_wall

    def detect_wall(self, x, y, margin):
        # return which wall we hit
        if x < margin:
            return 'left'
        elif x > 11.08 - margin:
            return 'right'
        elif y < margin:
            return 'bottom'
        elif y > 11.08 - margin:
            return 'top'
        else:
            return None

    def teleport_to(self, x, y, theta):
        # send teleport request
        req = TeleportAbsolute.Request()
        req.x = x
        req.y = y
        req.theta = theta
        self.teleport_client.call_async(req)

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
