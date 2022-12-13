import math

import rospy

import geometry_msgs.msg
import tf2_ros


# Wrap angle to [-pi, pi):
def wrap_angle(theta):
    return math.atan2(math.sin(theta), math.cos(theta))

# Configuration of the unicycle.
class Configuration:
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x = x
        self.y = y
        self.theta = theta
    
    def __repr__(self):
        return '({}, {}, {})'.format(self.x, self.y, self.theta)
    
    def set_from_tf_transform(self, transform):
        self.x = transform.transform.translation.x
        self.y = transform.transform.translation.y
        q = transform.transform.rotation
        self.theta = math.atan2(
          2.0 * (q.w * q.z + q.x * q.y),
          1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

# Pseudovelocities of the unicycle.
class Command:
    def __init__(self, driving_velocity, steering_velocity):
        self.driving_velocity = driving_velocity
        self.steering_velocity = steering_velocity
    
    def __repr__(self):
        return '({}, {})'.format(self.driving_velocity, self.steering_velocity)

# Posture regulation controller.
class PostureRegulationController:
    def __init__(self, hparams):
        self.hparams = hparams
    
    def compute_velocity_command(self, configuration, desired_configuration):
        # Unicycle configuration:
        x = configuration.x
        y = configuration.y
        theta = configuration.theta
      
        # Desired configuration:
        x_d = desired_configuration.x
        y_d = desired_configuration.y
        theta_d = desired_configuration.theta

        # Unicycle configuration in desired reference frame coordinates:
        x_r = math.cos(-theta_d) * (x - x_d) - math.sin(-theta_d) * (y - y_d)
        y_r = math.sin(-theta_d) * (x - x_d) + math.cos(-theta_d) * (y - y_d)
        theta_r = wrap_angle(-theta_d + theta)

        # Polar coordinates in relative coordinates:
        rho   = math.sqrt(math.pow(x_r, 2.0) + math.pow(y_r, 2.0))
        gamma = wrap_angle(math.atan2(y_r, x_r) - theta_r + math.pi)
        delta = wrap_angle(gamma + theta_r)

        # Feedback control:
        if abs(rho) < self.hparams.tol:
            driving_velocity = 0.0
            steering_velocity = 0.0
        else:
            driving_velocity  = self.hparams.k1 * rho * math.cos(gamma)
            steering_velocity = self.hparams.k2 * gamma + self.hparams.k1 * math.sin(gamma) * math.cos(gamma) / gamma * (gamma + self.hparams.k3 * delta)
        
        return Command(driving_velocity, steering_velocity)

# Hyperparameters for posture regulation controller.
class PostureRegulationHParams:
    def __init__(self, k1, k2, k3, tol=0.01):
        self.k1 = k1
        self.k2 = k2
        self.k3 = k3
        self.tol = tol

# Management of control loop.
class MotionControlManager:
    def __init__(self):
        # Assuming initial configuration is (0.0, 0.0, 0.0):
        self.configuration = Configuration()
        self.desired_configuration = Configuration(1.0, 1.0, math.pi)

        # Setting up posture regulation controller:
        self.posture_regulation_controller = PostureRegulationController(
            PostureRegulationHParams(1.0, 2.5, 3.0),
        )

        # Setting up topic names:
        cmd_vel_topic = '/mobile_base_controller/cmd_vel'

        # Setting up reference frames:
        self.map_frame = 'odom'
        self.base_footprint_frame = 'base_footprint'

        # Setting up TF listener:
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Setting up publishers:
        self.cmd_vel_publisher = rospy.Publisher(
            cmd_vel_topic,
            geometry_msgs.msg.Twist,
            queue_size=1
        )

    def publish_velocity_command(self, command):
        # Create a twist ROS message:
        cmd_vel_msg = geometry_msgs.msg.Twist()
        cmd_vel_msg.linear.x = command.driving_velocity
        cmd_vel_msg.linear.y = 0.0
        cmd_vel_msg.linear.z = 0.0
        cmd_vel_msg.angular.x = 0.0
        cmd_vel_msg.angular.y = 0.0
        cmd_vel_msg.angular.z = command.steering_velocity

        # Publish a twist ROS message:
        self.cmd_vel_publisher.publish(cmd_vel_msg)
    
    def start(self):
        rate = rospy.Rate(100) # 100 Hz

        # Main control loop:
        while not rospy.is_shutdown():
            # Read robot configuration:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame, self.base_footprint_frame, rospy.Time()
                )
                self.configuration.set_from_tf_transform(transform)
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                rate.sleep()
                continue

            # Posture regulation:
            command = \
                self.posture_regulation_controller.compute_velocity_command(
                    self.configuration,
                    self.desired_configuration
                )

            # Send velocity commands to robot:
            self.publish_velocity_command(command)

            # Keep controller frequency at specified rate:
            rate.sleep()


def main():
    rospy.init_node('posture_regulation', log_level=rospy.INFO)
    
    motion_control_manager = MotionControlManager()
    motion_control_manager.start()