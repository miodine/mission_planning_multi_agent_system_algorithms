#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

from evry_project_plugins.srv import DistanceToFlag

# import
import math


class Robot:
    def __init__(self, robot_name):
        """Constructor of the class Robot
        The required publishers / subscribers are created.
        The attributes of the class are initialized

        Args:
            robot_name (str): Name of the robot, like robot_1, robot_2 etc. To be used for your subscriber and publisher with the robot itself
        """
        self.speed = 0.0
        self.angle = 0.0
        self.sonar = 0.0           # Sonar distance
        self.x, self.y = 0.0, 0.0  # coordinates of the robot
        self.yaw = 0.0             # yaw angle of the robot
        self.robot_name = robot_name

        '''Listener and publisher'''

        rospy.Subscriber(self.robot_name + "/sensor/sonar_front",
                         Range, self.callbackSonar)
        rospy.Subscriber(self.robot_name + "/odom",
                         Odometry, self.callbackPose)
        self.cmd_vel_pub = rospy.Publisher(
            self.robot_name + "/cmd_vel", Twist, queue_size=1)

    def callbackSonar(self, msg):
        """Callback function that gets the data coming from the ultrasonic sensor

        Args:
            msg (Range): Message that contains the distance separating the US sensor from a potential obstacle
        """
        self.sonar = msg.range

    def get_sonar(self):
        """Method that returns the distance separating the ultrasonic sensor from a potential obstacle
        """
        return self.sonar

    def callbackPose(self, msg):
        """Callback function that gets the data coming from the ultrasonic sensor

        Args:
            msg (Odometry): Message that contains the coordinates of the agent
        """
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        quaternion = msg.pose.pose.orientation
        quaternion_list = [quaternion.x,
                           quaternion.y, quaternion.z, quaternion.w]
        roll, pitch, yaw = euler_from_quaternion(quaternion_list)
        self.yaw = yaw

    def get_robot_pose(self):
        """Method that returns the position and orientation of the robot"""
        return self.x, self.y, self.yaw

    def constraint(self, val, min=-2.0, max=2.0):
        """Method that limits the linear and angular velocities sent to the robot

        Args:
            val (float): [Desired velocity to send
            min (float, optional): Minimum velocity accepted. Defaults to -2.0.
            max (float, optional): Maximum velocity accepted. Defaults to 2.0.

        Returns:
            float: Limited velocity whose value is within the range [min; max]
        """
        # DO NOT TOUCH
        if val < min:
            return min
        if val > max:
            return max
        return val

    def set_speed_angle(self, linear, angular):
        """Method that publishes the proper linear and angular velocities commands on the related topic to move the robot

        Args:
            linear (float): desired linear velocity
            angular (float): desired angular velocity
        """
        cmd_vel = Twist()
        cmd_vel.linear.x = self.constraint(linear, min = -3.0, max = 3.0)
        cmd_vel.angular.z = self.constraint(angular, min=-1, max=1)
        self.cmd_vel_pub.publish(cmd_vel)

    def getDistanceToFlag(self):
        """Get the distance separating the agent from a flag. The service 'distanceToFlag' is called for this purpose.
        The current position of the robot and its id should be specified. The id of the robot corresponds to the id of the flag it should reach


        Returns:
            float: the distance separating the robot from the flag
        """
        rospy.wait_for_service('/distanceToFlag')
        try:
            service = rospy.ServiceProxy('/distanceToFlag', DistanceToFlag)
            pose = Pose2D()
            pose.x = self.x
            pose.y = self.y
            # int(robot_name[-1]) corresponds to the id of the robot. It is also the id of the related flag
            result = service(pose, int(self.robot_name[-1]))
            return result.distance
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)


class robotPID:
    ''' 
    A class for handling velocity and angle adjustment via PID control.
    '''

    def __init__(self, dt, tunings_velo: list, tunings_angle: list):
        # dt -> time increment
        self.dt = dt

        # CONTROLLER GAINS #

        # Coding:
        # k -> gain ...
        # v -> ... of velocity ...
        # a -> ... of angle ...
        # p,i,d -> at proportional, integral or derivative term

        self.k_v_p = tunings_velo[0]
        self.k_v_i = tunings_velo[1]
        self.k_v_d = tunings_velo[2]

        self.k_a_p = tunings_angle[0]
        self.k_a_i = tunings_angle[1]
        self.k_a_d = tunings_angle[2]

        # (v)elocity (e)rror from previous time-sample (z-operator^1)
        self.e_v_z_1 = 0
        self.v_epsilon = 1  # deadzone margin

        self.e_a_z_1 = 0      # (a)ngular (e)rror from previous time-sample
        self.a_epsilon = 0.001  # deadzone margin

    def pid_dist_to_velo(self, distance):
        # open loop PID -> distance to velocity
        e = distance
        if abs(e) <= self.v_epsilon:
            return 0

        proportional_term = e*self.k_v_p
        integral_term = self.k_v_i*(e+self.e_v_z_1)*self.dt
        derivative_term = self.k_v_d*(e-self.e_v_z_1)/self.dt

        self.e_v_z_1 = e
        v = proportional_term + integral_term + derivative_term
    
        return v

    def pid_angle(self, set_point_angle, current_angle):
        # closed loop PID for angle adjustment
        e = set_point_angle - current_angle
        if abs(e) <= self.a_epsilon:
            return 0
        if abs(self.e_v_z_1) <= self.v_epsilon:
            return 0

        proportional_term = e*self.k_a_p
        integral_term = self.k_a_i*(e+self.e_a_z_1)*self.dt
        derivative_term = self.k_a_d*(e-self.e_a_z_1)/self.dt

        self.e_a_z_1 = e
        a = proportional_term + integral_term + derivative_term
        return a


def estimate_desired_heading(x_r, y_r, x_g, y_g):
    # compute the vector from current robot position to 
    # the desired goal coordinates
    
    h_x = x_g-x_r
    h_y = y_g-y_r

    # compute the angle the angle at which the robot should
    # approach the target
    theta_desired = math.atan2(h_y, h_x)
    return theta_desired


def estimate_global_goal_coords(robot):
    # move the robot a bit - so that it would get 
    # accurate, global positioning from the get_robot_pose() method.

    robot.set_speed_angle(0.01, 0)
    rospy.sleep(0.5)
    robot.set_speed_angle(0, 0)
    rospy.sleep(0.5)

    # get initial telemetry
    x, y, theta = robot.get_robot_pose()
    dtf = float(robot.getDistanceToFlag())
    
    # assuming that the robot is positioned directly in front of the goal:
    x_g = dtf*math.cos(theta)/2
    y_g = dtf*math.sin(theta)/2

    return x_g, y_g


def run_strategy_1():
    """Main loop"""
    # env setup
    robot_name = rospy.get_param("~robot_name")
    robot = Robot(robot_name)
    print(f"Robot : {robot_name} is starting..")

    # sim setup
    dt = 0.25

    # controller setup
    tunings_v = [20, 5, 1]
    tunings_a = [1, 0.1, 0]
    robot_controller = robotPID(dt, tunings_v, tunings_a)

    angle = 0
    velocity = 0

    # strategy setup
    obstacle_detection_threshold = 5
    obstacle_deflection_angle = 0.6
    x_tmp = 0
    y_tmp = 0
    obs_ang_constraint = lambda val,sym_cstrt: max(min(val,sym_cstrt), -sym_cstrt)
    counter = -1 

    x_g, y_g = estimate_global_goal_coords(robot)

    while not rospy.is_shutdown():
        # Strategy

        # -- get telemetry --
        x, y, theta = robot.get_robot_pose()
        snr_obs = float(robot.get_sonar())
        dtf = float(robot.getDistanceToFlag())
        angle_desired = estimate_desired_heading(x, y, x_g, y_g)

        # -- run strategy --
        if snr_obs < obstacle_detection_threshold: # assign temporary goal
            x_tmp = x + 4*obs_ang_constraint(math.cos(angle_desired + obstacle_deflection_angle), 1.57)
            y_tmp = y + 4*obs_ang_constraint(math.sin(angle_desired + obstacle_deflection_angle), 1.57)
            counter = int(1/(dt))

        if counter > 0:
            angle_desired = estimate_desired_heading(x, y, x_tmp, y_tmp)
            counter -=1
        
        # -- condition control variables -- 
        angle = robot_controller.pid_angle(angle_desired, theta)        
        velocity = robot_controller.pid_dist_to_velo(dtf)

        print(f"{robot_name} distance to flag = ", dtf)
        # Finishing by publishing the desired speed.
        # DO NOT TOUCH.
        robot.set_speed_angle(velocity, angle)
        rospy.sleep(dt)


if __name__ == "__main__":
    print("Running strategy 1")
    rospy.init_node("Controller", anonymous=True)
    run_strategy_1()
