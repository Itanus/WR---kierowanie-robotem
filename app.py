#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from sensor_msgs.msg import LaserScan
import math

global target_pose
target_pose = Pose()
target_pose.x = -2
target_pose.y = -2
global pose
pose = Pose()
state = 0
angle = 0
angle_precision = math.pi / 90
dist_error = 0.2
global directions
directions = {
    'right': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left': 0,
}


def take_action(directions):
    Directions = directions
    new_vel = Twist()
    linear_x = 0
    angular_z = 0

    pub.publish(new_vel)
    d = 2

    print(Directions['front'])
    print(Directions['fleft'])
    print(Directions['fright'])
    if Directions['front'] > d and Directions['fleft'] > d and Directions['fright'] > d:
        change_state(1)
    elif Directions['front'] < d < Directions['fleft'] and Directions['fright'] > d:
        change_state(3)
    elif Directions['front'] > d > Directions['fright'] and Directions['fleft'] > d:
        change_state(2)
    elif Directions['front'] > d > Directions['fleft'] and Directions['fright'] > d:
        change_state(4)
    elif Directions['front'] < d < Directions['fleft'] and Directions['fright'] < d:
        change_state(2)
    elif Directions['front'] < d < Directions['fright'] and Directions['fleft'] < d:
        change_state(3)
    elif Directions['front'] < d and Directions['fleft'] < d and Directions['fright'] < d:
        change_state(3)
    elif Directions['front'] > d > Directions['fleft'] and Directions['fright'] < d:
        change_state(4)


def find_wall():
    global pub
    print("find wall")
    new_vel = Twist()
    new_vel.linear.x = 0.2
    new_vel.angular.z = -0.3


pub.publish(new_vel)


# return new_vel

def turn_left():
    global pub
    new_vel = Twist()


print("left")
# new_vel.linear.x = 0.2
new_vel.angular.z = 0.3
# return new_vel
pub.publish(new_vel)


def follow_wall():
    global directions, pub
    print("follow")
    new_vel = Twist()
    new_vel.linear.x = 0.5
    # return new_vel


pub.publish(new_vel)


# funkcja wywolywana przy przyjsciu danych ze skanera laserowego
def scan_callback(scan):
    print
    min(min(scan.ranges[36:71]), 10)
    directions = {
        # 'right':  min(min(scan.ranges[0:143]), 10),
        'fright': min(min(scan.ranges[36:71]), 10),
        'front': min(min(scan.ranges[72:107]), 10),
        'fleft': min(min(scan.ranges[108:143]), 10),
        # 'left':   min(min(scan.ranges[576:713]), 10),
    }

    take_action(directions)
    # print scan


# funkcja wywolywana przy przyjsciu danych o lokalizacji robota
def odom_callback(odom):
    global target_pose
    global pub
    global pose
    global angle

    pose = Pose()
    pose.x = odom.pose.pose.position.x
    pose.y = odom.pose.pose.position.y
    pose.theta = tf.transformations.euler_from_quaternion(
        [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z,
         odom.pose.pose.orientation.w])[2]
    angle = pose.theta

    print("Pozycja x: ", odom.pose.pose.position.x)
    print("Pozycja y: ", odom.pose.pose.position.y)
    print("Pozycja theta: ", pose.theta)


def change_state(new_state):
    global state
    state = new_state


def change_angle(target_pose):
    global angle, pub, angle_precision, state, pose
    corr_angle = math.atan2(target_pose.y - pose.y, target_pose.x - pose.x)
    new_angle = corr_angle - angle

    new_vel = Twist()
    if (math.fabs(new_angle) > angle_precision):
        if new_angle > 0:
            new_vel.angular.z = 0.6
        else:
            new_vel.angular.z = -0.6
    pub.publish(new_vel)

    if math.fabs(new_angle) <= angle_precision:
        change_state(1)


def go_forward(target_pose):
    global angle, pub, angle_precision, state, pose
    corr_angle = math.atan2(target_pose.y - pose.y, target_pose.x - pose.x)
    new_angle = corr_angle - angle
    new_pose = math.sqrt(pow(target_pose.y - pose.y, 2) + pow(target_pose.x - pose.x, 2))

    if new_pose > dist_error:
        new_vel = Twist()
        new_vel.linear.x = 0.6
        pub.publish(new_vel)
    else:
        change_state(6)

    if math.fabs(new_angle) > angle_precision:
        change_state(0)


def stop():
    new_vel = Twist()
    new_vel.linear.x = 0
    new_vel.angular.z = 0
    pub.publish(new_vel)


if __name__ == "__main__":
    global new_vel
    global pub
    global pose
    global target_pose
    global angle

    new_vel = Twist()
    rospy.init_node('zadanie_bug', anonymous=True)
    print("ready")
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.Subscriber('/scan', LaserScan, scan_callback)

    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        if state == 0:
            change_angle(target_pose)
        elif state == 1:
            go_forward(target_pose)
        elif state == 2:
            follow_wall()
        elif state == 3:
            turn_left()
        elif state == 4:
            find_wall()
        elif state == 6:
            stop()
            pass
        # pub.publish(new_vel)
        rate.sleep()

print("END")
