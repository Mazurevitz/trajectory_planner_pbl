import dubins
import matplotlib.pyplot as plt
import numpy as np
import json
import math
from Helper import Helper
from dubins_talker import DubinsTalker
import rospy
from std_msgs.msg import Float64MultiArray, String
from math import sqrt, atan2, degrees
from PointApproximator import PointApproximator
import time
from pynput import keyboard
from geometry_msgs.msg import PoseWithCovarianceStamped
import datetime
import pandas as pd 
# SET GLOBAL POINT APPROXIMATOR
point_approximator = PointApproximator()

# SET GLOBAL VARIABLES
FINAL_POINT_X = 350 # [cm]
FINAL_POINT_Y = 200 # [cm]
FINAL_THETA = 270 # [degrees]
HOST_TURNING_RADIUS = 68.0 # [degrees]

class PathHolder:

    def __init__(self):
        self.path = []

    def update_path(self, path):
        self.path = path

    def clean_path(self):
        self.path = []

    def get_stopping_frame(self):
        return {"MovementFrameTurnPropulsion": {

                "propulsionValue": 0,
                "turnValue": 0,
                "propulsionDirection": 1,
                "turnDirection": 1,
                "timeToDrive": 200,
                "isQueued": 0
            }}

class SubscriptionHandler(object):
    
    def __init__(self, callback_function):
        # save the subscriber object to a class member
        self.sub = None
        self.callback_function = callback_function

    def subscribe(self, name, data_type=Float64MultiArray):
        print("SUBSCRIBE: ", self.callback_function.__name__)
        self.sub = rospy.Subscriber(name, data_type, self.callback_function)

    def unsubscribe(self):
        # use the saved subscriber object to unregister the subscriber
        self.sub.unregister()

class PositionHolder():
    def __init__(self):
        self.starting_position_roll = 0
        self.odometry_position = []
        self.UWB_positions = []
        self.run_flag = True


    def update_roll(self, position):
        self.starting_position_roll = position

    def update_UWB(self, UWB_pos):
        self.UWB_positions.append(UWB_pos)

    def update_odometry(self, odometry_position):
        self.odometry_position.append(odometry_position)

pos_holder = PositionHolder()


def dubins_path(x0, y0, theta0, x1, y1, theta1, turning_radius = 3.0, step_size = 5):
    """
    Find points between starting point and finishing point
    
    Arguments:
        x0 {int} -- starting x
        y0 {int} -- starting y
        theta0 {int} -- starting yawn
        x1 {int} -- finishing x
        y1 {int} -- finishing y
        theta1 {int} -- finishing yawn
    """

    print(turning_radius, step_size)
    print("init path")
    if (step_size is None):
        step_size = 5
    q0 = (x0, y0, theta0)
    q1 = (x1, y1, theta1)

    print("q0, q1, turning_radius", q0, q1, turning_radius)
    
    path = dubins.path(q0, q1, turning_radius, 1)
    print("have path", path)
    configurations, _ = path.sample_many(step_size)

    return [(c[0], c[1]) for c in configurations]

def rotate_list_of_points(points: [], angle, pivot_index=2):

    if(len(points) < 3):
        return

    rad = math.radians(angle)
    pivot = points[pivot_index]

    rotated_points = [Helper.rotate_point(pivot, point, rad) for point in points]

    return rotated_points

def get_xy_as_polynomial(points: []):
    """turn three points into curve
    
    Arguments:
        points {[x, y]} -- array of 3 points given as [x, y]

    Returns:
        polynomial, x_list
    """
    x_list= [p[0] for p in points]
    y_list= [p[1] for p in points]

    # fit n-degree polynomial - 2 in our case
    z = np.polyfit(x_list,y_list, 2)
    polynomial1d = np.poly1d(z)

    return polynomial1d, x_list

def poly_to_coordinates(polynomial1d, x_list):
    # calculate new x's and y's
    x_new = np.linspace(x_list[0], x_list[-1], 10)
    y_new = polynomial1d(x_new)

    return x_new, y_new

def coordinates_to_curvature(polynomial1d, x_space):
    der1deg = np.polyder(polynomial1d, 1)
    der2deg = np.polyder(polynomial1d, 2)

    return [Helper.signed_curvature(t, der1deg, der2deg) for t in x_space]

def demo_rotating(path, x_list, y_list, pivot_index=0):
    """Demo function to present path projecting
    """
    starting_angle = Helper.get_angle_from_3pts(path[0], path[1], path[2])
    # print(starting_angle)
    rotated_initial_points = rotate_list_of_points(path[2:5], starting_angle, pivot_index)
    rx= [x[0] for x in rotated_initial_points]
    ry= [x[1] for x in rotated_initial_points]
    plt.plot(rx, ry, 'orange')
    plt.plot(x_list[0:3], y_list[0:3], color='black', linestyle='--')
    plt.plot(x_list[2:5], y_list[2:5], 'black')
    plt.show()

def demo_rotating_all(path, x_list, y_list, pivot_index=0):
    starting_angle = Helper.get_angle_from_3pts(path[0], path[1], path[2])
    # print(starting_angle)
    rotated_initial_points = rotate_list_of_points(path[2:5], starting_angle, pivot_index)
    rotated_whole_path = rotate_list_of_points(path[2:], starting_angle, pivot_index)
    rx= [x[0] for x in rotated_whole_path]
    ry= [x[1] for x in rotated_whole_path]
    plt.plot(rx, ry, 'orange')
    plt.plot(x_list[0:3], y_list[0:3], color='black', linestyle='--')
    plt.plot(x_list, y_list, 'black')
    plt.show()


def radius_to_pwm(radius):
    return 22356.72*radius**-0.92


def calculate_curvatures_and_send_path():
    path = path_holder.path
    
    x_list= [x[0] for x in path]
    y_list= [x[1] for x in path]

    starting_angle = Helper.get_angle_from_3pts(path[0], path[1], path[2])
    rotated_initial_points = rotate_list_of_points(path[2:5], starting_angle, 0)
    
    curvature = []
    angles = []
    radius_list = []

    angle_sum = 0

    for i in range(len(path)-3):
        rotation_angle = Helper.get_angle_from_3pts(path[i], path[i+1], path[i+2])
        angles.append(rotation_angle)
        angle_sum += rotation_angle
        rotated_path = rotate_list_of_points(path[i:i+3], angle_sum, 0)

        try:
            circle_center, circle_radius = Helper.find_circle(path[i][0], path[i][1], path[i+1][0], path[i+1][1], path[i+2][0], path[i+2][1])
            radius_list.append(math.floor(radius_to_pwm(circle_radius)))
        except:
            print("failed establishing circle")

        xr= [x[0] for x in rotated_path]
        yr= [x[1] for x in rotated_path]

        polynomial, x_poly_space = get_xy_as_polynomial(rotated_path)
        x_space, y = poly_to_coordinates(polynomial, x_poly_space)
        curv = coordinates_to_curvature(polynomial, x_space)
        curvature.append(np.mean(curv))

    # plt.plot(curvature)
    # plt.show(block=False)

    left_right_curvature = [1 if x > 0 else 2 for x in curvature]

    radius_list = [[rad, curv] for rad, curv in zip(radius_list, left_right_curvature)]
    print("radius_list", radius_list)
    frames = Helper.speeds_to_json_frames(radius_list)

    frames.insert(0, path_holder.get_stopping_frame())

    populate_to_ros(use_ros, frames)


def main(use_ros=False):

    starting_position, starting_angle = starting_procedure()

    path_holder.update_path(dubins_path(starting_position[0], starting_position[1], starting_angle-90,
        FINAL_POINT_X,FINAL_POINT_Y,FINAL_THETA,
        turning_radius = HOST_TURNING_RADIUS, step_size = 5))

    print("PATH ESTABLISHED")
    # path_holder.update_path(dubins_path(0, 0, 180,
    #             -100,100,180,
    #         turning_radius = HOST_TURNING_RADIUS, step_size = 5))

    name_prefix = "Prosto, lewo"
    now = datetime.datetime.now()
    path = path_holder.path
    path_as_np = np.asarray(path)
    # pd.DataFrame(path_as_np).to_csv("path/to/file.csv")
    np.savetxt(f'{now.hour}:{now.minute}__{starting_position[0]}-{starting_position[1]}-{starting_angle}_to_{FINAL_POINT_X}-{FINAL_POINT_Y}-{FINAL_THETA}.csv', path_as_np, delimiter=",")
    x_list= [x[0] for x in path]
    y_list= [x[1] for x in path]
    # if (starting_position[0] < FINAL_POINT_X):
    #     path.reverse()

    plt.plot(x_list, y_list)
    plt.plot(x_list[0], y_list[0], 'r', marker=(2, 0, starting_angle+90), markersize=20, linestyle='None')
    plt.axis('equal')
    plt.savefig(f'path_{starting_position[0]}-{starting_position[1]}-{starting_angle}_to_{FINAL_POINT_X}-{FINAL_POINT_Y}-{FINAL_THETA}.png')
    plt.show()

    # demo_rotating_all(path, x_list, y_list)

    # starting_angle = Helper.get_angle_from_3pts(path[0], path[1], path[2])
    # rotated_initial_points = rotate_list_of_points(path[2:5], 180, 0)
    # print("rotated_initial_points", rotated_initial_points)
    # first=Helper.get_angle_from_3pts(path[0], path[1], path[2])
    # rotated = Helper.get_angle_from_3pts(rotated_initial_points[0], rotated_initial_points[1], rotated_initial_points[2])
    # print("f", first,"r",  rotated)

    curvature = []
    angles = []
    radius_list = []
    right_left = []
    angle_sum = 0


    for i in range(len(path)-3):
        rotation_angle = Helper.get_angle_from_3pts(path[i], path[i+1], path[i+2])
        angles.append(rotation_angle)
        angle_sum += rotation_angle
        rotated_path = rotate_list_of_points(path[i:i+3], angle_sum, 0)

        right_left.append(Helper.determine_right_left(path[i], path[i+1], path[i+2]))

        # x_list_rot= [x[0] for x in rotated_path[i:i+3]]
        # y_list_rot= [x[1] for x in rotated_path[i:i+3]]

        # plt.plot(x_list_rot, y_list_rot, 'r')
        # plt.show()

        try:
            circle_center, circle_radius = Helper.find_circle(path[i][0], path[i][1], path[i+1][0], path[i+1][1], path[i+2][0], path[i+2][1])
            radius_list.append(math.floor(radius_to_pwm(circle_radius)))
        except:
            print("failed establishing circle")

        xr= [x[0] for x in rotated_path]
        yr= [x[1] for x in rotated_path]


        polynomial, x_poly_space = get_xy_as_polynomial(rotated_path)
        x_space, y = poly_to_coordinates(polynomial, x_poly_space)
        curv = coordinates_to_curvature(polynomial, x_space)
        curvature.append(np.mean(curv))

    # reset = Helper.generate_stopping_frames(10)
    run = Helper.generate_straight_run_frames(10)
    
    # radius_list.insert()

    plt.plot(curvature)
    # plt.plot(angles)
    plt.plot(right_left)
    plt.show()

    radius_list = [[rad, curv] for rad, curv in zip(radius_list, right_left)]
    print(radius_list)
    frames = Helper.speeds_to_json_frames(radius_list)

    for r in run:
        frames.insert(0, r)

    # for res in reset:
    #     frames.insert(0, res)

    print("curvature: ", len(curvature), "Sradius_list: ", len(radius_list))
    json_vals = json.dumps(frames)

    with open('/home/wojciech/catkin_ws/src/beginner_tutorials/scripts/dubins_output.json', 'w') as outfile:
        json.dump(json_vals, outfile)

    print("FINISHED!")

    populate_to_ros(use_ros, frames)

def populate_to_ros(use_ros, frames):
    if(use_ros):
        print("USE ROS")
        # print("INIT ROS NODE")
        # rospy.init_node('talker', anonymous=True)
        print("INITIALIZED!") 
        # listener()
        publish_steering(frames)

def reset_odometry():
    print("INIT RESET ODOMETRY")
    rospy.init_node("super_node")
    publisher = rospy.Publisher('CoTylkoChcesz', String, queue_size=10)
    rate = rospy.Rate(50) # Hz
    for i in range(20):
        publisher.publish("STOP")
        rate.sleep()    

def publish_steering(frames, publisher = None):
    print("INIT PUBLISHER")
    if(publisher == None):
        publisher = rospy.Publisher('STalkerIn', String, queue_size=10)
    rate = rospy.Rate(50) # Hz
    while not rospy.is_shutdown():
        i = 0
        for frame in frames:
            i+=1
            # print(i)
            rospy.loginfo(frame)
            publisher.publish(json.dumps(frame))
            rate.sleep()
        break

class RobotSimulator:

    def __init__(self):
        self.counter = 0

    def move(self):
        self.counter += 10

simulate_robot = RobotSimulator()

def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard")
    print("callback")
    coordinates = data.data
    if(point_approximator.update((coordinates[0]/10, coordinates[1]/10))):
        if(host_arrived(point_approximator.get_last_approximated_position(), 100)):
            print("HOST ARRIVED")
            path_holder.clean_path()
            publish_steering(Helper.generate_stopping_frames(20))
            sub_handler.unsubscribe()
            rospy.signal_shutdown("Host finished the route")

        elif(host_left_path(point_approximator.get_last_approximated_position(),
                (coordinates[0]/10, coordinates[1]/10), 100)): 
            print("HOST LEFT PATH")

            last_pos = point_approximator.get_last_n_approximated_positions(2)
            host_theta = Helper.angle_between_two_points(last_pos[0], last_pos[1])

            recalculate_path(int(coordinates[0]), int(coordinates[1]), host_theta) 
            calculate_curvatures_and_send_path()

def host_arrived(current_position: tuple, maximum_distance: int):
    return (Helper.euclidean_distance(FINAL_POINT_X, FINAL_POINT_Y,
     current_position[0], current_position[1]) < maximum_distance)

def host_left_path(theoretic_host_position: tuple, current_position: tuple, maximum_distance: int):
    print("host_position", theoretic_host_position, "current_position", current_position)
    return (Helper.euclidean_distance(theoretic_host_position[0], theoretic_host_position[1],
     current_position[0], current_position[1]) > maximum_distance)

def recalculate_path(host_x, host_y, host_theta):
    path_holder.update_path(dubins_path(host_x, host_y, host_theta, FINAL_POINT_X, 
        FINAL_POINT_Y, FINAL_THETA, turning_radius = HOST_TURNING_RADIUS, step_size = 5))

def listener():
    print("LISTEN")
    if(sub_handler.sub is not None):
        sub_handler.unsubscribe()
    sub_handler.subscribe("Two_Way_Ranging")

    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()

starting_point_approximator = PointApproximator(100)

def starting_procedure() -> (tuple, float):
    """Get starting position and angle by measuring positions for x seconds,
    then moving platform, and measuring position once again
        
    Returns:
        [tuple] -- [initial position]
        [float] -- [initial angle]
    """
    print("STARTING PROCEDURE")
    try:
        rospy.init_node("uwb_pos_getter")
    except:
        print("initialized earlier")

    time_to_drive = 100
    how_many_frames = 40

    print("STARTING")
    starting_pos = get_UWB_position()
    # starting_pos = [0, 0]
    print("ZERO POSITION: ", starting_pos)
    starting_run = Helper.generate_straight_run_frames(how_many_frames)
    publish_steering(starting_run)
    print("FORWARD RUN")
    # forward_roll_run()

    print("END FORWARD RUN")

    time.sleep(5)

    next_pos = get_UWB_position()
    # next_pos = [0, 20]
    # next_pos = starting_pos
    # next_pos[0] += 100
    print("next post", next_pos)
    angle = Helper.angle_between_two_points(starting_pos, next_pos)
    print("AFTER FORWARD ROLL: ", next_pos, " ANGLE: ", angle)
    return next_pos, Helper.angle_between_two_points(starting_pos, next_pos)

def forward_roll_run():
    print("forward_roll_run")
    odometry_sub_handler = SubscriptionHandler(odometry_callback)
    odometry_sub_handler.subscribe("odometry/encoders_pose", PoseWithCovarianceStamped)
    try:
        rospy.init_node("uwb_pos_getter")
    except:
        print("initialized earlier")

    publisher = rospy.Publisher('STalkerIn', String, queue_size=10)
    while(pos_holder.run_flag):
        # print('pos holder: ', pos_holder.starting_position_roll)
        publish_steering(Helper.generate_straight_run_frames(5), publisher=publisher)
    publish_steering(Helper.generate_stopping_frames(1), publisher=publisher)
    odometry_sub_handler.unsubscribe()
    rospy.signal_shutdown("end")

def odometry_callback(data):
    print(data.pose.pose.position.x)
    pos_holder.odometry_position.append((data.pose.pose.position.x, data.pose.pose.position.y))
    if(data.pose.pose.position.x > 1):
        pos_holder.run_flag = False

def get_UWB_position():
    starting_point_approximator.finished_updating = False
    starting_points_sub_handler = SubscriptionHandler(starting_points_callback)
    starting_points_sub_handler.subscribe("Two_Way_Ranging")
    print("Waiting for points approximator...")
    # starting_point_approximator.filled_counter = 0
    while(not starting_point_approximator.finished_updating):
        # print("Updating")
        _ = ''
    print("Finished!")
    starting_points_sub_handler.unsubscribe()
    return starting_point_approximator.get_last_approximated_position()

def starting_points_callback(data):
    if(starting_point_approximator.update((data.data[0]/10, data.data[1]/10))):
        starting_point_approximator.finished_updating = True

sub_handler = SubscriptionHandler(callback)
path_holder = PathHolder()

def on_press(key):
    # print("\n\n\n\n\n KEY EVENT")
    if key == keyboard.Key.esc:
        return False  # stop listener
    try:
        k = key.char  # single-char keys
    except:
        k = key.name  # other keys
    if k in ['@']:  # keys of interest
        # self.keys.append(k)  # store it in global-like variable
        path_holder.clean_path()
        publish_steering(Helper.generate_stopping_frames(10))
        rospy.signal_shutdown("Stopping procedure initiated")
        print('\n\nSTOPPING PROCEDURE\tkey pressed: ' + k)
        # return False  # stop listener; remove this if want more keys


if __name__ == '__main__':
    use_ros = True
    keyboard_listener = keyboard.Listener(on_press=on_press)
    keyboard_listener.start()  # start to listen on a separate thread
    # listener.join()  # remove if main thread is polling self.keys
    # test_angle = Helper.get_angle_from_3pts((1, 0),(1, 1),(0, 2))
    # print("TEST ANGLE: ", test_angle)
    reset_odometry()
    main(use_ros=use_ros)
