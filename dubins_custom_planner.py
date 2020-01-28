import dubins
import matplotlib.pyplot as plt
import numpy as np
import json
import math
from Helper import Helper
from dubins_talker import DubinsTalker
import rospy
from std_msgs.msg import String
from math import sqrt 
from PointApproximator import PointApproximator

# SET GLOBAL POINT APPROXIMATOR
point_approximator = PointApproximator()

# SET GLOBAL VARIABLES
FINAL_POINT_X = 200 # [cm]
FINAL_POINT_Y = 200 # [cm]
FINAL_THETA = 180 # [degrees]
HOST_TURNING_RADIUS = 68.0 # [degrees]

path = []

def euclidean_distance(x1, y1, x2, y2):
    return math.sqrt( (x2 - x1)**2 + (y2 - y1)**2 )

# Function to find the circle on  
# which the given three points lie  
def find_circle(x1, y1, x2, y2, x3, y3) : 
    x12 = x1 - x2;  
    x13 = x1 - x3;  
  
    y12 = y1 - y2;  
    y13 = y1 - y3;  
  
    y31 = y3 - y1;  
    y21 = y2 - y1;  
  
    x31 = x3 - x1;  
    x21 = x2 - x1;  
  
    # x1^2 - x3^2  
    sx13 = pow(x1, 2) - pow(x3, 2);  
  
    # y1^2 - y3^2  
    sy13 = pow(y1, 2) - pow(y3, 2);  
  
    sx21 = pow(x2, 2) - pow(x1, 2);  
    sy21 = pow(y2, 2) - pow(y1, 2);  
  
    try:
        f = (((sx13) * (x12) + (sy13) * 
            (x12) + (sx21) * (x13) + 
            (sy21) * (x13)) // (2 * 
            ((y31) * (x12) - (y21) * (x13)))); 
                
        g = (((sx13) * (y12) + (sy13) * (y12) + 
            (sx21) * (y13) + (sy21) * (y13)) // 
            (2 * ((x31) * (y12) - (x21) * (y13))));  
    
        c = (-pow(x1, 2) - pow(y1, 2) - 
            2 * g * x1 - 2 * f * y1);  
        # eqn of circle be x^2 + y^2 + 2*g*x + 2*f*y + c = 0  
        # where centre is (h = -g, k = -f) and  
        # radius r as r^2 = h^2 + k^2 - c  
        h = -g;  
        k = -f;  
        sqr_of_r = h * h + k * k - c;  
    
        # r is the radius  
        r = round(sqrt(sqr_of_r), 5);  
    
        # print("Centre = (", h, ", ", k, ")");  
        # print("Radius = ", r);  
        return (h, k), r
    except:
        print("failed calculating ")


def signed_curvature(x, f1, f2):
    return f2(x)*(1 + f1(x)**2)**-1.5

def rotate(origin: [], point: [], angle):
    """
    Rotate a point counterclockwise by a given angle around a given origin.

    The angle should be given in radians.
    
    Arguments:
        origin {[x, y]}
        point {[x, y]}
        angle {int}
    """
    ox, oy = origin
    px, py = point

    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    return qx, qy

def dubins_path(x0, y0, theta0, x1, y1, theta1, turning_radius = 3.0, step_size = 0.1):
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
    q0 = (x0, y0, theta0)
    q1 = (x1, y1, theta1)

    path = dubins.path(q0, q1, turning_radius, 3)
    configurations, _ = path.sample_many(step_size)

    return [(c[0], c[1]) for c in configurations]


def get_angle(p0, p1=np.array([0,0]), p2=None):
    ''' compute angle (in degrees) for p0p1p2 corner
    Inputs:
        p0,p1,p2 - points in the form of [x,y]
    '''
    if p2 is None:
        p2 = p1 + np.array([1, 0])
    v0 = np.array(p0) - np.array(p1)
    v1 = np.array(p2) - np.array(p1)

    angle = np.math.atan2(np.linalg.det([v0,v1]),np.dot(v0,v1))
    return np.degrees(angle) + 180

def rotate_list_of_points(points: [], angle, pivot_index=2):

    if(len(points) < 3):
        return

    rad = math.radians(angle)
    pivot = points[pivot_index]

    rotated_points = [rotate(pivot, point, rad) for point in points]

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

    return [signed_curvature(t, der1deg, der2deg) for t in x_space]

def demo_rotating(path, x_list, y_list, pivot_index=0):
    """Demo function to present path projecting
    """
    starting_angle = get_angle(path[0], path[1], path[2])
    # print(starting_angle)
    rotated_initial_points = rotate_list_of_points(path[2:5], starting_angle, pivot_index)
    rx= [x[0] for x in rotated_initial_points]
    ry= [x[1] for x in rotated_initial_points]
    plt.plot(rx, ry, 'orange')
    plt.plot(x_list[0:3], y_list[0:3], color='black', linestyle='--')
    plt.plot(x_list[2:5], y_list[2:5], 'black')
    plt.show()

def demo_rotating_all(path, x_list, y_list, pivot_index=0):
    starting_angle = get_angle(path[0], path[1], path[2])
    # print(starting_angle)
    rotated_initial_points = rotate_list_of_points(path[2:5], starting_angle, pivot_index)
    rotated_whole_path = rotate_list_of_points(path[2:], starting_angle, pivot_index)
    rx= [x[0] for x in rotated_whole_path]
    ry= [x[1] for x in rotated_whole_path]
    plt.plot(rx, ry, 'orange')
    plt.plot(x_list[0:3], y_list[0:3], color='black', linestyle='--')
    plt.plot(x_list, y_list, 'black')
    plt.show()

def detect_outlier(data_1):
    outliers=[]
    
    threshold=3
    mean_1 = np.mean(data_1)
    std_1 =np.std(data_1)
    
    
    for ind, y in enumerate(data_1):
        z_score= (y - mean_1)/std_1 
        if np.abs(z_score) > threshold:
            outliers.append(ind)
    return outliers

def radius_to_pwm(radius):
    return 22356.72*radius**-0.92

def main(path, use_ros=False):
    # ang = get_angle([0,0], [2,0], [0,1])
    # print(ang)

    x_list= [x[0] for x in path]
    y_list= [x[1] for x in path]
    # plt.plot(x_list, y_list)

    # demo_rotating_all(path, x_list, y_list)

    starting_angle = get_angle(path[0], path[1], path[2])
    rotated_initial_points = rotate_list_of_points(path[2:5], starting_angle, 0)
    
    curvature = []
    angles = []
    radius_list = []

    angle_sum = 0

    for i in range(len(path)-3):
        rotation_angle = get_angle(path[i], path[i+1], path[i+2])
        angles.append(rotation_angle)
        angle_sum += rotation_angle
        rotated_path = rotate_list_of_points(path[i:i+3], angle_sum, 0)

        try:
            circle_center, circle_radius = find_circle(path[i][0], path[i][1], path[i+1][0], path[i+1][1], path[i+2][0], path[i+2][1])
            # print(radius_to_pwm(circle_radius))
            radius_list.append(math.floor(radius_to_pwm(circle_radius)))
            # circle=plt.Circle(circle_center,circle_radius, color='b', fill=False)
            # if(circle is not None):
            #     plt.gcf().gca().add_artist(circle)
        except:
            print("failed establishing circle")

        xr= [x[0] for x in rotated_path]
        yr= [x[1] for x in rotated_path]

        plt.plot(x_list[i:i+3], y_list[i:i+3], 'black')
        # plt.plot(xr, yr, 'orange')
        # plt.show()

        # print(xr, yr)

        polynomial, x_poly_space = get_xy_as_polynomial(rotated_path)
        x_space, y = poly_to_coordinates(polynomial, x_poly_space)
        curv = coordinates_to_curvature(polynomial, x_space)
        curvature.append(np.mean(curv))

        # plt.plot(x_space, curv, 'g')
        # plt.show()

    # print("amount of frames:", len(radius_list))
    # print("plot ")
    
    plt.axis('equal')
    plt.grid()
    plt.show()


    plt.plot(radius_list)
    plt.show()

    plt.plot(curvature)
    plt.show()

    # out = detect_outlier(curvature)
    # while(len(out) is not 0):
    #     out = detect_outlier(curvature)
    #     for o in out:
    #         print("outlier:", o)
    #         curvature.pop(o)
    
    # curvature_rescaled = Helper().rescaleTableToNewValues(curvature, 0, 480)
    # print("curvature_rescaled", len(curvature_rescaled))
    # plt.plot(curvature_rescaled)
    # plt.show()

    radius_list = [[r, 1] for r in radius_list]
    frames = Helper.speeds_to_json_frames(radius_list)
    json_vals = json.dumps(frames)

    with open('/home/wojciech/catkin_ws/src/beginner_tutorials/scripts/dubins_output.json', 'w') as outfile:
        json.dump(json_vals, outfile)

    print("FINISHED!")
    test = False
    if(test):
        frames = [
            {"MovementFrameTurnPropulsion": {

                "propulsionValue": 80,
                "turnValue": 400,
                "propulsionDirection": 1,
                "turnDirection": 1,
                "timeToDrive": 200,
                "isQueued": 1
            }}
        ] * 20

    if(use_ros):
        print("INIT ROS NODE")
        rospy.init_node('talker', anonymous=True)
        print("INITIALIZED!")
        listener()
        # publish_steering(frames)

def publish_steering(frames):
        print("INIT PUBLISHER")
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

class Dupa:

    def __init__(self):
        self.counter = 0

    def add(self):
        self.counter += 10

dupa = Dupa()

def callback(data):
    # print("CALLBACK")
    # rospy.loginfo(rospy.get_caller_id() + "I heard")
    json_data = json.loads(data.data)
    for data in json_data['AccelerometerFrames']:
        if(point_approximator.update((data.get('xAxis'), data.get('yAxis')))):
            print("UPDATE EVENT")
            dupa.add()
            print(dupa.counter)
            # TODO: CALCULATE WANTED POSITION
            if(host_left_path(point_approximator.get_last_position(), (dupa.counter, 0))): 
                print("HOST LEFT PATH")
                path = recalculate_path(data.get('xAxis'), data.get('yAxis')) 

def host_left_path(host_position: tuple, current_position: tuple):
    return (euclidean_distance(host_position[0], host_position[1],
     current_position[0], current_position[1]) > 100)

def recalculate_path(host_x, host_y):
    return dubins_path(host_x, host_y, 0, FINAL_POINT_X, FINAL_POINT_Y, FINAL_THETA, turning_radius = HOST_TURNING_RADIUS, step_size = 5)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    # rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("AccelerometerFrame", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    use_ros = True
    path = dubins_path(0, 0, 0, FINAL_POINT_X, FINAL_POINT_Y, 180, turning_radius = HOST_TURNING_RADIUS, step_size = 5)

    main(path, use_ros=use_ros)

