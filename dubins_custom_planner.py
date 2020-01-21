import dubins
import matplotlib.pyplot as plt
import numpy as np
import json
import math
from Helper import Helper
from dubins_talker import DubinsTalker
import rospy
from std_msgs.msg import String

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

    path = dubins.path(q0, q1, turning_radius, 0)
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
    starting_angle = get_angle(path[0], path[1], path[2])
    print(starting_angle)
    rotated_initial_points = rotate_list_of_points(path[2:5], starting_angle, pivot_index)
    rx= [x[0] for x in rotated_initial_points]
    ry= [x[1] for x in rotated_initial_points]
    plt.plot(rx, ry, 'orange')
    plt.plot(x_list[0:3], y_list[0:3], color='black', linestyle='--')
    plt.plot(x_list[2:5], y_list[2:5], 'black')
    plt.show()

def demo_rotating_all(path, x_list, y_list, pivot_index=0):
    starting_angle = get_angle(path[0], path[1], path[2])
    print(starting_angle)
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


def main():
    # ang = get_angle([0,0], [2,0], [0,1])
    # print(ang)
    path = dubins_path(0, 0, 0, 4, 4, 90, turning_radius = 2.0, step_size = 0.1)

    x_list= [x[0] for x in path]
    y_list= [x[1] for x in path]
    # plt.plot(x_list, y_list)

    # demo_rotating_all(path, x_list, y_list)

    starting_angle = get_angle(path[0], path[1], path[2])
    rotated_initial_points = rotate_list_of_points(path[2:5], starting_angle, 0)
    
    curvature = []
    angles = []

    for i in range(len(path)-3):
        rotation_angle = get_angle(path[i], path[i+1], path[i+2])
        angles.append(rotation_angle)
        rotated_path = rotate_list_of_points(path[i:i+3], rotation_angle, 0)

        xr= [x[0] for x in rotated_path]
        yr= [x[1] for x in rotated_path]

        # plt.plot(x_list[i:i+3], y_list[i:i+3], 'black')
        # plt.plot(xr, yr, 'orange')
        # plt.show()

        # print(xr, yr)

        polynomial, x_poly_space = get_xy_as_polynomial(rotated_path)
        x_space, y = poly_to_coordinates(polynomial, x_poly_space)
        curv = coordinates_to_curvature(polynomial, x_space)
        curvature.append(np.mean(curv))

        # plt.plot(x_space, curv, 'g')
        # plt.show()

    # plt.show()

    out = detect_outlier(curvature)
    while(len(out) is not 0):
        out = detect_outlier(curvature)
        for o in out:
            print("pop")
            curvature.pop(o)

    curvature_rescaled = Helper().rescaleTableToNewValues(curvature, 0, 400)
    plt.plot(curvature_rescaled)
    plt.show()

    frames = Helper.speeds_to_json_frames(curvature_rescaled)
    json_vals = json.dumps(frames)

    with open('/home/wojciech/catkin_ws/src/beginner_tutorials/scripts/dubins_output.json', 'w') as outfile:
        json.dump(json_vals, outfile)

    print("FINISHED!")



    # dub_talker = DubinsTalker(frames)
    # dub_talker.talk()

    # plt.plot(x_list[:-3], curvature)
    # print(curvature)

    # plt.show()

def talker():
    publisher = rospy.Publisher('STalkerSteer', String, queue_size=10)
    print("Start talking from DubinsTalker")
    rospy.init_node('talker', anonymous=True)
    print("node initialized")
    rate = rospy.Rate(50) # 10hz
    print("rate set")
    # while not rospy.is_shutdown():
    #     for frame in frames:
    #         # rospy.loginfo(frame)
    #         publisher.publish(json.dumps(frame))
    #         rate.sleep()



if __name__ == '__main__':
    main()
    talker()
