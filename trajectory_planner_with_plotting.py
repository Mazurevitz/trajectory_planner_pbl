import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
import numpy as np
import numpy.polynomial.polynomial as poly
import autograd.numpy as autonp
from autograd import grad
from Helper import Helper
import json
import rospy
from std_msgs.msg import String
import json
import os
from pynput import keyboard

fig = plt.figure()
ax = fig.add_subplot(111)
ax.set_xlim([0, 5])
ax.set_ylim([-5, 5])

dotsx = []
dotsy = []


# class PolyCalculator:
#     def __init__(self):

class DataHolder:

    def __init__(self):
        self.frames = []


dh = DataHolder()

input_name = "STalkerIn"
os.system('xset r off')

def on_press(key):
    try:
        if(key.char == 'x'):
            print('alphanumeric key {0} pressed'.format(
                key.char))
            dh.frames = []
            pub = rospy.Publisher(input_name, String, queue_size=10)
            pub.publish({
                    "propulsionValue": 0,
                    "turnValue": 0,
                    "propulsionDirection": 1,
                    "turnDirection": 0,
                    "timeToDrive": 1000,
                    "isQueued": 0
                })

    except AttributeError:
        print('special key {0} pressed'.format(
            key))

# def exponent(x):
#     return 1/(1 + autonp.exp(-x))

def poly_to_function(x):
    x = x[::-1]
    return lambda coeff: x[0] ** 4 + x[1] ** 3 + x[2] ** 2 + x[3]

def curvature(x, f1, f2):
    return abs(f2(x))*(1 + f1(x)**2)**-1.5


def signed_curvature(x, f1, f2):
    return f2(x)*(1 + f1(x)**2)**-1.5


def parametric_curve_curvature(x, f1x, f2x, f1y, f2y):
    return f2x(x)*(1 + f1x(x)**2)**-1.5 # to fix


data_to_publish = []

def plot_polynomial(x: [], y: []) -> []: 
    # calculate polynomial
    z = np.polyfit(x, y, 4)
    polynomial1d = np.poly1d(z)

    # calculate new x's and y's
    x_new = np.linspace(x[0], x[-1], 1000)
    y_new = polynomial1d(x_new)

    der1deg = np.polyder(polynomial1d, 1)
    der2deg = np.polyder(polynomial1d, 2)

    curvature = [signed_curvature(t, der1deg, der2deg) for t in x_new]
    curvature_rescaled = Helper().rescaleTableToNewValues(curvature, 0, 400)
    print(curvature)
    print("\n\n", curvature_rescaled)

    dh.frames = Helper.speeds_to_json_frames(curvature_rescaled)
    json_vals = json.dumps(Helper.speeds_to_json_frames(curvature_rescaled))
    print(json_vals)
    with open('/home/wojciech/catkin_ws/src/beginner_tutorials/scripts/steering_test.json', 'w') as outfile:
        json.dump(json_vals, outfile)

    print("!saved")
    plt.plot(x_new, y_new, 'b--')
    plt.plot(x_new, curvature, 'g-')
    plt.show()

def plot_spline(x, y):
    print("spline")
    f2 = interp1d(dotsx, dotsy, kind='cubic')
    plt.plot(dotsx, f2(dotsx), '--')

def speeds_to_json_frames(speeds_with_directions: tuple):
    dirs_json = []
    for angle, direction in speeds_with_directions:
        dirs_json.append({"MovementFrameTurnPropulsion": {

            "propulsionValue": 75,
            "turnValue": angle,
            "propulsionDirection": 1,
            "turnDirection": direction,
            "timeToDrive": 50,
            "isQueued": 1
        }})

    return dirs_json


def onclick(event):
    print('button=%d, x=%d, y=%d, xdata=%f, ydata=%f' %
          (event.button, event.x, event.y, event.xdata, event.ydata))
    dotsx.append(event.xdata)
    dotsy.append(event.ydata)


    last_x_points = 4
    if(len(dotsx) >= last_x_points and len(dotsx) % last_x_points == 0):
        plot_polynomial(dotsx[-last_x_points:], dotsy[-last_x_points:])
        # plot_spline(dotsx[-last_x_points:], dotsy[-last_x_points:])
    
    plt.plot(event.xdata, event.ydata, 'bo')
    plt.plot(dotsx, dotsy, 'b-')
    fig.canvas.draw()
    plt.show()

def press(event):
    print('press', event.key)
    if (event.key == 'x'):
        plt.close()
    if (event.key == 'r'):
        clear_dots()
        
    
def clear_dots():
    dotsx.clear()
    dotsy.clear()

def interrupt(event):
    if(event.key == 'x'):
        print("\n\nSTOP!")
        pub = rospy.Publisher(input_name, String, queue_size=10)
        pub.publish({"MovementFrameTurnPropulsion":{

                "propulsionValue": 0,
                "turnValue": 0,
                "propulsionDirection": 1,
                "turnDirection": 0,
                "timeToDrive": 100,
                "isQueued": 0
        }})


def talker():
    pub = rospy.Publisher(input_name, String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(50) # 10hz

    cid = fig.canvas.mpl_connect('button_press_event', onclick)

    # fig.canvas.mpl_connect('key_press_event', interrupt)
    plt.show()

    # listener = keyboard.Listener(on_press=on_press)
    # listener.start()
    i = 0
    jsonFrames = json.dumps(dh.frames)
    # print(jsonFrame)
    while not rospy.is_shutdown():
        for i in range(0, 500):
            static_frame = ({"MovementFrameTurnPropulsion":{
                "propulsionValue": 75,
                "turnValue": 0,
                "propulsionDirection": 1,
                "turnDirection": 0,
                "timeToDrive": 50,
                "isQueued": 1
            }})
            rospy.loginfo(static_frame)
            print(i)
            pub.publish(json.dumps(static_frame))
            rate.sleep()

        if (i < len(dh.frames)):
            rospy.loginfo(dh.frames[i])
            # print(dh.frames[i])
            pub.publish(json.dumps(dh.frames[i]))
            rate.sleep()
            i+=1



if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass


