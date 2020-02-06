# import klampt
# from klampt.model import trajectory
# from klampt import vis
# from klampt.vis.glinterface import GLPluginInterface
import numpy as np
from math import atan2, degrees, sqrt, atan, pow
import math

class Helper:
    def getSpeedFromVector(self, vector: list) -> float:
        return sqrt(pow(vector[0], 2) + pow(vector[2], 2))

    def getAngleFromVector(self, vector: list) -> float:
        return 0 if vector[2] is 0 else np.rad2deg(atan(vector[0]/vector[2])) 

    def defineNewValueAfterRescale(self, OldValue, OldMin, OldMax, NewMin, NewMax):
        return (int(((OldValue - OldMin) * (NewMax - NewMin)) / (OldMax - OldMin)) + NewMin, 2 if OldValue < 0 else 1)

    def rescaleTableToNewValues(self, table: list, newMin: int, newMax: int) -> list:
        tabMin = min(table)
        tabMax = max(table)
        return [self.defineNewValueAfterRescale(x, tabMin, tabMax, newMin, newMax) for x in table]

    @staticmethod
    def angle_between_two_points(pointA, pointB) -> float:
        changeInX = pointB[0] - pointA[0]
        changeInY = pointB[1] - pointA[1]
        return degrees(atan2(changeInY,changeInX)) #remove degrees if you want your answer in radians    

    @staticmethod
    def speeds_to_json_frames(speeds_with_directions):
        dirs_json = []
        for angle, direction in speeds_with_directions:
            dirs_json.append({"MovementFrameTurnPropulsion": {

                "propulsionValue": 100,
                "turnValue": angle,
                "propulsionDirection": 1,
                "turnDirection": direction,
                "timeToDrive": 61,
                "isQueued": 1
            }})

        return dirs_json

    @staticmethod
    def generate_stopping_frames(count) -> list:
        return [{"MovementFrameTurnPropulsion": {

                "propulsionValue": 0,
                "turnValue": 0,
                "propulsionDirection": 1,
                "turnDirection": 0,
                "timeToDrive": 100,
                "isQueued": 0
            }}]*count

    @staticmethod
    def generate_straight_run_frames(count) -> list:
        return [
            {"MovementFrameTurnPropulsion": {

                "propulsionValue": 60,
                "turnValue": 0,
                "propulsionDirection": 1,
                "turnDirection": 2,
                "timeToDrive": 100 ,
                "isQueued": 1
            }}
        ] * count

    @staticmethod
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

    @staticmethod
    def euclidean_distance(x1, y1, x2, y2):
        return sqrt( (x2 - x1)**2 + (y2 - y1)**2 )

    @staticmethod
    def rotate_point(origin: [], point: [], angle):
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

    @staticmethod
    def signed_curvature(x, f1, f2):
        return f2(x)*(1 + f1(x)**2)**-1.5

    @staticmethod
    def get_angle_from_3pts(p0, p1=np.array([0,0]), p2=None):
        ''' compute angle (in degrees) for p0,p1,p2 corner
        Inputs:
            p0,p1,p2 - points in the form of [x,y]
        '''
        if p2 is None:
            p2 = p1 + np.array([1, 0])
        v0 = np.array(p0) - np.array(p1)
        v1 = np.array(p2) - np.array(p1)

        angle = np.math.atan2(np.linalg.det([v0,v1]),np.dot(v0,v1))
        return np.degrees(angle) + 180

    @staticmethod
    def detect_outlier(data_1):
        """Method used to return outliers of given array
        
        Arguments:
            data_1 {list} -- list of data   
        
        Returns:
            [list] -- list of outliers
        """
        outliers=[]
        
        threshold=3
        mean_1 = np.mean(data_1)
        std_1 =np.std(data_1)
        
        
        for ind, y in enumerate(data_1):
            z_score= (y - mean_1)/std_1 
            if np.abs(z_score) > threshold:
                outliers.append(ind)
        return outliers

    @staticmethod
    def determine_right_left(s, f, point):
        dot = ((f[0] - s[0])*(point[1] - s[1]) - (f[1] - s[1])*(point[0] - s[0]))
        if (dot == 0):
            return 0
        elif (dot > 0):
            return 1
        else:
            return 2 