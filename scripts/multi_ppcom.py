################## Explorer Trajectory Code ##################
__author__ = "Andreas Anastasiou, Antonis Nikolaides"
__copyright__ = "Copyright (C) 2024 KIOS Center of Excellence"
__version__ = "7.0"
##############################################################

import rospy
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2
from visualization_msgs.msg import Marker
import numpy as np
import numba as nb
import math
import traceback

maxVel = 4.0
debug = False
TAG = ""
odom = Odometry()
position = Point()
target = 0
neighbors = PointCloud2()
command_thread = None
update_from_neighbor_thread = None
cmdPub = None
coordinates = None
target_yaw = 0.0
grid_resolution = 6
namespace = "jurong"
adjacency = np.zeros((2,2))
adjacency_final = np.zeros((2,2))
update=True
#neighbors' offsets starting from same z plane counter-clockwise

def odomCallback(msg):
    try:
        global odom, position
        odom = msg
        position = odom.pose.pose.position
    except Exception as e:
        print(f'\n\n\nERROR: In odom callback function! {e}')

def set_tag(tag):
    global TAG
    TAG = tag

def detectCallback(msg):
    global detPub
    points = Header()
    threshold = 1000
    if points.stamp.secs - msg.header.stamp.secs < threshold:
        detPub.publish(msg)
        
def log_info(info):
    global TAG, debug
    if debug:
        rospy.loginfo(TAG + f"{info}")
        publish_text_viz(TAG + f"{info}")

def publish_text_viz(msg):
    viz_pub = rospy.Publisher("/"+namespace+"/text_viz", Marker, queue_size=1)
    marker = Marker()
    marker.header.frame_id = "world"
    marker.type = marker.TEXT_VIEW_FACING
    marker.text = msg
    marker.action = marker.ADD
    marker.scale.x = 3.0
    marker.scale.y = 3.0
    marker.scale.z = 3.0
    marker.color.a = 1.0
    marker.color.r = 0.0
    if namespace == "jurong":
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.pose.position.y = -50.0
    else:
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.position.y = -57.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = 0.0
    marker.pose.position.z = 30.0
    try:
        viz_pub.publish(marker)
    except:
        print('Did not published Text adjacency')

def euclidean_distance_points(point1,point2):
    p1 = np.zeros((3,1))
    p1[0] = point1.x
    p1[1] = point1.y
    p1[2] = point1.z
    p2 = np.zeros((3,1))
    p2[0] = point2.x
    p2[1] = point2.y
    p2[2] = point2.z
    return math.sqrt( math.pow(p1[0]-p2[0],2) + math.pow(p1[1]-p2[1],2) + math.pow(p1[2]-p2[2],2))

def main():
# init
    global detPub, command_thread, update_from_neighbor_thread, coordinates, target, grid_resolution, scenario
    global namespace, debug, adjacency, update, adjacency_final, position, uav_distance_com
    try:
        namespace = rospy.get_param('namespace') # node_name/argsname
        scenario = rospy.get_param('scenario')
        debug = rospy.get_param('debug')
        grid_resolution = rospy.get_param('grid_resolution')
        uav_distance_com = rospy.get_param('uav_distance_com')
        set_tag("[" + namespace.upper() + " TRAJ SCRIPT]: ")
    except Exception as e:

        print(e)
        namespace = "ERROR"
        scenario = 'mbs'
        uav_distance_com = 1
        debug = True
        set_tag("[" + namespace.upper() + " TRAJ SCRIPT]: ")

    rospy.init_node('multi_ppcom_pub', anonymous=True)

    rospy.Subscriber("/"+namespace+"/ground_truth/odometry", Odometry, odomCallback)

    # create command publisher
    detPub = rospy.Publisher("/"+namespace+"/detected_interest_points/", PointCloud2, queue_size=1)
    
    neighbors = PointCloud2()
    try:
        neighbors = rospy.wait_for_message("/"+namespace+"/nbr_odom_cloud", PointCloud2)
    except Exception as e:
        print(f'\nError in initialize of ppcom {e}')

    while not rospy.is_shutdown():
        try:
            for _, point in enumerate(sensor_msgs.point_cloud2.read_points(neighbors, skip_nans=True)):
                point_message = Point()
                point_message.x, point_message.y, point_message.z = point[0], point[1], point[2]
                d = euclidean_distance_points(position,point_message)
                if d>=uav_distance_com:
                    continue
                else:
                    # print('\n\n'+namespace+' has communication in Distance:'+str(d))
                    if namespace != 'jurong':
                        rospy.Subscriber("/jurong/detected_interest_points/", PointCloud2, detectCallback)
                    if namespace != 'raffles':
                        rospy.Subscriber("/raffles/detected_interest_points/", PointCloud2, detectCallback)
                    if namespace != 'changi':
                        rospy.Subscriber("/changi/detected_interest_points/", PointCloud2, detectCallback)
                    if namespace != 'sentosa':
                        rospy.Subscriber("/sentosa/detected_interest_points/", PointCloud2, detectCallback)
                    if namespace != 'nanyang':
                        rospy.Subscriber("/nanyang/detected_interest_points/", PointCloud2, detectCallback)
        except Exception as e:
            print(f'\nERROR in distance multicom {e}')
            print(e)


    # except e:
    #     print(e)
    #     print('Error while using PPCOM')


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("terminating...")
    except Exception as e:
        traceback.print_exc()
    finally:
        exit()