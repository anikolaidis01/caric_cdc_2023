################## Explorer Trajectory Code ##################
__author__ = "Andreas Anastasiou, Antonis Nikolaides"
__copyright__ = "Copyright (C) 2023 KIOS Center of Excellence"
__version__ = "7.0"
##############################################################

import rospy
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from caric_mission.srv import CreatePPComTopic
from visualization_msgs.msg import Marker
import numpy as np
import threading
import numba as nb

maxVel = 4.0
debug = False
TAG = ""
odom = Odometry()
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
mutex = threading.Lock()
non_visited_nodes_flag=False

visited_nodes =[]
obstacles_nodes = []
combined_array = []
#neighbors' offsets starting from same z plane counter-clockwise
offsets_all = [
    (-1,-1,0), (0,-1,0), (1,-1,0), (1,0,0), (1,1,0), (0,1,0), (-1,1,0), (-1,0,0), 
    (-1,-1,1), (0,-1,1), (1,-1,1), (1,0,1), (1,1,1), (0,1,1), (-1,1,1), (-1,0,1),(0,0,1),
    (-1,-1,-1), (0,-1,-1), (1,-1,-1), (1,0,-1), (1,1,-1), (0,1,-1), (-1,1,-1), (-1,0,-1),(0,0,-1)
]
offsets_cross = [
    (0,-1,0), (1,0,0), (0,1,0), (-1,0,0), (0,0,1), (0,0,-1)
]

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


def main():
# init
    global detPub, waypoint, command_thread, update_from_neighbor_thread, coordinates, target, grid_resolution, scenario
    global namespace, debug, adjacency, update, mutex, adjacency_final
    global visited_nodes
    try:
        namespace = rospy.get_param('namespace') # node_name/argsname
        scenario = rospy.get_param('scenario')
        debug = rospy.get_param('debug')
        grid_resolution = rospy.get_param('grid_resolution')
        set_tag("[" + namespace.upper() + " TRAJ SCRIPT]: ")
    except Exception as e:
        print(e)
        namespace = "jurong"
        scenario = 'mbs'
        debug = True
        set_tag("[" + namespace.upper() + " TRAJ SCRIPT]: ")


    rospy.init_node('multi_ppcom_pub', anonymous=True)

    # create command publisher
    detPub = rospy.Publisher("/"+namespace+"/detected_interest_points/", PointCloud2, queue_size=1)

    # Create a ppcom publisher
    # Wait for service to appear
    log_info("Waiting for ppcom")
    try: 
        rospy.wait_for_service('/create_ppcom_topic')
        # Create a service proxy
        create_ppcom_topic = rospy.ServiceProxy('/create_ppcom_topic', CreatePPComTopic)
        # Register the topic with ppcom router
        create_ppcom_topic(namespace, ['all'], '/'+namespace+'/detected_interest_points', 'sensor_msgs', 'PointCloud2')
        
        if namespace != 'jurong':
            rospy.Subscriber("/jurong/detected_interest_points/"+namespace+'/', PointCloud2, detectCallback)
        if namespace != 'raffles':
            rospy.Subscriber("/raffles/detected_interest_points/"+namespace+'/', PointCloud2, detectCallback)
        if namespace != 'changi':
            rospy.Subscriber("/changi/detected_interest_points/"+namespace+'/', PointCloud2, detectCallback)
        if namespace != 'sentosa':
            rospy.Subscriber("/sentosa/detected_interest_points/"+namespace+'/', PointCloud2, detectCallback)
        if namespace != 'nanyang':
            rospy.Subscriber("/nanyang/detected_interest_points/"+namespace+'/', PointCloud2, detectCallback)

    except e:
        print(e)
        print('Error while using PPCOM')