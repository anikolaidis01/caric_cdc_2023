################# Explorer Path Planning Code ################
__author__ = "Antonis Nikolaides"
__copyright__ = "Copyright (C) 2024 Kios Center of Excellence"
__version__ = "7.1" # Updated Version
##############################################################

import sys
import rospy
from std_msgs.msg import Int16MultiArray, Bool, Float32, Duration, String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2, PointCloud
import sensor_msgs.point_cloud2
from kios_solution.msg import area, norms
from caric_mission.srv import CreatePPComTopic
from visualization_msgs.msg import MarkerArray
from scipy.spatial import Delaunay
import numpy as np
import math
import traceback
import numba as nb
import random as rand
import time

build_map = True
repeat = True
debug = False
TAG = ""
odom = Odometry()
position = Point()
target = 0
command_thread = None
cmdPub = None
coordinates = None
grid_resolution = 6
namespace = "jurong"
arrived = False
remaining_time = sys.maxsize
neighbors = PointCloud2()
# uav_distance_com=50

# --- Define all exploration areas ---
# You can add, remove, or edit areas here.
# Format: (Name, (Center_X, Center_Y, Center_Z), (Size_X, Size_Y, Size_Z))

AREA_A_CENTER = (-200, -62, 15.0)
AREA_A_SIZE   = (60.0,    60.0, 30.0)

AREA_B_CENTER = (37, 180, 15.0)
AREA_B_SIZE   = (60.0,    60.0, 30.0)

# Placeholder locations for new areas - CHANGE THESE
AREA_C_CENTER = (41, 35, 15.0)
AREA_C_SIZE   = (20.0, 20.0, 30.0)

# AREA_D_CENTER = (50.0,  50.0, 6.0)
# AREA_D_SIZE   = (8.0,   8.0, 4.0)

# AREA_E_CENTER = (-50.0, -50.0, 7.0)
# AREA_E_SIZE   = (12.0,  12.0, 6.0)

# "Database" of all areas for easy processing
ALL_AREAS = [
    ('A', AREA_A_CENTER, AREA_A_SIZE),
    ('B', AREA_B_CENTER, AREA_B_SIZE),
    ('C', AREA_C_CENTER, AREA_C_SIZE),
    # ('D', AREA_D_CENTER, AREA_D_SIZE),
    # ('E', AREA_E_CENTER, AREA_E_SIZE),
]
# ------------------------------------


# drone_IDs = {'gcs':0, 'jurong':1, 'raffles':2, 'sentosa':3, 'changi':4, 'nanyang':5,'expl_uav_03':6, 'expl_uav_04':7, 'expl_uav_05':8, 'expl_uav_06':9, 'expl_uav_07':10, 'expl_uav_08':11, 'expl_uav_09':12, 'expl_uav_10':13}
drone_IDs = {'gcs':0, 'jurong':1, 'raffles':2, 'expl_uav_03':3, 'expl_uav_04':4, 'expl_uav_05':5, 'expl_uav_06':6, 'expl_uav_07':7, 'expl_uav_08':8, 'expl_uav_09':9, 'expl_uav_10':10}

offsets_cross = [
    (0,-1,0), (1,0,0), (0,1,0), (-1,0,0), (0,0,1), (0,0,-1)
]

def closest_node_index_1(node, nodes):
    distances = np.linalg.norm(nodes - node, axis=1)
    return np.argmin(distances)

@nb.jit(nopython=True, cache=True)
def construct_adjacency(data, x, y, z, size_x, size_y, size_z, coordinates):
    offsets_cross = [(0,-1,0), (1,0,0), (0,1,0), (-1,0,0), (0,0,1), (0,0,-1)]
    num_of_nodes = len(coordinates)
    adjacency_1 = np.zeros((num_of_nodes,num_of_nodes))
    # log_info("Starting Adjacency calculation. Please wait... ")
    for _,coord in enumerate(coordinates):
        for _, offset in enumerate(offsets_cross):
            neighbor_x = coord[0]+(offset[0] * data)
            neighbor_y = coord[1]+(offset[1] * data)
            neighbor_z = coord[2]+(offset[2] * data)
           
            
            gone_too_far_x = (neighbor_x < x) or (neighbor_x > (x + size_x*data))
            gone_too_far_y = (neighbor_y < y) or (neighbor_y > (y + size_y*data))
            gone_too_far_z = (neighbor_z < z) or (neighbor_z > (z + size_z*data))
            if gone_too_far_x or gone_too_far_y or gone_too_far_z:
                continue
            
            
            # neighbor_index = closest_node_index_1((neighbor_x, neighbor_y, neighbor_z),coordinates)
            distances = np.empty(coordinates.shape[0], dtype=coordinates.dtype)
            for i in nb.prange(coordinates.shape[0]):
                distances[i] = np.sqrt((coordinates[i, 0]-neighbor_x)*(coordinates[i, 0]-neighbor_x) + (coordinates[i, 1]-neighbor_y)*(coordinates[i, 1]-neighbor_y) + (coordinates[i, 2]-neighbor_z)*(coordinates[i, 2]-neighbor_z))
                
            #distances = np.linalg.norm(coordinates - np.array([neighbor_x, neighbor_y, neighbor_z]))
            neighbor_index =  np.argmin(distances)

            # my_index = closest_node_index_1((coord[0], coord[1], coord[2]),coordinates)
            distances = np.empty(coordinates.shape[0], dtype=coordinates.dtype)
            for i in nb.prange(coordinates.shape[0]):
                distances[i] = np.sqrt((coordinates[i, 0]-coord[0]) *(coordinates[i, 0]-coord[0]) + (coordinates[i, 1]-coord[1]) *(coordinates[i, 1]-coord[1]) + (coordinates[i, 2]-coord[2]) *(coordinates[i, 2]-coord[2]))
            #distances = np.linalg.norm(coordinates - np.array([coord[0], coord[1], coord[2]]))
            my_index =  np.argmin(distances)
            
            # cost = euclidean_distance_3d(coord, coordinates[neighbor_index])
            try:
                adjacency_1[my_index,neighbor_index] = 1#cost
                adjacency_1[neighbor_index,my_index] = 1#cost
                #print("DAME PAEIS POU ", my_index , " DAME ", neighbor_index)
            except:
                pass

    return adjacency_1

def check_point_inside_cuboid(vertices, point):
    DT = Delaunay(vertices)

    if DT.find_simplex(point) >= 0:
        return True
    return False

def calculateCircuits(positions, num_of_nodes, TravellingCost):
    UAVs = len(positions)
    # positions = index where each uav is located

    # Initialization of Set S matrices and CircuitX, CircuitY.
    Set_S_source = [[] for i in range(0, UAVs)]
    Set_S_destination = [[] for i in range(0, UAVs)]
    Set_S_cost = [[] for i in range(0, UAVs)]
    listV1 = [0 for i in range(0, num_of_nodes)]
    for z in range(0, UAVs):
        listV1[positions[z]] = 1
    while (sum(listV1) < num_of_nodes):
        for i in range(0, UAVs):
            node = 0
            flag = False
            futureCost = sys.maxsize
            for j in range(0, num_of_nodes):
                if (listV1[j] == 0):
                    if (futureCost >= TravellingCost[positions[i]][j]):
                        futureCost = TravellingCost[positions[i]][j]
                        node = j
                        flag = True
            if flag:
                listV1[node] = 1
                Set_S_source[i].append(positions[i])
                Set_S_destination[i].append(node)
                Set_S_cost[i].append(futureCost)
                positions[i] = node

    return Set_S_destination

def set_tag(tag):
    global TAG
    TAG = tag

def log_info(info):
    global TAG, debug
    if debug:
        rospy.loginfo(TAG + f"{info}")

def neighCallback(msg):
    global neighbors
    neighbors = msg

def odomCallback(msg):
    global odom, position
    odom = msg
    position = odom.pose.pose.position
    # if namespace == 'raffles':
    #     print("---- "+namespace+": "+str(position))

def updateCallback(msg):
    global flag_pub, position, neighbors
    for _, point in enumerate(sensor_msgs.point_cloud2.read_points(neighbors, skip_nans=True)):
        point_message = Point()
        point_message.x, point_message.y, point_message.z = point[0], point[1], point[2]
        d = euclidean_distance_points(position,point_message)
        if d>=uav_distance_com:
            continue
        else:
            flag_pub.publish(msg)
            # print('okUPD')
            
def arrivedCallback(msg):
    global arrived
    arrived = msg.data

def missionTimeCallback(msg):
    global remaining_time
    remaining_time = msg.data.secs

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

def euclidean_distance(p1,point):
    p2 = np.zeros((3,1))
    p2[0] = point.x
    p2[1] = point.y
    p2[2] = point.z
    return math.sqrt( math.pow(p1[0]-p2[0],2) + math.pow(p1[1]-p2[1],2) + math.pow(p1[2]-p2[2],2))

def euclidean_distance_3d(p1,p2):
    return math.sqrt( math.pow(p1[0]-p2[0],2) + math.pow(p1[1]-p2[1],2) + math.pow(p1[2]-p2[2],2))

@nb.njit(cache=True)
def generate_waypoint_str(center_x, center_y, center_z, size_x, size_y, size_z=0.0):
    hx = size_x / 2.0
    hy = size_y / 2.0
    hz = size_z / 2.0

    min_x, max_x = center_x - hx, center_x + hx
    min_y, max_y = center_y - hy, center_y + hy
    min_z, max_z = center_z - hz, center_z + hz

    # Use -1 on min/max to stay slightly inside the box, matching original logic
    x = np.random.uniform(min_x + 1, max_x - 1)
    y = np.random.uniform(min_y + 1, max_y - 1)
    z = np.random.uniform(min_z + 1, max_z - 1)
    
    return x, y, z

# Area boundaries calculation
@nb.jit(nopython=True, cache=True)
def compute_area_bounds(center, size):
    cx, cy, cz = center
    sx, sy, sz = size
    hx, hy, hz = sx / 2.0, sy / 2.0, sz / 2.0
    return (
        cx - hx, cx + hx,  # min_x, max_x
        cy - hy, cy + hy,  # min_y, max_y
        cz - hz, cz + hz   # min_z, max_z
    )

# Function to check if a point is in a given area
@nb.jit(nopython=True, cache=True)
def is_inside_area(pos, bounds):
    x, y, z = pos
    xmin, xmax, ymin, ymax, zmin, zmax = bounds
    return xmin <= x <= xmax and ymin <= y <= ymax and zmin <= z <= zmax

# New helper function to find which area the UAV is in
def get_current_area(pos_tuple, all_areas_list):
    """
    Checks the UAV's position against all defined areas.
    Returns the name of the first area it finds, or None.
    """
    for name, center, size in all_areas_list:
        bounds = compute_area_bounds(center, size)
        if is_inside_area(pos_tuple, bounds):
            return name
    return None # Not in any defined area

# --- NEW: Global metrics variables (no class needed) ---
current_area = None
last_area = None
last_pos = None
last_time = None
area_transitions = []  # List of (from_area, to_area, time, distance)
total_distance = 0.0
total_time = 0.0

# Publishers (created after rospy.init_node)
area_transition_pub = None
total_metrics_pub = None
debug_pub = None

# Replace your update_position_metrics function with this:
def update_position_metrics(pos, current_time):
    """Update metrics based on current position and time."""
    global current_area, last_area, last_pos, last_time, area_transitions, total_distance, total_time, total_start_time
    
    current_pos = (pos.x, pos.y, pos.z)
    
    # NEW: Initialize total_start_time on first call
    if 'total_start_time' not in globals() or total_start_time is None:
        total_start_time = current_time
    
    # NEW: Publish debug position every time (to ensure topic data exists)
    current_area_new = get_current_area(current_pos, ALL_AREAS)
    debug_msg = String()
    debug_msg.data = f"{namespace} pos=({pos.x:.2f},{pos.y:.2f},{pos.z:.2f}) area={current_area_new} time={current_time:.2f}"
    debug_pub.publish(debug_msg)
    
    # Initialize on first call
    if last_pos is None:
        last_pos = current_pos
        last_time = current_time
        current_area = current_area_new
        return
    
    # Calculate distance moved (for total distance tracking)
    distance_moved = euclidean_distance_3d(last_pos, current_pos)
    total_distance += distance_moved
    
    # Check for area transition (ALWAYS check, even if one area is None)
    if current_area_new != current_area:
        # Calculate EXACT time for the transition
        transition_time = current_time - last_time
        
        # Record the transition
        area_transitions.append({
            'from_area': current_area,
            'to_area': current_area_new,
            'time': transition_time,
            'distance': distance_moved,
            'start_time': last_time,
            'end_time': current_time
        })
        
        # Log the transition
        # log_info(f"Area transition: {current_area} -> {current_area_new} | Time: {transition_time:.2f}s | Distance: {distance_moved:.2f}m")
        
        # --- Publish area transition (ALWAYS publish, even None transitions) ---
        transition_msg = String()
        transition_msg.data = f"{namespace} {current_area} -> {current_area_new} time={transition_time:.2f} distance={distance_moved:.2f}"
        area_transition_pub.publish(transition_msg)
    
    # Calculate total mission time (from start to now)
    current_total_time = current_time - total_start_time
    
    # --- Publish updated totals (REAL-TIME total) ---
    total_msg = String()
    total_msg.data = f"{namespace} total_time={current_total_time:.2f} total_distance={total_distance:.2f} transitions={len(area_transitions)}"
    total_metrics_pub.publish(total_msg)
    
    # Update current area and reset tracking
    current_area = current_area_new
    last_pos = current_pos
    last_time = current_time

def main():
    # init
    global grid_resolution, namespace, debug, odom, position, arrived, repeat, remaining_time, flag_pub, neighbors, num_explorers
    global uav_distance_com, ALL_AREAS, current_area, last_area, last_pos, last_time, area_transitions, total_distance, total_time
    global area_transition_pub, total_metrics_pub, debug_pub
    try:
        namespace = rospy.get_param('namespace') # node_name/argsname
        scenario = rospy.get_param('scenario')
        debug = rospy.get_param('debug')
        grid_resolution = rospy.get_param('grid_resolution')
        uav_distance_com = rospy.get_param('uav_distance_com')
        num_explorers = rospy.get_param('~num_explorers', 1)
        set_tag("[" + namespace.upper() + " PATH SCRIPT]: ")
    except Exception as e:
        print(e)
        namespace = "ERROR"
        scenario = 'mbs'
        debug = True
        uav_distance_com = 1
        set_tag("[" + namespace.upper() + " PATH SCRIPT]: ")
		
    rospy.init_node(namespace, anonymous=True)
    rate = rospy.Rate(1) # Run the main loop at 1 Hz
    # --- NEW: Wait a bit for ROS to fully initialize ---
    rospy.sleep(1.0)
    
    # --- NEW: Initialize publishers ---
    area_transition_pub = rospy.Publisher(f'/{namespace}/area_travel_metrics', String, queue_size=1)
    total_metrics_pub = rospy.Publisher(f'/{namespace}/total_travel_metrics', String, queue_size=1)
    debug_pub = rospy.Publisher(f'/{namespace}/debug_position', String, queue_size=1)
    
    # NEW: Small delay to ensure publishers are ready
    rospy.sleep(0.5)
    
    # subscribe to self topics
    rospy.Subscriber("/"+namespace+"/ground_truth/odometry", Odometry, odomCallback)
    rospy.Subscriber("/"+namespace+"/arrived_at_target", Bool, arrivedCallback)
    rospy.Subscriber("/"+namespace+"/mission_duration_remained", Duration, missionTimeCallback)

    rospy.Subscriber('/'+namespace+'/command/update', Bool, updateCallback)

    # target point publisher
    target_pub = rospy.Publisher("/"+namespace+"/command/targetPoint", Point, queue_size=1)
    # velocity publisher
    velo_pub = rospy.Publisher("/"+namespace+"/command/velocity", Float32, queue_size=1)
    # update flag publisher
    flag_pub = rospy.Publisher("/"+namespace+"/command/update", Bool, queue_size=1, latch=False)
    # norm pub
    norm_pub = rospy.Publisher("/"+namespace+"/norms", norms, queue_size=1)

    rospy.Subscriber("/"+namespace+"/nbr_odom_cloud", PointCloud2, neighCallback)

    # Wait for service to appear
    # log_info("Waiting for ppcom")
    # rospy.wait_for_service('/create_ppcom_topic')
    # # Create a service proxy
    # create_ppcom_topic = rospy.ServiceProxy('/create_ppcom_topic', CreatePPComTopic)
    # # Register the topic with ppcom router
    # if namespace == 'jurong':
    #     create_ppcom_topic(namespace, ['raffles'], '/'+namespace+'/command/update', 'std_msgs', 'Bool')
    # else:
    #     create_ppcom_topic(namespace, ['jurong'], '/'+namespace+'/command/update', 'std_msgs', 'Bool')

    # Get Bounding Box Verticies
    bboxes = rospy.wait_for_message("/gcs/bounding_box_vertices/", PointCloud)
    bbox_points = np.zeros((int(len(bboxes.points)/8),8,3))
    counter = 0
    for i in range(0,int(len(bboxes.points)/8)):
        for j in range(8):
            bbox_points[i,j,0] = bboxes.points[counter].x
            bbox_points[i,j,1] = bboxes.points[counter].y
            bbox_points[i,j,2] = bboxes.points[counter].z
            counter += 1

    # Get inspection area details
    # log_info("Waiting for area details")
    # area_details = rospy.wait_for_message("/world_coords/", area)
    # log_info("Construct Adjacency")    
    # xrange = range(int(area_details.minPoint.x + area_details.resolution.data/2), int(area_details.minPoint.x + area_details.size.x * area_details.resolution.data - area_details.resolution.data/2) + int(area_details.resolution.data), int(area_details.resolution.data)) 
    # yrange = range(int(area_details.minPoint.y + area_details.resolution.data/2), int(area_details.minPoint.y + area_details.size.y * area_details.resolution.data - area_details.resolution.data/2) + int(area_details.resolution.data), int(area_details.resolution.data)) 
    # zrange = range(int(area_details.minPoint.z + area_details.resolution.data/2), int(area_details.minPoint.z + area_details.size.z * area_details.resolution.data - area_details.resolution.data/2) + int(area_details.resolution.data), int(area_details.resolution.data)) 
    # Constructing the graph
    # coordinates = np.asarray([(x,y,z) for x in xrange for y in yrange for z in zrange]).astype(float)
    # adjacency_org = constuct_adjacency(area_details, coordinates)
    # adjacency_org = construct_adjacency(area_details.resolution.data, area_details.minPoint.x, area_details.minPoint.y, area_details.minPoint.z, area_details.size.x, area_details.size.y, area_details.size.z, coordinates)

    # Wait for odometry to be ready
    
    rospy.wait_for_message("/"+namespace+"/ground_truth/odometry", Odometry)
    
    try:
        neighbors = rospy.wait_for_message("/"+namespace+"/nbr_odom_cloud", PointCloud2, timeout=5.0)
    except rospy.ROSException:
        rospy.logwarn("Timeout waiting for /%s/nbr_odom_cloud. Proceeding without neighbors.", namespace)
        
    neighbors = PointCloud2()  # empty cloud
    init_pos = position
    gcs_pos = Point()
    for _, point in enumerate(sensor_msgs.point_cloud2.read_points(neighbors, skip_nans=True)):
        if point[3] == 0:
            gcs_pos.x = point[0]
            gcs_pos.y = point[1]
            gcs_pos.z = point[2]
            break
    
    log_info("Waiting for adjacency build")
    rospy.wait_for_message("/"+namespace+"/adjacency_viz", MarkerArray)
    rate.sleep()

    # This logic is now handled by the main `while repeat` loop
    # go to initial points for map building
    # ... (old logic removed) ...

    bool_msg = Bool()
    bool_msg.data = True
    flag_pub.publish(bool_msg)
    rate.sleep()
    update_done = False
    if num_explorers > 1:
        log_info("Waiting for map merge to complete")
        # wait for neighbor update flag
        if namespace == 'jurong':
            if scenario != 'hangar':
                while not update_done:
                    try:
                        flag_pub.publish(bool_msg)
                        msg = rospy.wait_for_message("/"+namespace+"/command/update_done", Bool,1)
                        update_done = msg.data
                    except:
                        pass
                    rate.sleep()
        else:
            while not update_done:
                try:
                    flag_pub.publish(bool_msg)
                    print(namespace+" | its so gay staff")
                    msg = rospy.wait_for_message("/"+namespace+"/command/update_done", Bool,1)
                    update_done = msg.data
                except:
                    pass
                rate.sleep()
    else:
        flag_pub.publish(bool_msg)

    vel_msg = Float32()
    vel_msg.data = 3.5
    velo_pub.publish(vel_msg)
    
    # count = 0
    
    # Flag to track if we've sent the *first* target yet.
        # After all initialization...
    sent_first_target = False
    log_info("Starting dynamic area navigation loop...")

    while repeat and not rospy.is_shutdown():
        # --- Update area travel metrics ---
        current_time = rospy.Time.now().to_sec()
        update_position_metrics(position, current_time)
        
        if (not sent_first_target) or arrived:
            # Reset flags BEFORE publishing new target
            arrived = False
            sent_first_target = True

            # Determine current area
            current_pos = (position.x, position.y, position.z)
            current_area_current = get_current_area(current_pos, ALL_AREAS)
            log_info(f"Currently in area: {current_area_current}")

            # Choose next area (random, not current)
            all_names = [a[0] for a in ALL_AREAS]
            next_options = [n for n in all_names if n != current_area_current]
            if not next_options:
                next_options = all_names  # fallback
            next_area_name = rand.choice(next_options)
            log_info(f"Selected next area: {next_area_name}")

            # Get area definition
            next_area = next((a for a in ALL_AREAS if a[0] == next_area_name), None)
            if not next_area:
                log_info(f"ERROR: Area {next_area_name} not found!")
                rate.sleep()
                continue

            _, center, size = next_area
            wp_x, wp_y, wp_z = generate_waypoint_str(*center, *size)

            # Publish target
            target_msg = Point(x=wp_x, y=wp_y, z=5)
            log_info(f"Setting new target in Area {next_area_name}: ({wp_x:.2f}, {wp_y:.2f}, {5:.2f})")
            target_pub.publish(target_msg)

            # WAIT until trajectory node confirms arrival
            while not arrived and not rospy.is_shutdown():
                # Optional: re-publish target periodically (helps if message dropped)
                target_pub.publish(target_msg)
                # --- Update metrics during travel ---
                current_time = rospy.Time.now().to_sec()
                update_position_metrics(position, current_time)
                rate.sleep()

        else:
            # --- Update metrics during idle time ---
            current_time = rospy.Time.now().to_sec()
            update_position_metrics(position, current_time)
            rate.sleep()
        
        # --- End of new logic ---
        
        # The old TSP logic is preserved below, commented out.
        # If you want to run TSP *after* this area logic, you can
        # set `repeat = False` when some condition is met.

    #     # Generate and go to TSP points
    #     log_info("Waiting for map")
    #     occupied_indicies = rospy.wait_for_message("/"+namespace+"/adjacency/", Int16MultiArray)
    #     occupied_indicies = np.asarray(occupied_indicies.data)
    #     adjacency = np.copy(adjacency_org)
    #     adjacency[:,occupied_indicies] = 0

    #     log_info("Calculating Object Waypoints")
    #     targeted_points = np.empty((0,1))
    #     inspect_points = np.empty((0,1))
    #     for index in occupied_indicies:
    #             for box_i in range(0,int(len(bboxes.points)/8)):
    #                 if check_point_inside_cuboid(bbox_points[box_i], coordinates[index]):
    #                     targeted_points = np.append(targeted_points, index)
    #                     break
        
    #     log_info("Calculating Object Neighbors")
    #     targeted_points = targeted_points.astype(int)
        
    #     all_norms = np.empty((0,3))
    #     for target_point in targeted_points:
    #         points = np.where(adjacency[target_point]>0)[0]
    #         inspect_points = np.append(inspect_points, points)
    #         for point in points:            
    #             norm = coordinates[point] - coordinates[target_point]
    #             norm /= np.linalg.norm(norm)
    #             all_norms = np.append(all_norms, [norm], axis=0)

    #     log_info("Running unique")
    #     inspect_points, ind = np.unique(inspect_points,axis=0, return_index=True)
    #     all_norms = all_norms[ind]
    #     inspect_points = inspect_points.astype(int)


    #     log_info("Constructing norms message")
    #     norm_msg = norms()
    #     for i, point in enumerate(inspect_points):
    #         facet_mid = Point()
    #         facet_mid.x = coordinates[point,0]
    #         facet_mid.y = coordinates[point,1]
    #         facet_mid.z = coordinates[point,2]
    #         norm_msg.facet_mids.append(facet_mid)
    #         norm_point = Point()
    #         norm_point.x = all_norms[i,0]
    #         norm_point.y = all_norms[i,1]
    #         norm_point.z = all_norms[i,2]
    #         norm_msg.normals.append(norm_point)

    #     norm_pub.publish(norm_msg)

    #     log_info("Constructing TSP matrix")
    #     neighbors = PointCloud2()
    #     try:
    #         neighbors = rospy.wait_for_message("/"+namespace+"/nbr_odom_cloud", PointCloud2,2)
    #     except:
    #         pass
    #     uav_positions = np.empty((0,3))
    #     uav_indices = np.array([])
    #     for _, point in enumerate(sensor_msgs.point_cloud2.read_points(neighbors, skip_nans=True)):
    #         if point[3] != drone_IDs['gcs']:
    #             # log_info(point[3])
    #             uav_positions = np.append(uav_positions, [[point[0], point[1], point[2]]], axis=0)
    #             uav_indices = np.append(uav_indices, point[3])
    #             #test distance between UAVs
    #     # print([position.x, position.y, position.z])
    #     # for _, point in enumerate(sensor_msgs.point_cloud2.read_points(neighbors, skip_nans=True)):
    #     #     point_message = Point()
    #     #     point_message.x, point_message.y, point_message.z = point[0], point[1], point[2]
    #     #     d = euclidean_distance_points(position,point_message)
    #     #     if d<=90 and d>=50:
    #     #         continue
    #     #     else:
    #     #         print('Communication available! Distance:'+str(d))
            
    #     pos = 0
    #     while (pos < len(uav_indices)) and (uav_indices[pos] < drone_IDs[namespace]):
    #         pos += 1
        
    #     uav_positions = np.insert(uav_positions, pos, [position.x, position.y, position.z], axis=0)
    #     uav_indices = np.insert(uav_indices, pos, drone_IDs[namespace])

    #     num_of_agents = uav_positions.shape[0]

    #     points = np.concatenate((uav_positions, coordinates[inspect_points]))
    #     num_of_nodes = points.shape[0]

    #     adjacency = np.zeros((num_of_nodes,num_of_nodes))
    #     for i in range(num_of_nodes):
    #         for j in range(num_of_nodes):
    #             adjacency[i,j] = euclidean_distance_3d(points[i],points[j])

    #     log_info("Running mTSP")
    #     waypointsMatrix = calculateCircuits([i for i in range(num_of_agents)], num_of_nodes, adjacency)
    #     log_info("TSP Path length: " + str(len(waypointsMatrix[pos])))
    #     for waypoint in waypointsMatrix[pos]:
    #         point = Point()
    #         point.x = points[waypoint,0]
    #         point.y = points[waypoint,1]
    #         point.z = points[waypoint,2]
    #         # log_info("Setting target to point: " + str(points[waypoint]))
    #         while not arrived:
    #             if remaining_time < 10.0:
    #                 d__init = euclidean_distance_points(position, init_pos)
    #                 d_gcs = euclidean_distance_points(position, gcs_pos)
    #                 # los = False #COMMENT LOS LOGIC FOR REMOVING PPCOM
    #                 if d_init <= d_gcs:
    #                     log_info("Setting target to initial point: " + str(init_pos))
    #                     # while not los:
    #                     target_pub.publish(init_pos)
    #                     # for _, point in enumerate(sensor_ms.point_cloud2.read_points(neighbors, skip_nans=True)):
    #                     #     if point[3] == 0:
    #                             # los = True
    #                     # rate.sleep()
    #                 elif d_gcs < d_init:
    #                     log_info("Setting target to gcs point: " + str(gcs_pos))
    #                     # while not los:
    #                     target_pub.publish(gcs_pos)
    #                     # for _, point in enumerate(sensor_msgs.point_cloud2.read_points(neighbors, skip_nans=True)):
    #                     #     if point[3] == 0:
    #                             # los = True
    #                         # rate.sleep()
    #             else:
    #                 target_pub.publish(point)
    #                 rate.sleep()
    #         arrived = False
            

    #     if count < 2.0:
    #         vel_msg = Float32()
    #         vel_msg.data = 4 - count
    #         velo_pub.publish(vel_msg)
    #     count += 1
    
        # Sleep to maintain the loop rate (1 Hz)

    # Return to Home (ensure LOS with GCS)
    log_info("Setting target to initial point: " + str(init_pos))
    while not arrived:
        target_pub.publish(init_pos)
        # --- Update metrics during final travel ---
        current_time = rospy.Time.now().to_sec()
        update_position_metrics(position, current_time)
        rate.sleep()
    arrived = False

    # --- Publish final metrics ---
    final_msg = String()
    final_msg.data = f"{namespace} final_total_time={total_time:.2f} final_total_distance={total_distance:.2f} final_transitions={len(area_transitions)}"
    total_metrics_pub.publish(final_msg)

    # --- Print final metrics before exit ---
    log_info("=== FINAL TRAVEL METRICS ===")
    log_info(f"UAV: {namespace}")
    log_info(f"Total travel time: {total_time:.2f}s")
    log_info(f"Total distance: {total_distance:.2f}m")
    log_info(f"Number of area transitions: {len(area_transitions)}")
    
    for i, transition in enumerate(area_transitions):
        log_info(f"  Transition {i+1}: {transition['from_area']} -> {transition['to_area']} | "
                 f"Time: {transition['time']:.2f}s | Distance: {transition['distance']:.2f}m")

    while not rospy.is_shutdown():
        log_info("Finished")
        flag_pub.publish(bool_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        # --- Print metrics on interruption ---
        final_msg = String()
        final_msg.data = f"{namespace} final_total_time={total_time:.2f} final_total_distance={total_distance:.2f} final_transitions={len(area_transitions)}"
        total_metrics_pub.publish(final_msg)
        
        log_info("=== FINAL TRAVEL METRICS (Interrupted) ===")
        log_info(f"UAV: {namespace}")
        log_info(f"Total travel time: {total_time:.2f}s")
        log_info(f"Total distance: {total_distance:.2f}m")
        log_info(f"Number of area transitions: {len(area_transitions)}")
        print("terminating...")
    except Exception as e:
        traceback.print_exc()
    finally:
        exit()