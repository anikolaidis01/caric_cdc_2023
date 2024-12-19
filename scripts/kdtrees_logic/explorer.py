################## Explorer Trajectory Code ##################
__author__ = "Andreas Anastasiou, Angelos Zacharia"
__copyright__ = "Copyright (C) 2023 KIOS Center of Excellence"
__version__ = "7.0"
##############################################################

import rospy
from std_msgs.msg import Header, Float32, Bool, Int16MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Vector3, Quaternion, Twist, Point
from caric_mission.srv import CreatePPComTopic
from kios_solution.msg import area
from octomap_msgs.srv import BoundingBoxQuery
from visualization_msgs.msg import MarkerArray, Marker
import math
import numpy as np
import heapq
import threading
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import shortest_path
import traceback
from scipy.spatial import KDTree
import time

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
#neighbors' offsets starting from same z plane counter-clockwise
offsets_all = [
    (-1,-1,0), (0,-1,0), (1,-1,0), (1,0,0), (1,1,0), (0,1,0), (-1,1,0), (-1,0,0), 
    (-1,-1,1), (0,-1,1), (1,-1,1), (1,0,1), (1,1,1), (0,1,1), (-1,1,1), (-1,0,1),(0,0,1),
    (-1,-1,-1), (0,-1,-1), (1,-1,-1), (1,0,-1), (1,1,-1), (0,1,-1), (-1,1,-1), (-1,0,-1),(0,0,-1)
]
offsets_cross = [
    (0,-1,0), (1,0,0), (0,1,0), (-1,0,0), (0,0,1), (0,0,-1)
]
def print_score_interest_points():
 while not rospy.is_shutdown():

  points = rospy.wait_for_message('/'+namespace+'/detected_interest_points', PointCloud2)
  points = sensor_msgs.point_cloud2.read_points(points, skip_nans=True)

  for point in points:
     with open("score_"+namespace+".txt", "a") as file2:
      file2.write(str(point[0])+" "+str(point[1])+" "+str(point[2])+" "+str(point[3])+str(point[4])+" "+str(point[5])+" "+str(point[6])+" "+str(point[7])+"\n")
     
   


def dijkstra(g, arrival_pub, s, t):  #t = target is the node of the target point ,g=adj neigh
    global coordinates
    rate = rospy.Rate(2)
    if (s==t):
        # init arrival message
        log_info("Arrived at " + str(coordinates[t])) ######################################################CHANGE
        arrived_msg = Bool()
        arrived_msg.data = True
        arrival_pub.publish(arrived_msg)
        rate.sleep()
        return [s,s]
    
    if (len(np.nonzero(g[s,:])[0]) == 0): ## ean i grammi tou source den ehei pou na paei 
        log_info("Source " + str(t) + " blocked")
        arrived_msg = Bool()
        arrived_msg.data = True
        arrival_pub.publish(arrived_msg)
        rate.sleep()
        return [s,s]
    
    if (len(np.nonzero(g[:,t])[0]) == 0): ## ean sto  target den mporei na paei kanenas
        log_info("Target " + str(t) + " not reachable")
        arrived_msg = Bool()
        arrived_msg.data = True
        arrival_pub.publish(arrived_msg) #allaksa
        rate.sleep()
        return [s,s]
    
    # Initialize distance and priority queue
    d = {n: float('inf') for n in range(len(g))}
    d[s] = 0
    p = {}
    q = [(0, s)]
    
    last_w, curr_v = heapq.heappop(q)

    while  curr_v != t:

        neighbor_indices = np.nonzero(g[curr_v,:])[0] # pkoianei ta indices gia to pou mporei na paei apo to current 
        for n in neighbor_indices:
            n_w = g[curr_v,n]
            
            cand_w = last_w + n_w # equivalent to d[curr_v] + n_w 
            if cand_w < d[n]:
                d[n] = cand_w
                p[n] = curr_v
                heapq.heappush(q, (cand_w, n))
        last_w, curr_v = heapq.heappop(q)  
        
    return generate_path(p, s, t)


def generate_path(parents, start, end):
    path = [end]
    while path[0] != start:
        path.insert(0, parents[path[0]])
    return path

def set_tag(tag):
    global TAG
    TAG = tag

def log_info(info):
    global TAG, debug
    if debug:
        rospy.loginfo(TAG + f"{info}")
        publish_text_viz(TAG + f"{info}")

def odomCallback(msg):
    global odom
    odom = msg

def targetCallback(msg):
    global target, coordinates # target is likely intended to store the index of the closest coordinate to the target point.
                                # coordinates is probably a list or array of 3D points (x, y, z) representing locations in space.
    target_point = msg
    try:
        target =  closest_node_index((target_point.x,target_point.y,target_point.z),coordinates) #Finds the closest node only among connected nodes, based on an adjacency matrix (adjacency_neigh). If a node has no connections, it is ignored in the distance calculation.

    except:
        pass
   
def yawCallback(msg):
    global target_yaw
    target_yaw = msg.data

def veloCallback(msg):
    global maxVel
    maxVel = msg.data

def neighCallback(msg):
    global neighbors
    neighbors = msg
    
def euclidean_distance_3d(p1,p2):
    return math.sqrt( math.pow(p1[0]-p2[0],2) + math.pow(p1[1]-p2[1],2) + math.pow(p1[2]-p2[2],2))

def construct_adjacency(area_details, coordinates):
    #start_time=time.time()
    global offsets_cross
    num_of_nodes = len(coordinates) 
    adjacency_1 = np.zeros((num_of_nodes, num_of_nodes)) 
    
    tree = KDTree(coordinates)
    log_info("Starting Adjacency calculation. Please wait... ")
    
    for  coord in coordinates:
        _, my_index = tree.query(coord, k=1)
        
        for  offset in offsets_cross:
            neighbor_x = coord[0] + (offset[0] * area_details.resolution.data) 
            neighbor_y = coord[1] + (offset[1] * area_details.resolution.data)
            neighbor_z = coord[2] + (offset[2] * area_details.resolution.data)
            
            gone_too_far_x = (neighbor_x < area_details.minPoint.x) or (neighbor_x > (area_details.minPoint.x + area_details.size.x * area_details.resolution.data))
            gone_too_far_y = (neighbor_y < area_details.minPoint.y) or (neighbor_y > (area_details.minPoint.y + area_details.size.y * area_details.resolution.data))
            gone_too_far_z = (neighbor_z < area_details.minPoint.z) or (neighbor_z > (area_details.minPoint.z + area_details.size.z * area_details.resolution.data))
            
            if gone_too_far_x or gone_too_far_y or gone_too_far_z:
                continue
            
            _, neighbor_index = tree.query((neighbor_x, neighbor_y, neighbor_z), k=1)
            
            
            try:
                adjacency_1[my_index, neighbor_index] = 1  # or some cost if calculated
                adjacency_1[neighbor_index, my_index] = 1  # or some cost if calculated
            except IndexError:
                pass
    # elapsed_time = time.time() - start_time
    # with open("222.txt", "a") as file:
    #    file.write(f"Time taken to construct adjacency: {elapsed_time:.4f} seconds\n")
   
    return adjacency_1


    #no change


def update_adjacency(adjacency, coordinates, obstacle_coordinates):##kamneis occupied opou ehei obstracle
    #start_time=time.time()
    global grid_resolution
    adjacency_temp = np.copy(adjacency)
    tree = KDTree(coordinates)
    #inds = np.array([], dtype=int)
    # Add octomap voxel centers as obstacles in graph
    # for obstacle in obstacle_coordinates:
    #     _, index = tree.query((obstacle[0], obstacle[1], obstacle[2]), k=1)
    #     inds = np.append(inds, [index], axis=0)
    obstacle_coords_array = list(obstacle_coordinates)
    _, indices = tree.query(obstacle_coords_array, k=1)
    #print the indeces
    # with open("index5.txt", "a") as file:  
    #     file.write(str(namespace)+" length indeces: "+str(len(inds))+"length of obstr: my obst indx are " +str(inds) +"\n")
    indices = indices.astype(int)
    adjacency_temp[:,indices] = 0
    # end_time = time.time()
    # elapsed_time = end_time - start_time
    # with open("execution_time22.txt", "a") as file:
    #    file.write(f"Time taken to construct adjacency: {elapsed_time:.4f} seconds\n")
    
    adjacency_neigh = update_adjacency_with_neighbors(adjacency_temp)
    return adjacency_temp, adjacency_neigh


# def update_adjacency_with_neighbors(adjacency):   
#     # start_time=time.time()
#     global neighbors, grid_resolution, coordinates, area_details # ta neighbours edw  einai oi coords twn uav twn allwn extos sftou me to nm 
#     # add LOS neighbors as obstacles in graph
#     tree = KDTree(coordinates)

#     adjacency_temp = np.copy(adjacency)

#     for point in sensor_msgs.point_cloud2.read_points(neighbors, skip_nans=True):
#         # with open("seeNeigh.txt", "a") as file:
#         #  file.write(namespace+str(point[3])+" "+str(point[2])+"\n")
#        # if point[2] >= 1 : # den einai gcs kai peta 
#             #index = closest_node_index_1((point[0], point[1], point[2]), coordinates) ## finds what node from coordinates is the  neighbour uav
#             _,index=tree.query((point[0], point[1], point[2]),k=1)       # finds what node from coordinates is the  neighbour uav
#             adjacency_temp[:,index]=0 ## en mporei na paei se coords pou vriskontai alla uav 
#             if point[3] != 0 and point[2] >= 1 :
#              for offset in offsets_cross:
#                 neighbor_x = coordinates[index][0]+(offset[0] * grid_resolution)
#                 neighbor_y = coordinates[index][1]+(offset[1] * grid_resolution)
#                 neighbor_z = coordinates[index][2]+(offset[2] * grid_resolution)
#                 if (area_details.minPoint.x <= neighbor_x <= area_details.minPoint.x + area_details.size.x * area_details.resolution.data and
#                     area_details.minPoint.y <= neighbor_y <= area_details.minPoint.y + area_details.size.y * area_details.resolution.data and
#                     area_details.minPoint.z <= neighbor_z <= area_details.minPoint.z + area_details.size.z * area_details.resolution.data):
                   
            
#                  _,neighbor_index=tree.query((neighbor_x, neighbor_y, neighbor_z),k=1)
#                  adjacency_temp[:,neighbor_index]=0
        
            
             

#     # mark isolated nodes as obstacles in graph
#     # arr = np.sum(adjacency_temp, axis=1) ## the sum of each row 
#     # isolated_indicies = np.where(arr < 2)[0] ## giati fefkoume jines pou en mikroteres pou 2 (des tablet)
#     isolated_indices = np.where(np.sum(adjacency_temp, axis=1) < 2)[0] #prostheteis tin kathe grammi kai opkoia ehei 0 i 1 times 1 pernoume to index tis
#     adjacency_temp[:, isolated_indices] = 0

#     # for _, index in enumerate(isolated_indicies):
#     #     adjacency_temp[:,index] = 0
#     # elapsed_time = time.time() - start_time
#     # with open("222.txt", "a") as file:
#     #    file.write(f"Time taken to construct adjacency: {elapsed_time:.4f} seconds\n")
   
#     return adjacency_temp

def update_adjacency_with_neighbors(adjacency):   
    # start_time=time.time()
    global neighbors, grid_resolution, coordinates, area_details # ta neighbours edw  einai oi coords twn uav twn allwn extos sftou me to nm 
    # add LOS neighbors as obstacles in graph
    tree = KDTree(coordinates)

    adjacency_temp = np.copy(adjacency)

    for point in sensor_msgs.point_cloud2.read_points(neighbors, skip_nans=True):
        # with open("seeNeigh.txt", "a") as file:
        #  file.write(namespace+str(point[3])+" "+str(point[2])+"\n")
       if point[2] >= 1 and point[3] != 0: # den einai gcs kai peta 
            #index = closest_node_index_1((point[0], point[1], point[2]), coordinates) ## finds what node from coordinates is the  neighbour uav
            _,index=tree.query((point[0], point[1], point[2]),k=1)       # finds what node from coordinates is the  neighbour uav
            adjacency_temp[:,index]=0 ## en mporei na paei se coords pou vriskontai alla uav 
            for offset in offsets_cross:
                neighbor_x = coordinates[index][0]+(offset[0] * grid_resolution)
                neighbor_y = coordinates[index][1]+(offset[1] * grid_resolution)
                neighbor_z = coordinates[index][2]+(offset[2] * grid_resolution)
                if (area_details.minPoint.x <= neighbor_x <= area_details.minPoint.x + area_details.size.x * area_details.resolution.data and
                    area_details.minPoint.y <= neighbor_y <= area_details.minPoint.y + area_details.size.y * area_details.resolution.data and
                    area_details.minPoint.z <= neighbor_z <= area_details.minPoint.z + area_details.size.z * area_details.resolution.data):
                   
            
                 _,neighbor_index=tree.query((neighbor_x, neighbor_y, neighbor_z),k=1)
                 adjacency_temp[:,neighbor_index]=0
        
            
             

    # mark isolated nodes as obstacles in graph
    # arr = np.sum(adjacency_temp, axis=1) ## the sum of each row 
    # isolated_indicies = np.where(arr < 2)[0] ## giati fefkoume jines pou en mikroteres pou 2 (des tablet)
    isolated_indices = np.where(np.sum(adjacency_temp, axis=1) < 2)[0] #prostheteis tin kathe grammi kai opkoia ehei 0 i 1 times 1 pernoume to index tis
    adjacency_temp[:, isolated_indices] = 0

    # for _, index in enumerate(isolated_indicies):
    #     adjacency_temp[:,index] = 0
    # elapsed_time = time.time() - start_time
    # with open("222.txt", "a") as file:
    #    file.write(f"Time taken to construct adjacency: {elapsed_time:.4f} seconds\n")
   
    return adjacency_temp








def update_from_neighbor(coordinates): 
    global adjacency, update, namespace, mutex, adjacency_final, scenario #adjacency = with obstracles
    log_info("waiting for update command")
    flag_pub = rospy.Publisher("/"+namespace+"/command/update_done", Bool, queue_size=1, latch=True)
    flag_pub2 = rospy.Publisher("/"+namespace+"/command/update", Bool, queue_size=1, latch=True)
    adj_pub = rospy.Publisher("/"+namespace+"/adjacency", Int16MultiArray, queue_size=1, latch=True)
    occ_pub = rospy.Publisher("/"+namespace+"/occupancy_coords", Int16MultiArray, queue_size=1, latch=True)## dame mpainoun oulla ta occupied pou eivre o explorer kata tin diarkiea twn targets twn aksonon
    bool_msg = Bool() 
    bool_msg.data = True

    rospy.wait_for_message("/"+namespace+"/command/update", Bool) #otan o jurong perasei apo ta target point (true apo explorer_path line 385)
    rate = rospy.Rate(1)
    if scenario != 'hangar':
        occupancy_coords = Int16MultiArray()
        log_info("Waiting for neighbor map")
        while len(occupancy_coords.data) == 0:
            try:
                if namespace == 'jurong' :
                    rospy.wait_for_message("/raffles/command/update/"+namespace, Bool,1) #otan o raffles perasei apo ta target point (true apo explorer_path line 394)
                    occupancy_coords = rospy.wait_for_message('/raffles/occupancy_coords/'+namespace, Int16MultiArray, 1)# pkoianei ta occumpany cords tou ruffles , ta opoia dimosieuse sto line 658
                else:
                    rospy.wait_for_message("/jurong/command/update/"+namespace, Bool,1) # pes oti o ruffles einai o teleutaios pou the ftasei sta target point , ara tha erthei edw kai tha kartera ton jurong na kanei ommand/update, opou tha ginei me to jurong path sto line 343
                    occupancy_coords = rospy.wait_for_message('/jurong/occupancy_coords/'+namespace, Int16MultiArray, 1)
            except rospy.exceptions.ROSException as e:
                log_info("Waiting for neighbor map")
            rate.sleep()
        #start_time=time.time()     
        flag_pub.publish(bool_msg) ##me auto edw vgaei to apo to while sto path
       #flag_pub2.publish(bool_msg)#se periptwsi pou ojurong den parei to msg apo ton path
        log_info("Acquiring mutex")
        mutex.acquire()# kai na min to kleidwna skeutou tha sinehize sto main kai tha epkoiane occoupied pou einai girw tou stin teleutaia thesi sto target 
        update = False #telleiwsame pou ta target twn aksonon , meto pou erthei edw kai meinei sto while o allos explorer panw den tha mporei na parei occ apo main thread
        #flag_pub2.publish(bool_msg)
        log_info("Merging map")
        #flag_pub2.publish(bool_msg)
        # occupancy_coords = enumerate(sensor_msgs.point_cloud2.read_points(occupancy_coords, skip_nans=True, field_names=['x','y','z']))
        # adjacency_final, _ = update_adjacency(adjacency, coordinates, occupancy_coords)
        occupied_indicies = np.asarray(occupancy_coords.data)
        adjacency[:,occupied_indicies] = 0
        #flag_pub2.publish(bool_msg)
        clear_agent_box(6, namespace)
        occupancy_coords = rospy.wait_for_message('/'+namespace+'/octomap_point_cloud_centers', PointCloud2) 
        #flag_pub2.publish(bool_msg)
        occupancy_coords = sensor_msgs.point_cloud2.read_points(occupancy_coords, skip_nans=True, field_names=['x','y','z'])
        adjacency_final, _ = update_adjacency(adjacency, coordinates, occupancy_coords)
        #flag_pub2.publish(bool_msg)
        mutex.release() # dikaioute na mpein to main thead sto aquire kai na kanei update map
        log_info("Merging DONE")
    else:
        #flag_pub2.publish(bool_msg)
        log_info("Acquiring mutex")
        mutex.acquire()
        update = False
        log_info("Final map update")
        clear_agent_box(6, namespace)
        occupancy_coords = rospy.wait_for_message('/'+namespace+'/octomap_point_cloud_centers', PointCloud2)
        occupancy_coords = sensor_msgs.point_cloud2.read_points(occupancy_coords, skip_nans=True, field_names=['x','y','z'])
        adjacency_final, _ = update_adjacency(adjacency, coordinates, occupancy_coords)
        mutex.release()
        log_info("Final map update DONE")

    # filename = "./"+namespace+"_adjacency.csv"
    # np.savetxt(filename, adjacency_final, delimiter=",")

    arr = np.sum(adjacency_final, axis=0)
    occupied_msg = Int16MultiArray()
    occupied_msg.data = np.where(arr == 0)[0].astype(int)
    # end_time = time.time()
    # elapsed_time = end_time - start_time
    # with open("now3.txt", "a") as file:
    #    file.write(f"Time taken to construct adjacency: {elapsed_time:.4f} seconds\n")
   
    while not rospy.is_shutdown():
        #start_time=time.time()
        clear_agent_box(6, namespace)
        flag_pub.publish(bool_msg)
        #flag_pub2.publish(bool_msg)
        adj_pub.publish(occupied_msg)# ksipna ex_path.py kai  photogrfer(meso gcs to photografer an den ehei los me explorer)##dameee
        occ_pub.publish(occupied_msg)#en gia otan o deuteros explorer en efkike apo to while panw kai thelei occ pub gia na fkei , pou tin stigmi pou to updta egine false den mporei na to pkoiae apo to main thread
        arr = np.sum(adjacency_final, axis=0) #apo panw einai to merged teliko map adj
        occupied_msg = Int16MultiArray()
        occupied_msg.data = np.where(arr == 0)[0].astype(int)
        publish_graph_viz(coordinates, adjacency_neigh)
        rate.sleep
        # end_time = time.time()
        # elapsed_time = end_time - start_time
        # with open("22.txt", "a") as file:
        #  file.write(f"Time taken to construct adjacency: {elapsed_time:.4f} seconds\n")
   
  

def closest_node_index_1(node, nodes):#pkianei kathe coord twn coordinates kai vriskei pkoio apo auta ta coord einai pkio konta sto node
    distances = np.linalg.norm(nodes - node, axis=1)
    return np.argmin(distances)

# def closest_node_index(node, nodes): 
#     global adjacency_neigh 
#     arr = np.sum(adjacency_neigh, axis=0)   
#     valid_dist_indices = np.nonzero(arr)[0] 
#     tree = KDTree(nodes[valid_dist_indices])     
#     _, ind = tree.query(node, k=1)  # `ind` is relative to `valid_dist_indices`

    
   
#     return valid_dist_indices[ind]


def closest_node_index(node, nodes): #vriskei pkoia indexes apo coords den einai occupied kai apo auta vriskei to kontinotero pou einai me to node 
    global adjacency_neigh          
    arr = np.sum(adjacency_neigh, axis=0)   
    valid_dist_indices = np.nonzero(arr)[0] 
    distances = np.linalg.norm(nodes[valid_dist_indices] - node, axis=1)
   
    # nodes = np.asarray(nodes)
    # deltas = nodes[valid_dist_indices] - node
    # dist_2 = np.einsum('ij,ij->i', deltas, deltas)
    
    return valid_dist_indices[np.argmin(distances)]#valid_dist_indices[np.argmin(dist_2)] Finds the index of the smallest value in the distances array.This corresponds to the closest valid node to the target point.Maps this index back to the original indices in the nodes array.

def clear_agent_box(size, namespace):
    # start_time=time.time()
    global odom, neighbors
    clear_bbox = rospy.ServiceProxy('/'+namespace+'/octomap_server_'+namespace+'/clear_bbx', BoundingBoxQuery)
    min = Point()
    max = Point()
    min.x = odom.pose.pose.position.x - size/4
    min.y = odom.pose.pose.position.y - size/4
    min.z = odom.pose.pose.position.z - size/4
    max.x = odom.pose.pose.position.x + size/4
    max.y = odom.pose.pose.position.y + size/4
    max.z = odom.pose.pose.position.z + size/4
    clear_bbox(max=max, min=min)

    for point in sensor_msgs.point_cloud2.read_points(neighbors, skip_nans=True):
        min.x = point[0] - size/4
        min.y = point[1] - size/4
        min.z = point[2] - size/4
        max.x = point[0] + size/4
        max.y = point[1] + size/4
        max.z = point[2] + size/4
        clear_bbox(max=max, min=min)
    # end_time = time.time()
    # elapsed_time = end_time - start_time
    # with open("execution_time.txt", "a") as file:
    #  file.write(f"Time taken to construct adjacency: {elapsed_time:.4f} seconds\n")
        

def go_to_point():
    global cmd_pub, odom, waypoint, target_yaw, maxVel
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        #print(waypoint)
        if waypoint[0] != -3000:
            header_msg = Header()
            header_msg.frame_id = 'world'
            trajset_msg = MultiDOFJointTrajectory()
            trajpt_msg = MultiDOFJointTrajectoryPoint()
            transform_msgs = Transform()
            translation_msg = Vector3()
            rotation_msg = Quaternion()
            zero_vector_msg = Vector3()
            velocities_msg = Twist()
            acceleration_msg = Twist()
    
            if maxVel == 0.0:
                translation_msg.x = waypoint[0]
                translation_msg.y = waypoint[1]
                translation_msg.z = waypoint[2]
                velocities_msg.linear.x = 0.0#max(min((waypoint[0]-odom.pose.pose.position.x) * 1.0,maxVel), -maxVel)
                velocities_msg.linear.y = 0.0#max(min((waypoint[1]-odom.pose.pose.position.y) * 1.0,maxVel), -maxVel)
                velocities_msg.linear.z = 0.0#max(min((waypoint[2]-odom.pose.pose.position.z) * 1.0,2.0), -2.0)
            else:
                translation_msg.x = 0.0
                translation_msg.y = 0.0
                translation_msg.z = 0.0
                velocities_msg.linear.x = max(min((waypoint[0]-odom.pose.pose.position.x) * 1.0,maxVel), -maxVel)
                velocities_msg.linear.y = max(min((waypoint[1]-odom.pose.pose.position.y) * 1.0,maxVel), -maxVel)
                velocities_msg.linear.z = max(min((waypoint[2]-odom.pose.pose.position.z) * 1.0,maxVel), -maxVel)

            rotation_msg.x = 0.0
            rotation_msg.y = 0.0
            rotation_msg.z = np.sin(target_yaw/2.0)
            rotation_msg.w = np.cos(target_yaw/2.0)

            #velocities_msg.linear = zero_vector_msg
            #q = odom.pose.pose.orientation
            #agent_yaw = np.degrees(np.arctan2(2.0 * (q.y * q.z + q.w *q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z))

            velocities_msg.angular.x = 0.0
            velocities_msg.angular.y = 0.0
            velocities_msg.angular.z = 0.0#max(min((target_yaw - agent_yaw) * 0.5, 5.0), -5.0)
            
            acceleration_msg.linear.x = 0.0
            acceleration_msg.linear.y = 0.0
            acceleration_msg.linear.z = 0.0

            acceleration_msg.angular.x = 0.0
            acceleration_msg.angular.y = 0.0
            acceleration_msg.angular.z = 0.0
            
            transform_msgs.translation = translation_msg
            transform_msgs.rotation = rotation_msg
            
            trajpt_msg.transforms.append(transform_msgs)
            trajpt_msg.velocities.append(velocities_msg)
            trajpt_msg.accelerations.append(acceleration_msg)
            
            trajset_msg.points.append(trajpt_msg)
            
            header_msg.stamp = rospy.Time.now()
            trajset_msg.header = header_msg

            cmdPub.publish(trajset_msg)
        rate.sleep()


def publish_text_viz(msg):
    start_time=time.time()
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
    
    text_viz_pub.publish(marker)
    end_time = time.time()
    elapsed_time = end_time - start_time
    with open("heree.txt", "a") as file:
       file.write(f"Time taken to construct adjacency: {elapsed_time:.4f} seconds\n")
   
   
def create_marker(coord, color_b):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.scale.x = grid_resolution
        marker.scale.y = grid_resolution
        marker.scale.z = grid_resolution
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = color_b
        marker.color.a = 0.9
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = coord[0]
        marker.pose.position.y = coord[1]
        marker.pose.position.z = coord[2]
        return marker

def publish_graph_viz(coords, adj):
    # start_time=time.time()
    global waypoint, namespace, viz_pub

    marker_array = MarkerArray()


    # Loop through each coordinate and add markers for isolated nodes
    for indx, coord in enumerate(coords):
        if not np.any(adj[:, indx]):  
            marker = create_marker(coord, color_b=0.0)
            marker_array.markers.append(marker)
        elif not np.any(adj[indx, :]): 
            marker = create_marker(coord, color_b=1.0)
            marker_array.markers.append(marker)

    # Add the target marker
    marker = Marker()
    marker.header.frame_id = "world"
    marker.type = Marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = grid_resolution / 5.0
    marker.scale.y = grid_resolution / 5.0
    marker.scale.z = grid_resolution / 5.0
    marker.color.a = 1.0
    marker.color.r = 0.0
    if namespace == "jurong":
        marker.color.g = 0.0
        marker.color.b = 1.0
    else:
        marker.color.g = 1.0
        marker.color.b = 0.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = coords[target][0]
    marker.pose.position.y = coords[target][1]
    marker.pose.position.z = coords[target][2]
    marker_array.markers.append(marker)

    # Assign unique IDs to all markers
    for id, m in enumerate(marker_array.markers):
        m.id = id

    # Publish the marker array
    viz_pub.publish(marker_array)
    # end_time = time.time()
    # elapsed_time = end_time - start_time
    # with open("time.txt", "a") as file:
    #    file.write(f"Time taken to construct adjacency: {elapsed_time:.4f} seconds\n")
   



def main():
    # init
    global cmdPub, waypoint, command_thread, update_from_neighbor_thread, coordinates, target, grid_resolution, scenario 
    global namespace, debug, adjacency, update, mutex, adjacency_final, area_details, viz_pub,text_viz_pub, adjacency_neigh
  
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
		
    rospy.init_node(namespace, anonymous=True)

    # subscribe to self topics
    rospy.Subscriber("/"+namespace+"/ground_truth/odometry", Odometry, odomCallback)
    rospy.Subscriber("/"+namespace+"/command/targetPoint", Point, targetCallback)
    rospy.Subscriber("/"+namespace+"/command/yaw", Float32, yawCallback)
    rospy.Subscriber("/"+namespace+"/command/velocity", Float32, veloCallback)
    
    # create command publisher
    cmdPub = rospy.Publisher("/"+namespace+"/command/trajectory", MultiDOFJointTrajectory, queue_size=1)
    # adjacency vis pub
    viz_pub = rospy.Publisher("/"+namespace+"/adjacency_viz", MarkerArray, queue_size=1)
    # occupied coordinates publisher
    occ_pub = rospy.Publisher('/'+namespace+'/occupancy_coords', Int16MultiArray, queue_size=1)
    # occupied coordinates publisher
    arrival_pub = rospy.Publisher('/'+namespace+'/arrived_at_target', Bool, queue_size=1)
    #text viz pub
    text_viz_pub = rospy.Publisher("/"+namespace+"/text_viz", Marker, queue_size=1)
    rate = rospy.Rate(10)######################################################################svisto



    # Get Neighbor Positions
    rospy.Subscriber("/"+namespace+"/nbr_odom_cloud", PointCloud2, neighCallback)

    # Create a ppcom publisher
    # Wait for service to appear
    log_info("Waiting for ppcom")
    rospy.wait_for_service('/create_ppcom_topic')
    # Create a service proxy
    create_ppcom_topic = rospy.ServiceProxy('/create_ppcom_topic', CreatePPComTopic)
    # Register the topic with ppcom router
    create_ppcom_topic(namespace, ['all'], '/'+namespace+'/occupancy_coords', 'std_msgs', 'Int16MultiArray')
    create_ppcom_topic(namespace, ['all'], '/'+namespace+'/adjacency', 'std_msgs', 'Int16MultiArray')
    # Register the topic with ppcom router
    if namespace == 'jurong':
        create_ppcom_topic(namespace, ['raffles'], '/'+namespace+'/command/update_done', 'std_msgs', 'Bool')
        create_ppcom_topic(namespace, ['raffles'], '/'+namespace+'/command/update', 'std_msgs', 'Bool')
    else:
        create_ppcom_topic(namespace, ['jurong'], '/'+namespace+'/command/update_done', 'std_msgs', 'Bool')
        create_ppcom_topic(namespace, ['jurong'], '/'+namespace+'/command/update', 'std_msgs', 'Bool')

    #Get inspection area details
    log_info("Waiting for area details")
    area_details = rospy.wait_for_message("/world_coords/"+namespace, area) # here we get the message from gcs for the operate value 
    
    #TO ENSURE THAT THE SPECIFIC NAMESPASE PATH OR NO PATH TAKE THE AREA DETAILS
    # with open("confirm_area_details.txt", "a") as file:
    #     file.write(namespace+" sketto  "+ str(area_details)+"/n")
    xrange = range(int(area_details.minPoint.x + area_details.resolution.data/2), int(area_details.minPoint.x + area_details.size.x * area_details.resolution.data - area_details.resolution.data/2) + int(area_details.resolution.data), int(area_details.resolution.data)) # This is the resolution of the area, meaning the spacing between the grid points in the x, y, and z directions.


    yrange = range(int(area_details.minPoint.y + area_details.resolution.data/2), int(area_details.minPoint.y + area_details.size.y * area_details.resolution.data - area_details.resolution.data/2) + int(area_details.resolution.data), int(area_details.resolution.data)) 
    zrange = range(int(area_details.minPoint.z + area_details.resolution.data/2), int(area_details.minPoint.z + area_details.size.z * area_details.resolution.data - area_details.resolution.data/2) + int(area_details.resolution.data), int(area_details.resolution.data)) 
    

    # Constructing the graph
    log_info("Constructing initial graph")
    coordinates = np.asarray([(x,y,z) for x in xrange for y in yrange for z in zrange]) #This iterates over every combination of x, y, and z values within the ranges defined by xrange, yrange, and zrange
    adjacency_org = construct_adjacency(area_details, coordinates)
   #print the adjacency_org with the new construct_adjacency function that i changed
    # with open("adjacency_org2.txt", "w") as file:
    #  for row in adjacency_org:
    #     # Convert each row to a string and write it to the file
    #     file.write(' '.join(map(str, row)) + '\n')
#### ta idia me path
    # Get obstacles' coordinates
    log_info("Waiting for octomap")

    occupancy_coords = rospy.wait_for_message('/'+namespace+'/octomap_point_cloud_centers', PointCloud2) # Get obstacles' coordinates from octomap which are 50 metre radius, when the explorer is in the grouth
    log_info("translating octomap")
    occupancy_coords = sensor_msgs.point_cloud2.read_points(occupancy_coords, skip_nans=True, field_names=['x','y','z']) 
    
    #occupancy_coords.append(gcs_pos) # vazw kai gcs san occoupied
    #to see how much the occupancy_coords prints
    # er=list(occupancy_coords)
    # with open("test2.txt", "a") as file:
    #          file.write(str(namespace)+" my obst indx are " +str(er) +"\n")
    log_info("calling adjacency update")
    adjacency, adjacency_neigh = update_adjacency(adjacency_org, coordinates, occupancy_coords)
    log_info("publishing adjacency")
    publish_graph_viz(coordinates, adjacency) # "Waiting for adjacency build" 318 in ex_path wait this function 
   
   
    # create ros control thread
    waypoint = (-3000,-3000,-3000)
    command_thread = threading.Thread(target=go_to_point)
    command_thread.start()

    # create map merge thread
    update_from_neighbor_thread = threading.Thread(target=update_from_neighbor, args=(coordinates,)) 
    update_from_neighbor_thread.start()

    #print_score_thread = threading.Thread(target=print_score_interest_points)
    #print_score_thread.start()

    log_info("Waiting for target point") ###
    
    rospy.wait_for_message("/"+namespace+"/command/targetPoint", Point)

    octomap_length = 0
    # end_time = time.time()
    # elapsed_time = end_time - start_time
    # with open("time.txt", "a") as file:
    #    file.write(f"Time taken to construct adjacency: {elapsed_time:.4f} seconds "+namespace+"\n")
    tree = KDTree(coordinates)

    while not rospy.is_shutdown():
        try:
            #start_time=time.time()
            _,agent_index =  tree.query((odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z),k=1)
            #agent_index = closest_node_index_1((odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z),coordinates)
            path = dijkstra(adjacency_neigh, arrival_pub, agent_index, target)# vriskei to kltro monopati based ta liotera hops gia na paei sto target
            waypoint = coordinates[path[1]] # enable the go_to_point

            clear_agent_box(6, namespace)
            occupancy_coords_msg = rospy.wait_for_message('/'+namespace+'/octomap_point_cloud_centers', PointCloud2)#pkoianei ta cords pou fatsise o explorer
            occupancy_coords = sensor_msgs.point_cloud2.read_points(occupancy_coords_msg, skip_nans=True, field_names=['x','y','z'])
            if update: # ksekina na einai me true dld akoma eimaste sta target
                # log_info("Updating map")
                mutex.acquire()
                adjacency, adjacency_neigh = update_adjacency(adjacency_org,coordinates, occupancy_coords)
                mutex.release()
                # arr = np.sum(adjacency, axis=0)#collumn
                # # obstacle_indicies = np.where(arr == 0)[0].astype(int)
                # occupied_msg = Int16MultiArray()
                # occupied_msg.data = np.where(arr == 0)[0].astype(int) #This returns the indices of the array arr where the sum is 0
                occupied_msg = Int16MultiArray()
                occupied_msg.data = np.flatnonzero(np.sum(adjacency, axis=0) == 0).astype(int)

                # # for ind in obstacle_indicies:
                #     occupied_msg.data.append(ind)
                occ_pub.publish(occupied_msg)# otan telleiwsei apo ta targets tote tha parei o allos explorer ton xarti tou
                

            else:
                           
                if abs(octomap_length - occupancy_coords_msg.width) > 20: #ean ehoume >20 nea occupied cords
                    mutex.acquire()
                    log_info("Updating Map")
                    # publish_text_viz("Map Updated")
                    octomap_length = occupancy_coords_msg.width
                    try:
                        adjacency_final, adjacency_neigh = update_adjacency(adjacency_final,coordinates, occupancy_coords)
                        # adjacency_neigh = update_adjacency_with_neighbors(adjacency_final)
                    except:
                        pass
                    finally:
                        mutex.release()
                else:
                    adjacency_neigh = update_adjacency_with_neighbors(adjacency_final)
                # publish_text_viz("")
            
        except Exception as e:
            pass
        
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("terminating...")
        command_thread.terminate()
        update_from_neighbor_thread.terminte()
    except Exception as e:
        traceback.print_exc()
    finally:
        exit()
