#!/usr/bin/env python

import os
import rospy
import sys
from karmasim_ros_wrapper.msg import CarControls
from karmasim_dev_pkg.msg import SampleMessage
from karmasim_ros_wrapper.msg import UxvStates,Uxv,PointsOfInterest,PointOfInterest
from geometry_msgs.msg import Pose, Quaternion
from karmasim_ros_wrapper.srv import VehicleStart, VehicleStop, VehicleUnload, VehicleLoad
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Range
script_dir = os.path.dirname(__file__)
rel_path = '../res/ydc_formal.txt'
abs_file_path = os.path.join(script_dir, rel_path)
filehandle = open(abs_file_path)
lines = filehandle.readlines()
line_count = len(lines)
neighbor = list(range(line_count))
neig_pos = list(range(line_count))

i = 0
c=0
for line in lines:

    if line.find('0.5') == -1:
        neighbor[i] = line.strip()
    i+=1
    if line.find('0.5') != -1 :
        neig_pos[c] = line.strip()
    c+=1

while not i<1:
    for index1 in neighbor:
        if  type(index1) == type(4) :

            neighbor.remove(index1)

    i-=1

while not c<1:
    for index in neig_pos:
        if type(index) == type(int()) :
            neig_pos.remove(index)
    c-=1
a = 0
for index in neighbor:

    neighbor[a] = index.split('-')
    a+=1
b = 0

for index in neig_pos:

    neig_pos[b] = index.split(" (")
    b+=1
neig_position_list=[]
for neig_position in neig_pos:
    aaa={'name':None,'x':None,'y':None,'z':None}
    aaa['name'] = neig_position[0][:4]
    strr = aaa['name']
    strr=strr.strip()
    aaa['name']=strr

    if not 'yol' in neig_position[0] and not 'ara' in neig_position[-1]:
        
        pos = neig_position[1]
        pos=pos.split(',')
        aaa['x']=float(pos[0])
        aaa['y']=float(pos[1])
        aaa['z']=pos[2]
        if ')' in  aaa['z']:

            aaa['z']=aaa['z'].replace(')','')
        neig_position_list.append(aaa)
    if 'yol' in neig_position[0]:
        pos = neig_position[0].split('\t')
        pos = pos[1]

        pos=pos.split(',')

        aaa['x']=pos[0]
        aaa['y']=(pos[1])
        aaa['z']=pos[2]
        if '(' in  aaa['x']:

            aaa['x']=aaa['x'].replace('(','')
        if ')' in  aaa['z']:

            aaa['z']=aaa['z'].replace(')','')
        aaa['x']= float(aaa['x'])
        aaa['y']= float(aaa['y'])
        neig_position_list.append(aaa)

    if 'ara' in neig_position[-1]:
        pass
neighbor_list=[]
for neighbor_indexxx in neighbor:
    if 'wp' in neighbor_indexxx[0]:
        neighbor_list.append(neighbor_indexxx)
for neig in neig_position_list:
    name = neig['name']
    globals()[name]=neig
"""
wp1={'x':round(85),'y':round(-142.6),'name':'wp1'}
wp2={'x':round(85),'y':round(-92.6),'name':'wp2'}
wp3={'x':round(85),'y':round(77.2),'name':'wp3'}
wp4={'x':round(30.1),'y':round(77.2),'name':'wp4'}
wp5={'x':round(-75),'y':round(77.2),'name':'wp5'}
wp6={'x':round(-129.9),'y':round(77.2),'name':'wp6'}
wp7={'x':round(-129.9),'y':round(-27.6),'name':'wp7'}
wp8={'x':round(-129.9),'y':round(-142.8),'name':'wp8'}
wp9={'x':round(30.1),'y':round(-142.8),'name':'wp9'}
wp10={'x':round(30.1),'y':round(-92.6),'name':'wp10'}
wp11={'x':round(30.1),'y':round(-27.7),'name':'wp11'}
wp12={'x':round(30.1),'y':round(27.5),'name':'wp12'}
wp13={'x':round(-25),'y':round(27.5),'name':'wp13'}
wp14={'x':round(-25),'y':round(-27.6),'name':'wp14'}
wp15={'x':round(-75),'y':round(-27.6),'name':'wp15'}
wp16={'x':0,'y':round(-27.7),'name':'wp16'}

wp1={'x':(85),'y':(-142.6),'name':'wp1'}
wp2={'x':(85),'y':(-92.6),'name':'wp2'}
wp3={'x':(85),'y':(77.2),'name':'wp3'}
wp4={'x':(30.1),'y':(77.2),'name':'wp4'}
wp5={'x':(-75),'y':(77.2),'name':'wp5'}
wp6={'x':(-129.9),'y':(77.2),'name':'wp6'}
wp7={'x':(-129.9),'y':(-27.6),'name':'wp7'}
wp8={'x':(-129.9),'y':(-142.8),'name':'wp8'}
wp9={'x':(30.1),'y':(-142.8),'name':'wp9'}
wp10={'x':(30.1),'y':(-92.6),'name':'wp10'}
wp11={'x':(30.1),'y':(-27.7),'name':'wp11'}
wp12={'x':(30.1),'y':(27.5),'name':'wp12'}
wp13={'x':(-25),'y':(27.5),'name':'wp13'}
wp14={'x':(-25),'y':(-27.6),'name':'wp14'}
wp15={'x':(-75),'y':(-27.6),'name':'wp15'}
wp16={'x':0,'y':-27.7,'name':'wp16'}"""
import sys

class Vertex:
    def __init__(self, node):
        self.id = node
        self.adjacent = {}
        # Set distance to infinity for all nodes
        self.distance = sys.maxsize
        # Mark all nodes unvisited
        self.visited = False
        # Predecessor
        self.previous = None

    def add_neighbor(self, neighbor, weight=0):
        self.adjacent[neighbor] = weight

    def get_connections(self):
        return self.adjacent.keys()

    def get_id(self):
        return self.id

    def get_weight(self, neighbor):
        return self.adjacent[neighbor]

    def set_distance(self, dist):
        self.distance = dist

    def get_distance(self):
        return self.distance

    def set_previous(self, prev):
        self.previous = prev

    def set_visited(self):
        self.visited = True

    def __str__(self):
        return str(self.id) + ' adjacent: ' + str([x.id for x in self.adjacent])
    def __lt__(self,nxt):

        return self.distance
class Graph:
    def __init__(self):
        self.vert_dict = {}
        self.num_vertices = 0

    def __iter__(self):
        return iter(self.vert_dict.values())

    def add_vertex(self, node):
        self.num_vertices = self.num_vertices + 1
        new_vertex = Vertex(node)
        self.vert_dict[node] = new_vertex
        return new_vertex

    def get_vertex(self, n):
        if n in self.vert_dict:
            return self.vert_dict[n]
        else:
            return None

    def add_edge(self, frm, to, cost = 0):
        if frm not in self.vert_dict:
            self.add_vertex(frm)
        if to not in self.vert_dict:
            self.add_vertex(to)

        self.vert_dict[frm].add_neighbor(self.vert_dict[to], cost)
        self.vert_dict[to].add_neighbor(self.vert_dict[frm], cost)

    def get_vertices(self):
        return self.vert_dict.keys()

    def set_previous(self, current):
        self.previous = current

    def get_previous(self, current):
        return self.previous
def shortest(v, path):
    ''' make shortest path from v.previous'''
    if v.previous:
        path.append(v.previous.get_id())
        shortest(v.previous, path)
    return
import heapq
def dijkstra(aGraph, start, target):
    print ("Dijkstra's shortest path")
    # Set the distance for the start node to zero
    start.set_distance(0)

    # Put tuple pair into the priority queue
    unvisited_queue = [(v.get_distance(),v) for v in aGraph]
    heapq.heapify(unvisited_queue)

    while len(unvisited_queue):
        # Pops a vertex with the smallest distance
        uv = heapq.heappop(unvisited_queue)
        current = uv[1]
        current.set_visited()

        #for next in v.adjacent:
        for next in current.adjacent:
            # if visited, skip
            if next.visited:
                continue
            new_dist = current.get_distance() + current.get_weight(next)

            if new_dist < next.get_distance():
                next.set_distance(new_dist)
                next.set_previous(current)
                print ('updated : current = %s next = %s new_dist = %s' \
                       %(current.get_id(), next.get_id(), next.get_distance()))
            else:
                print ('not updated : current = %s next = %s new_dist = %s' \
                       %(current.get_id(), next.get_id(), next.get_distance()))

        # Rebuild heap
        # 1. Pop every item
        while len(unvisited_queue):
            heapq.heappop(unvisited_queue)
        # 2. Put all vertices not visited into the queue
        unvisited_queue = [(v.get_distance(),v) for v in aGraph if not v.visited]
        heapq.heapify(unvisited_queue)
import math
def dist_cal(x1=0.00,y1=0.00,x2=0.00,y2=0.00):
    return math.sqrt(((abs(float(x1)-float(x2))**2)+(abs(float(y1)-float(y2))**2)))
def find_wp(patient_pos,target_pos_list):
    swap = 10000
    for target_wp in target_pos_list:
        distance =dist_cal(patient_pos['x'],patient_pos['y'],target_wp['x'],target_wp['y'])

        if swap > abs(distance):
            swap =distance
            wp_name = target_wp['name']
    return swap,wp_name
def get_rota(patient_pos):

    g = Graph()

    """g.add_vertex('wp1')
    g.add_vertex('wp2')
    g.add_vertex('wp3')
    g.add_vertex('wp4')
    g.add_vertex('wp5')
    g.add_vertex('wp6')
    g.add_vertex('wp7')
    g.add_vertex('wp8')
    g.add_vertex('wp9')
    g.add_vertex('wp10')
    g.add_vertex('wp11')
    g.add_vertex('wp12')
    g.add_vertex('wp13')
    g.add_vertex('wp14')
    g.add_vertex('wp15')
    g.add_vertex('wp16')
    g.add_edge('wp1', 'wp2', dist_cal(wp1['x'],wp1['y'],wp2['x'],wp2['y']))
    g.add_edge('wp1', 'wp9', dist_cal(wp1['x'],wp1['y'],wp9['x'],wp9['y']))
    g.add_edge('wp2', 'wp3', dist_cal(wp2['x'],wp2['y'],wp3['x'],wp3['y']))
    g.add_edge('wp2', 'wp10', dist_cal(wp2['x'],wp2['y'],wp10['x'],wp10['y']))
    g.add_edge('wp3', 'wp4', dist_cal(wp3['x'],wp3['y'],wp4['x'],wp4['y']))
    g.add_edge('wp4', 'wp5', dist_cal(wp4['x'],wp4['y'],wp5['x'],wp5['y']))
    g.add_edge('wp4', 'wp12', dist_cal(wp4['x'],wp4['y'],wp12['x'],wp12['y']))
    g.add_edge('wp5', 'wp6', dist_cal(wp5['x'],wp5['y'],wp6['x'],wp6['y']))
    g.add_edge('wp5', 'wp15', dist_cal(wp5['x'],wp5['y'],wp15['x'],wp15['y']))
    g.add_edge('wp6', 'wp7', dist_cal(wp6['x'],wp6['y'],wp7['x'],wp7['y']))
    g.add_edge('wp7', 'wp8', dist_cal(wp7['x'],wp7['y'],wp8['x'],wp8['y']))
    g.add_edge('wp7', 'wp15', dist_cal(wp7['x'],wp7['y'],wp15['x'],wp15['y']))
    g.add_edge('wp8', 'wp9', dist_cal(wp8['x'],wp8['y'],wp9['x'],wp9['y']))
    g.add_edge('wp9', 'wp10', dist_cal(wp9['x'],wp9['y'],wp10['x'],wp10['y']))
    g.add_edge('wp10', 'wp11', dist_cal(wp10['x'],wp10['y'],wp11['x'],wp11['y']))
    g.add_edge('wp11', 'wp12', dist_cal(wp11['x'],wp11['y'],wp12['x'],wp12['y']))
    g.add_edge('wp11', 'wp16', dist_cal(wp11['x'],wp11['y'],wp16['x'],wp16['y']))
    g.add_edge('wp12', 'wp13', dist_cal(wp12['x'],wp12['y'],wp13['x'],wp13['y']))
    g.add_edge('wp13', 'wp14', dist_cal(wp13['x'],wp13['y'],wp14['x'],wp14['y']))
    g.add_edge('wp14', 'wp15', dist_cal(wp14['x'],wp14['y'],wp15['x'],wp15['y']))
    g.add_edge('wp14', 'wp16', dist_cal(wp14['x'],wp14['y'],wp16['x'],wp16['y']))
"""
    for nei in neig_position_list:
        g.add_vertex(nei['name'])
    for neighbor_ind in neighbor_list:
        for i in neig_position_list:
            global fisrt,second
            if i['name']==neighbor_ind[0]:

                fisrt = i
            if i['name']==neighbor_ind[1]:
                second= i
        g.add_edge(neighbor_ind[0],neighbor_ind[1],dist_cal(float(fisrt['x']),float(fisrt['y']),float(second['x']),float(second['y'])))




    print ('Graph data:')
    for v in g:
        for w in v.get_connections():
            vid = v.get_id()
            wid = w.get_id()
            print (' %s , %s, %3d)'  % ( vid, wid, v.get_weight(w)))
    wp_list = [wp1,wp2,wp3,wp4,wp5,wp6,wp7,wp8,wp9,wp10,wp11,wp12,wp13,wp14,wp15,wp16]
    target_wp = find_wp(patient_pos,wp_list)
    #dijkstra(g, g.get_vertex('wp1'), g.get_vertex('wp5'))
    dijkstra(aGraph=g,start=g.get_vertex('wp16'),target=g.get_vertex(target_wp[1]))
    target = g.get_vertex(target_wp[1])
    path = [target.get_id()]
    shortest(target, path)
    
    shortest_path = path[::-1]
    #shortest_path.pop(0)
    
    station= shortest_path[0]

    target_pos_list = []
    ccounter=0
    for station in shortest_path:
        for wp in wp_list:
            if station == wp['name']:
                target_pos_list.append(wp)
                ccounter += 1
   
    
    return target_pos_list
pois_requirement_ = PointOfInterest.requirement
pois_type = PointOfInterest.type
pois_name = PointOfInterest.name
pois_list_patient =[]
def PointsOfInterest_callback(dataa):
    global pois_list_patient
    pois_list_patient = []
    for poisof in dataa.pois:
        if poisof.type==2:
            global pois_name
            pois_name = poisof.name
            global pois_requirement_
            pois_requirement_ = poisof.requirement
            global pois_type
            pois_type = poisof.type
            pois_list_patient.append(poisof)

def get_rota_for_hospitals(hospital_pos,patient_pos):




    g = Graph()
    for nei in neig_position_list:
        g.add_vertex(nei['name'])
    for neighbor_ind in neighbor_list:
        for i in neig_position_list:
            global fisrt,second
            if i['name']==neighbor_ind[0]:

                fisrt = i
            if i['name']==neighbor_ind[1]:
                second= i
        g.add_edge(neighbor_ind[0],neighbor_ind[1],dist_cal(float(fisrt['x']),float(fisrt['y']),float(second['x']),float(second['y'])))
    """g.add_edge('wp1', 'wp2', dist_cal(wp1['x'],wp1['y'],wp2['x'],wp2['y']))
    g.add_edge('wp1', 'wp9', dist_cal(wp1['x'],wp1['y'],wp9['x'],wp9['y']))
    g.add_edge('wp2', 'wp3', dist_cal(wp2['x'],wp2['y'],wp3['x'],wp3['y']))
    g.add_edge('wp2', 'wp10', dist_cal(wp2['x'],wp2['y'],wp10['x'],wp10['y']))
    g.add_edge('wp3', 'wp4', dist_cal(wp3['x'],wp3['y'],wp4['x'],wp4['y']))
    g.add_edge('wp4', 'wp5', dist_cal(wp4['x'],wp4['y'],wp5['x'],wp5['y']))
    g.add_edge('wp4', 'wp12', dist_cal(wp4['x'],wp4['y'],wp12['x'],wp12['y']))
    g.add_edge('wp5', 'wp6', dist_cal(wp5['x'],wp5['y'],wp6['x'],wp6['y']))
    g.add_edge('wp5', 'wp15', dist_cal(wp5['x'],wp5['y'],wp15['x'],wp15['y']))
    g.add_edge('wp6', 'wp7', dist_cal(wp6['x'],wp6['y'],wp7['x'],wp7['y']))
    g.add_edge('wp7', 'wp8', dist_cal(wp7['x'],wp7['y'],wp8['x'],wp8['y']))
    g.add_edge('wp7', 'wp15', dist_cal(wp7['x'],wp7['y'],wp15['x'],wp15['y']))
    g.add_edge('wp8', 'wp9', dist_cal(wp8['x'],wp8['y'],wp9['x'],wp9['y']))
    g.add_edge('wp9', 'wp10', dist_cal(wp9['x'],wp9['y'],wp10['x'],wp10['y']))
    g.add_edge('wp10', 'wp11', dist_cal(wp10['x'],wp10['y'],wp11['x'],wp11['y']))
    g.add_edge('wp11', 'wp12', dist_cal(wp11['x'],wp11['y'],wp12['x'],wp12['y']))
    g.add_edge('wp11', 'wp16', dist_cal(wp11['x'],wp11['y'],wp16['x'],wp16['y']))
    g.add_edge('wp12', 'wp13', dist_cal(wp12['x'],wp12['y'],wp13['x'],wp13['y']))
    g.add_edge('wp13', 'wp14', dist_cal(wp13['x'],wp13['y'],wp14['x'],wp14['y']))
    g.add_edge('wp14', 'wp15', dist_cal(wp14['x'],wp14['y'],wp15['x'],wp15['y']))
    g.add_edge('wp14', 'wp16', dist_cal(wp14['x'],wp14['y'],wp16['x'],wp16['y']))"""





    print ('Graph data:')
    for v in g:
        for w in v.get_connections():
            vid = v.get_id()
            wid = w.get_id()
            print (' %s , %s, %3d)'  % ( vid, wid, v.get_weight(w)))
    wp_list = [wp1,wp2,wp3,wp4,wp5,wp6,wp7,wp8,wp9,wp10,wp11,wp12,wp13,wp14,wp15,wp16]
    target_wp = find_wp(patient_pos,wp_list)
    hospital_wp = find_wp(hospital_pos,wp_list)
    
    #dijkstra(g, g.get_vertex('wp1'), g.get_vertex('wp5'))
    dijkstra(aGraph=g,start=g.get_vertex(str(target_wp[1])),target=g.get_vertex(hospital_wp[1]))
    target = g.get_vertex(hospital_wp[1])
    path = [target.get_id()]
    shortest(target, path)
    
    shortest_path = path[::-1]


    station= shortest_path[0]
    
    target_pos_list = []
    ccounter=0
    for station in shortest_path:
        for wp in wp_list:
            if station == wp['name']:


                target_pos_list.append(wp)
                ccounter += 1
    

    return target_pos_list


pois_list_ = []
pois_pose = Pose()
pois_req = PointOfInterest.requirement
pois_name = PointOfInterest.name
def poi_callback(data):
    global pois_list_
    pois_list_ = list(range(19))
    for poisof in data.pois:
        for i in range(19):
            global pois_req,pois_name,pois_pose
            pois_req = poisof.requirement
            pois_name = poisof.name
            pois_pose = poisof.pose
            pois_list_[i] = [pois_pose,pois_name,pois_req]


roll =0
pitch = 0
yaw=0
uxv_name = ''

def get_rotation(msg):
    global roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)


def readFile(filename):
    filehandle = open(filename)
    
    global first_wp_
    first_wp_ = filehandle.read()

    filehandle.close()

uxv_pose_ = Pose()
uxv_fuel_ = Uxv.fuel_current
uxv_unavailable = Uxv.is_unavailable
uxv_ori = Quaternion()
roll_instant =0
pitch_instant = 0
yaw_instant=0

def uxv_states_callback(data):

    for uxv in data.uxvs:
        if uxv.name == uxv_name:
            global uxv_pose_,uxv_ori
            global roll_instant, pitch_instant, yaw_instant
            uxv_pose_ = uxv.pose
            uxv_ori = uxv.pose.orientation
            orientation_list = [uxv_ori.x, uxv_ori.y, uxv_ori.z, uxv_ori.w]
            (roll_instant, pitch_instant, yaw_instant) = euler_from_quaternion(orientation_list)

            global uxv_fuel_
            uxv_fuel_ =uxv.fuel_current
            global uxv_unavailable
            uxv_unavailable = uxv.is_unavailable
def GPSYaw_callback(dataa):
    global ugv_yaw
    ugv_yaw = dataa.yaw

def diffenance(a,b):
    return abs(float(a)-float(b))

"""
def collision_avoidance(ugv_pub):
    
    while True:
        if float(dist_front_center.range)<float(5):
            go_back(dist_front_center)
            break
        elif float(dist_back_center.range)<float(5):
            go_ahead(dist_back_center)
            break
        elif float(dist_front_center.range)<float(5) and float(dist_front_left.range)<float(5):
            con = (float(dist_front_center.range)+float(dist_front_left.range))/2
            turn_right_for_collision(con)
            turn_right(yaw_instant)
            break
        elif float(dist_front_center.range)<float(5) and float(dist_front_right.range)<float(5):
            con = (float(dist_front_center.range)+float(dist_front_right.range))/2
            turn_left_for_collision(con)
            turn_left(yaw_instant)
            break
        elif float(dist_back_center.range)<float(5) and float(dist_front_left.range)<float(5):
            con = (float(dist_back_center.range)+float(dist_front_left.range))/2
            turn_right_for_collision_rear(con)
            turn_right(yaw_instant)
            break
        elif float(dist_back_center.range)<float(5) and float(dist_front_right.range)<float(5):
            con = (float(dist_back_center.range)+float(dist_front_right.range))/2
            turn_left_for_collision_rear(con)
            turn_left(yaw_instant)
            break
        elif float(dist_back_center.range)<float(5) and float(dist_front_center.range)<float(5):
            con = (float(dist_back_center.range)+float(dist_front_center.range))/2
            print('waiting.....')
            wait(con)
        else:
            break
"""
def takeoff(a):
    target_throttle = 0.0
    target_steering = 0.0
    car_cmd = CarControls()
    car_cmd.handbrake = False
    car_cmd.manual = False
    car_cmd.gear_immediate = True
    if (yaw_instant < (-pi/2)*0.75 and yaw_instant > (-pi/2)*1.25) or yaw_instant > (+pi/2)*0.75 and yaw_instant < (+pi/2)*1.25:
        tar = int(float(a['y']))
    elif (yaw_instant < 0.25 and yaw_instant > -0.25) or (yaw_instant > pi*0.75 and yaw_instant < pi*1.25):
        tar = int(float(a['x']))
    while not rospy.is_shutdown():
        ugv_pos_y= uxv_pose_.position.y
        
        condisition = diffenance(int(ugv_pos_y),tar)
        if condisition>10:

            car_cmd.brake = 0
            target_throttle = 0.6
            if target_throttle > 0:
                car_cmd.manual = False
                if car_cmd.manual_gear != 1:
                    car_cmd.manual_gear = 1
            car_cmd.throttle = target_throttle
            car_cmd.steering = target_steering
            ugv_pub.publish(car_cmd)
        if condisition < 10:
            target_throttle = 0
            car_cmd.brake = 1
            car_cmd.throttle = target_throttle
            car_cmd.steering = target_steering
            ugv_pub.publish(car_cmd)
            break
pi = math.pi

def turn_left(first_yaw):
    car_cmd = CarControls()
    car_cmd.handbrake = False
    car_cmd.manual = False
    car_cmd.gear_immediate = True

    while not rospy.is_shutdown():
       
        if round((first_yaw-(pi/2)),2)==round(-pi,2):
           if yaw_instant>0:
               
               target_throttle = 0.0
               target_steering = 0.0
               car_cmd.throttle = target_throttle
               car_cmd.steering = target_steering
               ugv_pub.publish(car_cmd)
               
               break
           else:
               target_steering = -0.25
               target_throttle = 0.25
               car_cmd.brake = 0
               if target_throttle > 0:
                   car_cmd.manual = False
                   if car_cmd.manual_gear != 1:
                       car_cmd.manual_gear = 1
               car_cmd.throttle = target_throttle
               car_cmd.steering = target_steering
               ugv_pub.publish(car_cmd)
        else:
            if yaw_instant>(first_yaw-(pi/2)):

                target_steering = -0.25
                target_throttle = 0.25
                car_cmd.brake = 0
                if target_throttle > 0:
                    car_cmd.manual = False
                    if car_cmd.manual_gear != 1:
                        car_cmd.manual_gear = 1
                car_cmd.throttle = target_throttle
                car_cmd.steering = target_steering
                ugv_pub.publish(car_cmd)
            else:
                target_throttle = 0.0
                target_steering = 0.0
                car_cmd.throttle = target_throttle
                car_cmd.steering = target_steering
                ugv_pub.publish(car_cmd)
                
                break
def turn_right(first_yaw):
    car_cmd = CarControls()
    car_cmd.handbrake = False
    car_cmd.manual = False
    car_cmd.gear_immediate = True
    while not rospy.is_shutdown():
        
        if int((first_yaw-(pi/2)))==int(-pi):
            
            if yaw_instant>0 :
                target_throttle = 0.0
                target_steering = 0.0
                car_cmd.throttle = target_throttle
                car_cmd.steering = target_steering
                ugv_pub.publish(car_cmd)
                
                break
            else:
                target_steering = 0.25
                target_throttle = 0.25
                car_cmd.brake = 0
                if target_throttle > 0:
                    car_cmd.manual = False
                    if car_cmd.manual_gear != 1:
                        car_cmd.manual_gear = 1
                car_cmd.throttle = target_throttle
                car_cmd.steering = target_steering
                ugv_pub.publish(car_cmd)
        else:
            if yaw_instant<(first_yaw+(pi/2)):

                target_steering = 0.25
                target_throttle = 0.25
                car_cmd.brake = 0
                if target_throttle > 0:
                    car_cmd.manual = False
                    if car_cmd.manual_gear != 1:
                        car_cmd.manual_gear = 1
                car_cmd.throttle = target_throttle
                car_cmd.steering = target_steering
                ugv_pub.publish(car_cmd)
            else:
                target_throttle = 0.0
                target_steering = 0.0
                car_cmd.throttle = target_throttle
                car_cmd.steering = target_steering
                ugv_pub.publish(car_cmd)
                
                break

def turn_left_for_collision(condition):
    car_cmd = CarControls()
    car_cmd.handbrake = False
    car_cmd.manual = False
    car_cmd.gear_immediate = True

    while not rospy.is_shutdown():
        if condition<6:

            target_steering = -0.4
            target_throttle = 0.4
            car_cmd.brake = 0
            if target_throttle > 0:
                car_cmd.manual = False
                if car_cmd.manual_gear != 1:
                    car_cmd.manual_gear = 1
            car_cmd.throttle = target_throttle
            car_cmd.steering = target_steering
            ugv_pub.publish(car_cmd)
            rospy.sleep(3)

        else:
            target_throttle = 0.4
            target_steering = 0.4
            car_cmd.throttle = target_throttle
            car_cmd.steering = target_steering
            ugv_pub.publish(car_cmd)
            
            rospy.sleep(3)
            break
def turn_right_for_collision(condition):
    car_cmd = CarControls()
    car_cmd.handbrake = False
    car_cmd.manual = False
    car_cmd.gear_immediate = True
    while not rospy.is_shutdown():
        if condition<6:

            target_steering = 0.25
            target_throttle = 0.25
            car_cmd.brake = 0
            if target_throttle > 0:
                car_cmd.manual = False
                if car_cmd.manual_gear != 1:
                    car_cmd.manual_gear = 1
            car_cmd.throttle = target_throttle
            car_cmd.steering = target_steering
            ugv_pub.publish(car_cmd)
            rospy.sleep(3)

        else:
            target_throttle = -0.25
            target_steering = 0.25
            car_cmd.throttle = target_throttle
            car_cmd.steering = target_steering
            ugv_pub.publish(car_cmd)
            
            rospy.sleep(3)
            break
def turn_left_for_collision_rear(condition):
    car_cmd = CarControls()
    car_cmd.handbrake = False
    car_cmd.manual = False
    car_cmd.gear_immediate = True

    while not rospy.is_shutdown():
        if condition<6:

            target_steering = -0.4
            target_throttle = -0.4
            car_cmd.brake = 0
            if target_throttle > 0:
                car_cmd.manual = False
                if car_cmd.manual_gear != 1:
                    car_cmd.manual_gear = 1
            car_cmd.throttle = target_throttle
            car_cmd.steering = target_steering
            ugv_pub.publish(car_cmd)
            rospy.sleep(3)

        else:
            target_throttle = 0.4
            target_steering = -0.4
            car_cmd.throttle = target_throttle
            car_cmd.steering = target_steering
            ugv_pub.publish(car_cmd)
            
            rospy.sleep(3)

            break
def turn_right_for_collision_rear(condition):
    car_cmd = CarControls()
    car_cmd.handbrake = False
    car_cmd.manual = False
    car_cmd.gear_immediate = True
    while not rospy.is_shutdown():
        if condition<6:

            target_steering = 0.25
            target_throttle = -0.25
            car_cmd.brake = 0
            if target_throttle > 0:
                car_cmd.manual = False
                if car_cmd.manual_gear != 1:
                    car_cmd.manual_gear = 1
            car_cmd.throttle = target_throttle
            car_cmd.steering = target_steering
            ugv_pub.publish(car_cmd)
            rospy.sleep(3)

        else:
            target_throttle = -0.25
            target_steering = -0.25
            car_cmd.throttle = target_throttle
            car_cmd.steering = target_steering
            ugv_pub.publish(car_cmd)
            
            rospy.sleep(3)
            break
def go_ahead(condisition):
    
    
    
    
    car_cmd = CarControls()
    car_cmd.handbrake = False
    car_cmd.manual = False
    car_cmd.gear_immediate = True
    
    if condisition>5:
        car_cmd.brake = 0
        target_throttle = 0.4
        if target_throttle > 0:
            car_cmd.manual = False
            if car_cmd.manual_gear != 1:
                car_cmd.manual_gear = 1
        car_cmd.throttle = target_throttle
        ugv_pub.publish(car_cmd)
    elif condisition<5:
        car_cmd.brake = 0
        target_throttle = 0
        if target_throttle > 0:
            car_cmd.manual = False
            if car_cmd.manual_gear != 1:
                car_cmd.manual_gear = 1
        car_cmd.throttle = target_throttle
        ugv_pub.publish(car_cmd)
def go_back(condisition):
    
    
    
    
    car_cmd = CarControls()
    car_cmd.handbrake = False
    car_cmd.manual = False
    car_cmd.gear_immediate = True
    

    if condisition>5:
        car_cmd.brake = 0
        target_throttle = -0.4
        if target_throttle < 0:
            car_cmd.manual = True
            car_cmd.manual_gear = -1
        car_cmd.throttle = target_throttle
        ugv_pub.publish(car_cmd)
    elif condisition<5:
        car_cmd.brake = 0
        target_throttle = 0
        car_cmd.throttle = target_throttle
        ugv_pub.publish(car_cmd)
def go_to_position(target_pos_list,uxv_pose_):
    
    
    
    counter = 0
    for target_pos in target_pos_list:
        
        
        

        sub = rospy.Subscriber('/karmasim_node/'+str(uxv_name)+'/odom_local_ned', Odometry, get_rotation)
        first_yaw = yaw
        
        while not rospy.is_shutdown():
            ugv_pos_y = uxv_pose_.position.y
            ugv_pos_x = uxv_pose_.position.x
            
            
            
            if ((diffenance(target_pos['x'],ugv_pos_x)**2)-(diffenance(target_pos['y'],ugv_pos_y)**2))**0.5>5:
                if (first_yaw < 0.1 and first_yaw > -0.1) or (first_yaw > pi*0.95 and first_yaw < pi*1.05):
                    condisition = diffenance(ugv_pos_x,target_pos['x'])
                    go_ahead(condisition)

                    # x ekseni dogrultusunda git
                if (first_yaw > (+pi/2)*0.95 and first_yaw < (+pi/2)*1.05) or (first_yaw < (-pi/2)*0.95 and first_yaw > (-pi/2)*1.05):
                    condisition = diffenance(ugv_pos_y,target_pos['y'])
                    go_ahead(condisition)

                    # y ekseni dogrultusunda git

                #go to this position until reach
            else:
                if first_yaw < (-pi/2)*0.95 and first_yaw > (-pi/2)*1.05:
                    if target_pos['x']>target_pos_list[counter+1]['x']:
                        turn_right(first_yaw)
                    elif target_pos['x']<target_pos_list[counter+1]['x']:
                        turn_left(first_yaw)
                if first_yaw > (+pi/2)*0.95 and first_yaw < (+pi/2)*1.05:
                    if target_pos['x']>target_pos_list[counter+1]['x']:
                        turn_left(first_yaw)
                    elif target_pos['x']<target_pos_list[counter+1]['x']:
                        turn_right(first_yaw)
                if first_yaw > pi*0.95 and first_yaw < pi*1.05:
                    if target_pos['y']<target_pos_list[counter+1]['y']:
                        turn_right(first_yaw)
                    elif target_pos['y']>target_pos_list[counter+1]['y']:
                        turn_left(first_yaw)
                if first_yaw < 0.1 and first_yaw > -0.1 :
                    if target_pos['y']<target_pos_list[counter+1]['y']:
                        turn_left(first_yaw)
                    elif target_pos['y']>target_pos_list[counter+1]['y']:
                        turn_right(first_yaw)
                if target_pos == target_pos_list[len(target_pos_list)-1]:
                    break
        counter += 1

def check_distance_back_center(msg):
    global dist_back_center
    dist_back_center = msg

def check_distance_front_center(msg2):
    global dist_front_center
    dist_front_center = msg2

def check_distance_front_left(msg3):
    global dist_front_left
    dist_front_left = msg3

def check_distance_front_right(msg4):
    global dist_front_right
    dist_front_right = msg4

def wait(con):
    while not rospy.is_shutdown():
        if con<7:
            car_cmd = CarControls()
            car_cmd.brake = 0
            car_cmd.throttle = 0
            car_cmd.steering = 0
            ugv_pub.publish(car_cmd)
            rospy.sleep(5)
        else:
            break

back_yaw = 0
def get_yaw(first_yaw):
    global back_yaw
    back_yaw = first_yaw
def find_hospital(patient_pos,hospitals):


    hospital_pos_list = hospitals
    swap = 10000
    for hospital_pos in hospital_pos_list:
        distance =dist_cal(patient_pos['x'],patient_pos['y'],float(hospital_pos['x']),float(hospital_pos['y']))

        if swap > abs(distance):
            swap =distance
            hospital_pos_ = hospital_pos
            hospital_pos_['x'] = float(hospital_pos['x'])
            hospital_pos_['y'] = float(hospital_pos['y'])
            hospital_pos_['z'] = float(hospital_pos['z'])
    
    return hospital_pos_
def unload(target_pos,hospital_pos):
    
    
    
    cargo_type = target_pos['requirement']
    if cargo_type == 1 or cargo_type == 2:
        target_poi = target_pos['name']
    elif cargo_type == 5:
        target_poi = hospital_pos['name']
    rospy.wait_for_service('/karmasim_node/vehicle_unload')
    req = rospy.ServiceProxy('/karmasim_node/vehicle_unload', VehicleUnload)
    amount_number=1
    res = req(uxv_name, target_poi, cargo_type, amount_number)
    print('unload result: %s\t message: %s ' % (res.success, res.message))
def load(target_pos,hospital_pos):
    
    
    
    cargo_type = patient_pos['requirement']
    if cargo_type == 1:
        source_poi = hospital_pos['name']
    elif cargo_type == 2:
        source_poi = 'headquarter_1'
    elif cargo_type == 5:
        source_poi = target_pos['name']
    amount_number=1
    rospy.wait_for_service('/karmasim_node/vehicle_load')
    req = rospy.ServiceProxy('/karmasim_node/vehicle_load', VehicleLoad)
    res = req(uxv_name, source_poi, cargo_type, amount_number)
    print('unload result: %s\t message: %s ' % (res.success, res.message))

def find_yaw_target_for_minus_pi_divided_2(car_pos,target_pos):

    if int(car_pos['x'])<0 and int(target_pos['x'])<0:
        if int(car_pos['x'])< int(target_pos['x']):
            return 1 #turn left demek
        if int(car_pos['x'])> int(target_pos['x']):
            return 2 # turn right demek
    if int(car_pos['x'])<0 and target_pos['x']>0:
        if int(car_pos['x'])< int(target_pos['x']):
            return 1 # turn left demek
    if round(car_pos['x'])>0 and target_pos['x']>0:
        if round(car_pos['x'])< round(target_pos['x']):
            return 2 #turn left demek
        if round(car_pos['x'])> round(target_pos['x']):
            return 1
    if round(car_pos['x'])>0 and target_pos['x']<0:
        if round(car_pos['x'])> round(target_pos['x']):
            return 2
    if round(car_pos['y'])<0 and target_pos['y']<0:
        if round(car_pos['y'])>round(target_pos['y']):
            return 3 # go ahead demek
        if round(car_pos['y'])<round(target_pos['y']):
            return 4 # back demek
    if round(car_pos['y'])<0 and target_pos['y']>0:
        if round(car_pos['y'])<round(target_pos['y']):
            return 4
    if round(car_pos['y'])>0 and target_pos['y']>0:
        if round(car_pos['y'])<round(target_pos['y']):
            return 4
        if round(car_pos['y'])>round(target_pos['y']):
            return 3
    if round(car_pos['y'])>0 and target_pos['y']<0:
        if round(car_pos['y'])>round(target_pos['y']):
            return 3
def find_yaw_target_for_pi_divided_2(car_pos,target_pos):
    if int(car_pos['x'])<0 and target_pos['x']<0:
        if int(car_pos['x'])< int(target_pos['x']):
            return 2 #turn right demek
        if int(car_pos['x'])>int(target_pos['x']):
            return 1 # turn left demek
    if int(car_pos['x'])<0 and target_pos['x']>0:
        if int(car_pos['x'])< int(target_pos['x']):
            return 2 # turn right demek
    if int(car_pos['x'])>0 and target_pos['x']>0:
        if int(car_pos['x'])< int(target_pos['x']):
            return 2 #turn right demek
        if int(car_pos['x'])>int(target_pos['x']):
            return 1
    if int(car_pos['x'])>0 and target_pos['x']<0:
        if int(car_pos['x'])>int(target_pos['x']):
            return 1
    if int(car_pos['y'])<0 and target_pos['y']<0:
        if int(car_pos['y'])>int(target_pos['y']):
            return 4 # back demek
        if int(car_pos['y'])<int(target_pos['y']):
            return 3 # go ahead demek
    if int(car_pos['y'])<0 and target_pos['y']>0:
        if int(car_pos['y'])<int(target_pos['y']):
            return 3
    if int(car_pos['y'])>0 and target_pos['y']>0:
        if int(car_pos['y'])<int(target_pos['y']):
            return 3
        if int(car_pos['y'])>int(target_pos['y']):
            return 4
    if int(car_pos['y'])>0 and target_pos['y']<0:
        if int(car_pos['y'])>int(target_pos['y']):
            return 4
def find_yaw_target_for_pi(car_pos,target_pos):
    if int(car_pos['x'])<0 and target_pos['x']<0:
        if int(car_pos['x'])<target_pos['x']:
            return 3
        if int(car_pos['x'])>target_pos['x']:
            return 4
    if int(car_pos['x'])<0 and target_pos['x']>0:
        if int(car_pos['x'])<target_pos['x']:
            return 3
    if int(car_pos['x'])>0 and target_pos['x']>0:
        if int(car_pos['x'])<target_pos['x']:
            return 3
        if int(car_pos['x'])>target_pos['x']:
            return 4
    if int(car_pos['x'])>0 and target_pos['x']<0:
        if int(car_pos['x'])>target_pos['x']:
            return 4
    if int(car_pos['y'])<0 and target_pos['y']<0:
        if int(car_pos['y'])>target_pos['y']:
            return 2
        if int(car_pos['y'])<target_pos['y']:
            return 1
    if int(car_pos['y'])<0 and target_pos['y']>0:
        if int(car_pos['y'])<target_pos['y']:
            return 1
    if int(car_pos['y'])>0 and target_pos['y']>0:
        if int(car_pos['y'])<target_pos['y']:
            return 1
        if int(car_pos['y'])>target_pos['y']:
            return 2
    if int(car_pos['y'])>0 and target_pos['y']<0:
        if int(car_pos['y'])>target_pos['y']:
            return 2
def find_yaw_target_for_zero(car_pos,target_pos):
    
    rospy.sleep(20)
    if int(car_pos['x'])<0 and target_pos['x']<0:
        if int(car_pos['x'])<target_pos['x']:
            return 4
        if int(car_pos['x'])>target_pos['x']:
            return 3
    if int(car_pos['x'])<0 and target_pos['x']>0:
        if int(car_pos['x'])<target_pos['x']:
            return 4
    if int(car_pos['x'])>0 and target_pos['x']>0:
        if int(car_pos['x'])<target_pos['x']:
            return 4
        if int(car_pos['x'])>target_pos['x']:
            return 3
    if int(car_pos['x'])>0 and target_pos['x']<0:
        if int(car_pos['x'])>target_pos['x']:
            return 3
    if int(car_pos['y'])<0 and target_pos['y']<0:
        if int(car_pos['y'])>target_pos['y']:

            return 1
        if int(car_pos['y'])<target_pos['y']:
            return 2
    if int(car_pos['y'])<0 and target_pos['y']>0:
        if int(car_pos['y'])<target_pos['y']:
            return 2
    if int(car_pos['y'])>0 and target_pos['y']>0:
        if int(car_pos['y'])<target_pos['y']:
            return 2
        if int(car_pos['y'])>target_pos['y']:
            return 1
    if int(car_pos['y'])>0 and target_pos['y']<0:
        if int(car_pos['y'])>target_pos['y']:
            return 1

if __name__ == "__main__":
    
    
    
    rospy.init_node('contester_ugv_node')

    #Relative path example
    script_dir = os.path.dirname(__file__)
    rel_path = '../res/ydc_formal.txt'
    abs_file_path = os.path.join(script_dir, rel_path)
    readFile(abs_file_path)

    uxv_name = rospy.get_param('~uxv_name', 'ugv_1')
    ugv_pub = rospy.Publisher('ugv_cmd', CarControls, queue_size=10)
    sample_msg_pub = rospy.Publisher('/karmasim_node/sample_message', SampleMessage, queue_size=10)
    rospy.Subscriber('/karmasim_node/uxv_states', UxvStates, uxv_states_callback)
    
    contest_parameters = rospy.get_param('/scenario')
    world_boundaries = contest_parameters["world_boundaries"]
    
    rospy.wait_for_service('/karmasim_node/vehicle_start')
    request = rospy.ServiceProxy('/karmasim_node/vehicle_start', VehicleStart)
    result = request(uxv_name)
    if uxv_name=='ugv_1':

        request = rospy.ServiceProxy('/karmasim_node/vehicle_stop', VehicleStop)
        result = request(uxv_name)
    if uxv_name=='ugv_2':

        request = rospy.ServiceProxy('/karmasim_node/vehicle_stop', VehicleStop)
        result = request(uxv_name)
    if uxv_name=='ugv_3':

        request = rospy.ServiceProxy('/karmasim_node/vehicle_stop', VehicleStop)
        result = request(uxv_name)
    """rospy.Subscriber('/karmasim_node/points_of_interest',PointsOfInterest,poi_callback)
    rospy.Subscriber('/karmasim_node/'+uxv_name+'/distance/dist_front_left',Range,check_distance_front_left)
    rospy.Subscriber('/karmasim_node/'+uxv_name+'/distance/dist_front_right',Range,check_distance_front_right)
    rospy.Subscriber('/karmasim_node/'+uxv_name+'/distance/dist_front_center',Range,check_distance_front_center)
    rospy.Subscriber('/karmasim_node/'+uxv_name+'/distance/dist_back_center',Range,check_distance_back_center)

    rospy.Subscriber('/karmasim_node/points_of_interest',PointsOfInterest,PointsOfInterest_callback)
    sub = rospy.Subscriber('/karmasim_node/'+str(uxv_name)+'/odom_local_ned', Odometry, get_rotation)
    ugv_pos_x = uxv_pose_.position.x
    ugv_pos_y =uxv_pose_.position.y

    #go_to_position([wp11, wp10, wp2, wp1],uxv_pose_)
    counter = 0




    car_cmd = CarControls()
    hospitals = []
    headquarters=[]
    patients=[]
    charge_stations=[]
    asset_count = contest_parameters['asset_count']
    cargo_unload_distance = contest_parameters['cargo_unload_distance']
    cargo_load_distance = contest_parameters['cargo_load_distance']
    charging_point_distance = contest_parameters['charging_point_distance']

    for i in range(int(asset_count)):
        if contest_parameters['asset_'+str(i)]['type']=='hospital':
            hospitals.append(contest_parameters['asset_'+str(i)])
        if contest_parameters['asset_'+str(i)]['type']=='headquarter':
            headquarters.append(contest_parameters['asset_'+str(i)])
        if contest_parameters['asset_'+str(i)]['type']=='patient':
            patients.append(contest_parameters['asset_'+str(i)])
        if contest_parameters['asset_'+str(i)]['type']=='charge_station':
            charge_stations.append(contest_parameters['asset_'+str(i)])
    for i in range(2):
        
        pois_list_patient= pois_list_patient
        rospy.sleep(2)

    concurrent_task_number= int(contest_parameters['concurrent_task_number'])
    patients_name_list=[]
    for i in range(concurrent_task_number):
        patients_name = pois_list_patient[i].name
        patients_name_list.append(patients_name)
    c = 0
    target_list_pos = []
    target_req_list=[]
    for i in range(len(patients)-1):
        if c==concurrent_task_number:
            c=0
        if patients_name_list[c]==patients[i]['name']:
            pois_list_patient.append(patients[i])

        c+=1

    max_fuel = int(contest_parameters['uav_fuel_capacity'])
    percentage_of_fuel=(uxv_fuel_/max_fuel)*100
    message = SampleMessage()
    target_list=[]
    for i in range(concurrent_task_number):
        target_req_list.append(pois_list_patient[i].requirement)
    
    target_req_list_first_copy = target_req_list[:]
    while 1 in target_req_list:
        target_req_list.remove(1)
    while 2 in target_req_list:
        target_req_list.remove(2)

    
    for i in range(len(target_req_list_first_copy)):
       
        if int(target_req_list_first_copy[i])==3:
            target_list.append(pois_list_patient[i])
    if len(target_list)==0:
        print('There is no patient to go')
        request = rospy.ServiceProxy('/karmasim_node/vehicle_stop', VehicleStop)
        result = request(uxv_name)
    else:
        takeoff(wp16)


        
        target_pos = target_list[int(uxv_name[-1])-1]
        target_req = target_req_list[int(uxv_name[-1])-1]
        
        for patient in patients:
            if patient['name']==target_pos.name:
                patient_pos_ =patient
        patient_pos = patient_pos_
        
        patient_pos['x'] = float(patient_pos_['x'])
        patient_pos['y'] = float(patient_pos_['y'])
        patient_pos['z'] = float(patient_pos_['z'])
        patient_pos['requirement'] = 5
        #patient_pos = {'name':'patient_13','requirement':5,'x':-75.14,'y':35.14,'z':0.5}#deneme amacli
        #patient_pos = {'name':'patient_7','requirement':5,'x':-130.14,'y':49.25,'z':0.5}#deneme amacli

        #turn_right(yaw)
        target_pos_list = get_rota(patient_pos)
        
        # go to patient
        for target_pos in target_pos_list:
            

            

            if target_pos==target_pos_list[0]:
                first_yaw = yaw
            else:
                first_yaw = yaw_instant


            while not rospy.is_shutdown():
                ugv_pos_y = uxv_pose_.position.y
                ugv_pos_x = uxv_pose_.position.x
                
                
                if abs(((diffenance(target_pos['x'],ugv_pos_x)**2)-(diffenance(target_pos['y'],ugv_pos_y)**2)))**0.5 > 12:
                    if (first_yaw < 0.25 and first_yaw > -0.25) or (first_yaw > pi*0.75 and first_yaw < pi*1.25):
                        condisition = diffenance(ugv_pos_x,target_pos['x'])
                        go_ahead(condisition)

                        # x ekseni dogrultusunda git
                    if (first_yaw > (+pi/2)*0.75 and first_yaw < (+pi/2)*1.25) or (first_yaw < (-pi/2)*0.75 and first_yaw > (-pi/2)*1.25):
                        condisition = diffenance(ugv_pos_y,target_pos['y'])
                        go_ahead(condisition)

                        # y ekseni dogrultusunda git

                    #go to this position until reach

                else:
                    if target_pos == target_pos_list[len(target_pos_list)-1]:
                        car_cmd.brake = 0
                        target_throttle = 0
                        if target_throttle > 0:
                            car_cmd.manual = False
                            if car_cmd.manual_gear != 1:
                                car_cmd.manual_gear = 1
                        car_cmd.throttle = target_throttle
                        ugv_pub.publish(car_cmd)
                        break
                    else:
                        if first_yaw < (-pi/2)*0.75 and first_yaw > (-pi/2)*1.25:
                            target_pos['x']=int(float(target_pos['x']))
                            target_pos_list[target_pos_list.index(target_pos)+1]['x']=int(float(target_pos_list[target_pos_list.index(target_pos)+1]['x']))
                            if target_pos['x']>target_pos_list[target_pos_list.index(target_pos)+1]['x']:

                                turn_left(first_yaw)
                                get_yaw(yaw_instant)
                                break
                            elif target_pos['x']<target_pos_list[target_pos_list.index(target_pos)+1]['x']:

                                turn_right(first_yaw)
                                get_yaw(yaw_instant)
                                break
                            if int(target_pos['x'])==int(target_pos_list[target_pos_list.index(target_pos)+1]['x']):
                                go_ahead(dist_cal(target_pos['x'],target_pos['y'],target_pos_list[target_pos_list.index(target_pos)+1]['x'],target_pos_list[target_pos_list.index(target_pos)+1]['y']))
                                break
                        if first_yaw > (+pi/2)*0.75 and first_yaw < (+pi/2)*1.25:
                            target_pos['x']=int(float(target_pos['x']))
                            target_pos_list[target_pos_list.index(target_pos)+1]['x']=int(float(target_pos_list[target_pos_list.index(target_pos)+1]['x']))
                            if target_pos['x']>target_pos_list[target_pos_list.index(target_pos)+1]['x']:

                                turn_left(first_yaw)
                                get_yaw(yaw_instant)
                                break
                            elif target_pos['x']<target_pos_list[target_pos_list.index(target_pos)+1]['x']:

                                turn_right(first_yaw)
                                get_yaw(yaw_instant)
                                break
                            if int(target_pos['x'])==int(target_pos_list[target_pos_list.index(target_pos)+1]['x']):
                                go_ahead(dist_cal(target_pos['x'],target_pos['y'],target_pos_list[target_pos_list.index(target_pos)+1]['x'],target_pos_list[target_pos_list.index(target_pos)+1]['y']))
                                break
                        if first_yaw > pi*0.75 and first_yaw < pi*1.25:
                            target_pos['y']=int(float(target_pos['y']))
                            target_pos_list[target_pos_list.index(target_pos)+1]['y']=int(float(target_pos_list[target_pos_list.index(target_pos)+1]['y']))

                            if target_pos['y']<target_pos_list[target_pos_list.index(target_pos)+1]['y']:

                                turn_left(first_yaw)
                                get_yaw(yaw_instant)
                                break
                            elif target_pos['y']>target_pos_list[target_pos_list.index(target_pos)+1]['y']:

                                turn_right(first_yaw)
                                get_yaw(yaw_instant)
                                break
                            if int(target_pos['y'])==int(target_pos_list[target_pos_list.index(target_pos)+1]['y']):
                                go_ahead(dist_cal(target_pos['x'],target_pos['y'],target_pos_list[target_pos_list.index(target_pos)+1]['x'],target_pos_list[target_pos_list.index(target_pos)+1]['y']))
                                break
                        if first_yaw > -pi*0.75 and first_yaw < -pi*1.25:
                            target_pos['y']=int(float(target_pos['y']))
                            target_pos_list[target_pos_list.index(target_pos)+1]['y']=int(float(target_pos_list[target_pos_list.index(target_pos)+1]['y']))
                            if target_pos['y']<target_pos_list[target_pos_list.index(target_pos)+1]['y']:

                                turn_left(first_yaw)
                                get_yaw(yaw_instant)
                                break
                            elif target_pos['y']>target_pos_list[target_pos_list.index(target_pos)+1]['y']:

                                turn_right(first_yaw)
                                get_yaw(yaw_instant)
                                break
                            if int(target_pos['y'])==int(target_pos_list[target_pos_list.index(target_pos)+1]['y']):
                                go_ahead(dist_cal(target_pos['x'],target_pos['y'],target_pos_list[target_pos_list.index(target_pos)+1]['x'],target_pos_list[target_pos_list.index(target_pos)+1]['y']))
                                break
                        if first_yaw < 0.25 and first_yaw > -0.25 :
                            target_pos['y']=int(float(target_pos['y']))
                            target_pos_list[target_pos_list.index(target_pos)+1]['y']=int(float(target_pos_list[target_pos_list.index(target_pos)+1]['y']))
                            if int(target_pos['y'])<int(target_pos_list[target_pos_list.index(target_pos)+1]['y']):

                                turn_right(first_yaw)
                                get_yaw(yaw_instant)
                                break
                            elif target_pos['y']>target_pos_list[target_pos_list.index(target_pos)+1]['y']:

                                turn_left(first_yaw)
                                get_yaw(yaw_instant)
                                break
                            if int(target_pos['y'])==int(target_pos_list[target_pos_list.index(target_pos)+1]['y']):

                                go_ahead(dist_cal(target_pos['x'],target_pos['y'],target_pos_list[target_pos_list.index(target_pos)+1]['x'],target_pos_list[target_pos_list.index(target_pos)+1]['y']))
                                break

            counter += 1

        #go to patient pos

        car_cmd.brake = 0
        car_cmd.throttle = 0
        car_cmd.steering = 0
        ugv_pub.publish(car_cmd)
        target_wp_pos = target_pos_list[-1]
        first_yaw2 = yaw_instant

        while not rospy.is_shutdown():
            ugv_pos_x = uxv_pose_.position.x
            ugv_pos_y =uxv_pose_.position.y
            
            condisition_for_patient_move = dist_cal(float(patient_pos['x']),float(patient_pos['y']), ugv_pos_x, ugv_pos_y)
            
            if condisition_for_patient_move <8:
                break
            if first_yaw2 < 0.25 and first_yaw2 > -0.25:
                order = find_yaw_target_for_zero(target_wp_pos,patient_pos)
                if order == 1:
                    turn_left(first_yaw2)
                    go_ahead(condisition_for_patient_move)
                if order ==2:
                    turn_right(first_yaw2)
                    go_ahead(condisition_for_patient_move)
                if order ==3:
                    go_ahead(condisition_for_patient_move)
                if order==4:
                    go_back(condisition_for_patient_move)
            if first_yaw2 < (-pi/2)*0.75 and first_yaw2 > (-pi/2)*1.25:
                order = find_yaw_target_for_minus_pi_divided_2(target_wp_pos,patient_pos)
                if order == 1:
                    turn_left(first_yaw2)
                    go_ahead(condisition_for_patient_move)
                if order ==2:
                    turn_right(first_yaw2)
                    go_ahead(condisition_for_patient_move)
                if order ==3:
                    go_ahead(condisition_for_patient_move)
                if order==4:
                    go_back(condisition_for_patient_move)
            if first_yaw2 > (+pi/2)*0.75 and first_yaw2 < (+pi/2)*1.25:
                order = find_yaw_target_for_pi_divided_2(target_wp_pos,patient_pos)
                if order == 1:
                    turn_left(first_yaw2)
                    go_ahead(condisition_for_patient_move)
                if order ==2:
                    turn_right(first_yaw2)
                    go_ahead(condisition_for_patient_move)
                if order ==3:
                    go_ahead(condisition_for_patient_move)
                if order==4:
                    go_back(condisition_for_patient_move)
            if first_yaw2 > pi*0.75 and first_yaw2 < pi*1.25:
                order = find_yaw_target_for_pi(target_wp_pos,patient_pos)
                if order == 1:
                    turn_left(first_yaw2)
                    go_ahead(condisition_for_patient_move)
                if order == 2:
                    turn_right(first_yaw2)
                    go_ahead(condisition_for_patient_move)
                if order == 3:
                    go_ahead(condisition_for_patient_move)
                if order == 4:
                    go_back(condisition_for_patient_move)
        car_cmd.brake = 0
        target_throttle = 0
        car_cmd.throttle = target_throttle
        car_cmd.steering = 0
        ugv_pub.publish(car_cmd)

        hospital_pos = find_hospital(patient_pos,hospitals)

        if dist_cal(patient_pos['x'],patient_pos['y'],ugv_pos_x, ugv_pos_y) <int(cargo_load_distance):
            load(patient_pos,hospital_pos)


        target_pos_list_for_hospital = get_rota_for_hospitals(hospital_pos,patient_pos)
        
        if len(target_pos_list_for_hospital)==2:

            if target_pos_list_for_hospital[0]['x']==target_pos_list_for_hospital[1]['x'] or target_pos_list_for_hospital[0]['y']==target_pos_list_for_hospital[1]['y']:
                target_pos_list_for_hospital.pop()

        #go to hospital
        for target_pos in target_pos_list_for_hospital:
            
            



            if target_pos==target_pos_list_for_hospital[0]:
                first_yaw = yaw
            else:
                first_yaw = yaw


            while not rospy.is_shutdown():
                ugv_pos_y = uxv_pose_.position.y
                ugv_pos_x = uxv_pose_.position.x
                
                
                if abs(((diffenance(target_pos['x'],ugv_pos_x)**2)-(diffenance(target_pos['y'],ugv_pos_y)**2)))**0.5>12:

                    if (first_yaw < 0.25 and first_yaw > -0.25) or (first_yaw > pi*0.75 and first_yaw < pi*1.25):
                        condisition = diffenance(ugv_pos_x,target_pos['x'])
                        go_ahead(condisition)

                        # x ekseni dogrultusunda git
                    if (first_yaw > (+pi/2)*0.75 and first_yaw < (+pi/2)*1.25) or (first_yaw < (-pi/2)*0.75 and first_yaw > (-pi/2)*1.25):
                        condisition = diffenance(ugv_pos_y,target_pos['y'])
                        go_ahead(condisition)

                        # y ekseni dogrultusunda git

                    #go to this position until reach

                else:
                    if target_pos == target_pos_list_for_hospital[len(target_pos_list_for_hospital)-1]:
                        car_cmd.brake = 0
                        target_throttle = 0
                        if target_throttle > 0:
                            car_cmd.manual = False
                            if car_cmd.manual_gear != 1:
                                car_cmd.manual_gear = 1
                        car_cmd.throttle = target_throttle
                        ugv_pub.publish(car_cmd)
                        break
                    else:
                        if first_yaw < (-pi/2)*0.75 and first_yaw > (-pi/2)*1.25:
                            target_pos['x']=int(float(target_pos['x']))
                            target_pos_list_for_hospital[target_pos_list_for_hospital.index(target_pos)+1]['x'] = int(float(target_pos_list_for_hospital[target_pos_list_for_hospital.index(target_pos)+1]['x']))
                            if target_pos['x']>target_pos_list_for_hospital[target_pos_list_for_hospital.index(target_pos)+1]['x']:

                                turn_left(first_yaw)
                                get_yaw(yaw_instant)
                                break
                            elif target_pos['x']<target_pos_list_for_hospital[target_pos_list_for_hospital.index(target_pos)+1]['x']:

                                turn_right(first_yaw)
                                get_yaw(yaw_instant)
                                break
                            elif int(target_pos['x'])==int(target_pos_list_for_hospital[target_pos_list_for_hospital.index(target_pos)+1]['x']):
                                go_ahead(dist_cal(target_pos['x'],target_pos['y'],target_pos_list_for_hospital[target_pos_list_for_hospital.index(target_pos)+1]['x'],target_pos_list_for_hospital[target_pos_list_for_hospital.index(target_pos)+1]['y']))
                                break
                        if first_yaw > (+pi/2)*0.75 and first_yaw < (+pi/2)*1.25:
                            target_pos['x']=int(float(target_pos['x']))
                            target_pos_list_for_hospital[target_pos_list_for_hospital.index(target_pos)+1]['x'] = int(float(target_pos_list_for_hospital[target_pos_list_for_hospital.index(target_pos)+1]['x']))

                            if target_pos['x']>target_pos_list_for_hospital[target_pos_list_for_hospital.index(target_pos)+1]['x']:

                                turn_left(first_yaw)
                                get_yaw(yaw_instant)
                                break
                            elif target_pos['x']<target_pos_list_for_hospital[target_pos_list_for_hospital.index(target_pos)+1]['x']:

                                turn_right(first_yaw)
                                get_yaw(yaw_instant)
                                break
                            elif int(target_pos['x'])==int(target_pos_list_for_hospital[target_pos_list_for_hospital.index(target_pos)+1]['x']):
                                go_ahead(dist_cal(target_pos['x'],target_pos['y'],target_pos_list_for_hospital[target_pos_list_for_hospital.index(target_pos)+1]['x'],target_pos_list_for_hospital[target_pos_list_for_hospital.index(target_pos)+1]['y']))
                                break
                        if first_yaw > pi*0.75 and first_yaw < pi*1.25:
                            target_pos['y']=int(float(target_pos['y']))
                            target_pos_list_for_hospital[target_pos_list_for_hospital.index(target_pos)+1]['y'] = int(float(target_pos_list_for_hospital[target_pos_list_for_hospital.index(target_pos)+1]['y']))

                            if target_pos['y']<target_pos_list_for_hospital[target_pos_list_for_hospital.index(target_pos)+1]['y']:

                                turn_left(first_yaw)
                                get_yaw(yaw_instant)
                                break
                            elif target_pos['y']>target_pos_list_for_hospital[target_pos_list_for_hospital.index(target_pos)+1]['y']:

                                turn_right(first_yaw)
                                get_yaw(yaw_instant)
                                break
                            elif int(target_pos['y'])==int(target_pos_list_for_hospital[target_pos_list_for_hospital.index(target_pos)+1]['y']):
                                go_ahead(dist_cal(target_pos['x'],target_pos['y'],target_pos_list_for_hospital[target_pos_list_for_hospital.index(target_pos)+1]['x'],target_pos_list_for_hospital[target_pos_list_for_hospital.index(target_pos)+1]['y']))
                                break
                        if first_yaw > -pi*0.75 and first_yaw < -pi*1.25:
                            target_pos['y']=int(float(target_pos['y']))
                            target_pos_list_for_hospital[target_pos_list_for_hospital.index(target_pos)+1]['y'] = int(float(target_pos_list_for_hospital[target_pos_list_for_hospital.index(target_pos)+1]['y']))
                            if target_pos['y']<target_pos_list_for_hospital[target_pos_list_for_hospital.index(target_pos)+1]['y']:

                                turn_left(first_yaw)
                                get_yaw(yaw_instant)
                                break
                            elif target_pos['y']>target_pos_list_for_hospital[target_pos_list_for_hospital.index(target_pos)+1]['y']:

                                turn_right(first_yaw)
                                get_yaw(yaw_instant)
                                break
                            elif int(target_pos['y'])==int(target_pos_list_for_hospital[target_pos_list_for_hospital.index(target_pos)+1]['y']):
                                go_ahead(dist_cal(target_pos['x'],target_pos['y'],target_pos_list_for_hospital[target_pos_list_for_hospital.index(target_pos)+1]['x'],target_pos_list_for_hospital[target_pos_list_for_hospital.index(target_pos)+1]['y']))
                                break
                        if first_yaw < 0.25 and first_yaw > -0.25 :
                            target_pos['y']=int(float(target_pos['y']))
                            target_pos_list_for_hospital[target_pos_list_for_hospital.index(target_pos)+1]['y'] = int(float(target_pos_list_for_hospital[target_pos_list_for_hospital.index(target_pos)+1]['y']))
                            if target_pos['y']<target_pos_list_for_hospital[target_pos_list_for_hospital.index(target_pos)+1]['y']:

                                turn_right(first_yaw)
                                get_yaw(yaw_instant)
                                break
                            elif target_pos['y']>target_pos_list_for_hospital[target_pos_list_for_hospital.index(target_pos)+1]['y']:

                                turn_left(first_yaw)
                                get_yaw(yaw_instant)
                                break
                            elif int(target_pos['y'])==int(target_pos_list_for_hospital[target_pos_list_for_hospital.index(target_pos)+1]['y']):
                                go_ahead(dist_cal(target_pos['x'],target_pos['y'],target_pos_list_for_hospital[target_pos_list_for_hospital.index(target_pos)+1]['x'],target_pos_list_for_hospital[target_pos_list_for_hospital.index(target_pos)+1]['y']))
                                break
        # go to hospital point
        while not rospy.is_shutdown():

            ugv_pos_x = uxv_pose_.position.x
            ugv_pos_y =uxv_pose_.position.y
            
            condisition_for_hospital_move = dist_cal(float(hospital_pos['x']),float(hospital_pos['y']), ugv_pos_x, ugv_pos_y)
            
            if condisition_for_hospital_move <8:
                break
            if first_yaw2 < 0.25 and first_yaw2 > -0.25:
                order = find_yaw_target_for_zero(target_wp_pos,hospital_pos)
                if order == 1:
                    turn_left(first_yaw2)
                    go_ahead(condisition_for_hospital_move)
                if order ==2:
                    turn_right(first_yaw2)
                    go_ahead(condisition_for_hospital_move)
                if order ==3:
                    go_ahead(condisition_for_hospital_move)
                if order==4:
                    go_back(condisition_for_hospital_move)
            if first_yaw2 < (-pi/2)*0.75 and first_yaw2 > (-pi/2)*1.25:
                order = find_yaw_target_for_minus_pi_divided_2(target_wp_pos,hospital_pos)
                if order == 1:
                    turn_left(first_yaw2)
                    go_ahead(condisition_for_hospital_move)
                if order ==2:
                    turn_right(first_yaw2)
                    go_ahead(condisition_for_hospital_move)
                if order ==3:
                    go_ahead(condisition_for_hospital_move)
                if order==4:
                    go_back(condisition_for_hospital_move)
            if first_yaw2 > (+pi/2)*0.75 and first_yaw2 < (+pi/2)*1.25:
                order = find_yaw_target_for_pi_divided_2(target_wp_pos,hospital_pos)
                if order == 1:
                    turn_left(first_yaw2)
                    go_ahead(condisition_for_hospital_move)
                if order ==2:
                    turn_right(first_yaw2)
                    go_ahead(condisition_for_hospital_move)
                if order ==3:
                    go_ahead(condisition_for_hospital_move)
                if order==4:
                    go_back(condisition_for_hospital_move)
            if first_yaw2 > pi*0.75 and first_yaw2 < pi*1.25:
                order = find_yaw_target_for_pi(target_wp_pos,hospital_pos)
                if order == 1:
                    turn_left(first_yaw2)
                    go_ahead(condisition_for_hospital_move)
                if order == 2:
                    turn_right(first_yaw2)
                    go_ahead(condisition_for_hospital_move)
                if order == 3:
                    go_ahead(condisition_for_hospital_move)
                if order == 4:
                    go_back(condisition_for_hospital_move)
        car_cmd.brake = 0
        target_throttle = 0
        car_cmd.throttle = target_throttle
        car_cmd.steering = 0
        ugv_pub.publish(car_cmd)
        if dist_cal(float(hospital_pos['x']),float(hospital_pos['y']),ugv_pos_x, ugv_pos_y) <cargo_unload_distance:
            unload(patient_pos,hospital_pos)










        request = rospy.ServiceProxy('/karmasim_node/vehicle_stop', VehicleStop)
        result = request(uxv_name)"""
