#!/usr/bin/env python
from karmasim_ros_wrapper.msg import UxvStates,Uxv,PointsOfInterest,PointOfInterest
import os
import rospy
from karmasim_dev_pkg.msg import SampleMessage
from karmasim_ros_wrapper.msg import VelCmd, UxvStates, Uxv
import tty,sys
from karmasim_ros_wrapper.srv import VehicleStart, VehicleStop, VehicleUnload, VehicleLoad
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Pose, Quaternion
import math

from sensor_msgs.msg import Range
uxv_name = ""
uxv_pose_ = Pose()
def uxv_callback(dataa):
    global uxv_fuel_
    uxv_fuel_ =dataa.fuel_current

uxv_fuel_ = Uxv.fuel_current
uxv_unavailable = Uxv.is_unavailable
uxv_cargo = Uxv.cargo_weight
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
            global uxv_cargo
            uxv_cargo = uxv.cargo_weight

def vels(target_lin_z_speed, target_lin_x_speed, target_lin_y_speed, target_ang_z_speed):
    return 'currently:\t linear_z %s\t linear_x %s\t linear_y %s\t angular_z %s ' % (
        target_lin_z_speed, target_lin_x_speed, target_lin_y_speed, target_ang_z_speed)
roll =0
pitch = 0
yaw=0

def find_hospital(patient_pos,hospitals):
    hospital_pos_list = hospitals
    swap = 10000
    for hospital_pos in hospital_pos_list:
        distance =dist_cal(patient_pos['x'],patient_pos['y'],float(hospital_pos['x']),float(hospital_pos['y']))

        if swap > abs(distance):
            swap =distance
            hospital_pos_ = hospital_pos
    return hospital_pos_

def check_distance_bottom(msg):
    global dist_bottom

    dist_bottom = msg

def check_distance_front(msg2):
    global dist_front

    dist_front = msg2

def check_distance_left(msg3):
    global dist_left

    dist_left = msg3

def check_distance_rear(msg4):
    global dist_rear

    dist_rear = msg4

def check_distance_right(msg5):
    global dist_right

    dist_right = msg5

def check_distance_top(msg6):
    global dist_top

    dist_top = msg6

def collision_avoidance(uav_pub):
    
    target_lin_x_speed = 0.0
    target_lin_y_speed = 0.0
    target_lin_z_speed = 0.0
    target_ang_x_speed = 0.0
    target_ang_y_speed = 0.0
    target_ang_z_speed = 0.0

    drone_cmd = VelCmd()
    drone_cmd.twist.linear.x = 0
    drone_cmd.twist.linear.y = 0
    drone_cmd.twist.linear.z = 0
    drone_cmd.twist.angular.x = 0
    drone_cmd.twist.angular.y = 0
    drone_cmd.twist.angular.z = 0

    uav_pos_z = uxv_pose_.position.z
    while float(dist_bottom.range) < float(4) or float(dist_top.range) < float(4) or uav_pos_z < (float(world_boundaries['zmax']) + float(5)) or float(dist_front.range) < float(4) or float(dist_left.range) < float(4) or float(dist_right.range) < float(4) or float(dist_rear.range) < float(4) :
        while float(dist_bottom.range) < float(4):

            
            
            if float(dist_bottom.range) < float(3):
                
               
                target_lin_z_speed = 5
                
            elif float(dist_bottom.range) > float(8):

                break

            target_lin_x_speed = 0
            target_lin_y_speed = 0
            drone_cmd.twist.linear.x = target_lin_x_speed
            drone_cmd.twist.linear.y = target_lin_y_speed
            drone_cmd.twist.linear.z = -target_lin_z_speed

            drone_cmd.twist.angular.x = target_ang_x_speed
            drone_cmd.twist.angular.y = target_ang_y_speed
            drone_cmd.twist.angular.z = target_ang_z_speed
            uav_pub.publish(drone_cmd)

        while float(dist_top.range) < float(4) or uav_pos_z < (float(world_boundaries['zmax']) + float(5)):
            
            
            if float(dist_top.range) < float(4):
                
                
                target_lin_z_speed = -5
                
            elif float(dist_top.range) > float(10):

                break
            target_lin_x_speed = 0
            target_lin_y_speed = 0
            drone_cmd.twist.linear.x = target_lin_x_speed
            drone_cmd.twist.linear.y = target_lin_y_speed
            drone_cmd.twist.linear.z = -target_lin_z_speed

            drone_cmd.twist.angular.x = target_ang_x_speed
            drone_cmd.twist.angular.y = target_ang_y_speed
            drone_cmd.twist.angular.z = target_ang_z_speed
            uav_pub.publish(drone_cmd)

        while float(dist_front.range) < float(4):
            
            
            if float(dist_front.range) < float(4) :
                
                
                target_lin_y_speed = 0
                target_lin_z_speed = 5
               


            elif float(dist_front.range) > float(8):

                break
            target_lin_x_speed = 0

            drone_cmd.twist.linear.x = target_lin_x_speed
            drone_cmd.twist.linear.y = target_lin_y_speed
            drone_cmd.twist.linear.z = -target_lin_z_speed

            drone_cmd.twist.angular.x = target_ang_x_speed
            drone_cmd.twist.angular.y = target_ang_y_speed
            drone_cmd.twist.angular.z = target_ang_z_speed
            uav_pub.publish(drone_cmd)

        while float(dist_left.range) < float(4):
    
            
            if float(dist_left.range) < float(4):
                
                
                target_lin_x_speed = 0
                target_lin_z_speed = 5
                
                


            elif float(dist_left.range) > float(8):

                break

            target_lin_y_speed = 0

            drone_cmd.twist.linear.x = target_lin_x_speed
            drone_cmd.twist.linear.y = target_lin_y_speed
            drone_cmd.twist.linear.z = -target_lin_z_speed

            drone_cmd.twist.angular.x = target_ang_x_speed
            drone_cmd.twist.angular.y = target_ang_y_speed
            drone_cmd.twist.angular.z = target_ang_z_speed
            uav_pub.publish(drone_cmd)

        while float(dist_right.range) < float(4):
            
           
            if float(dist_right.range) < float(4):
                
                
                target_lin_x_speed = 0
                target_lin_z_speed = 5
                



            elif float(dist_right.range) > float(8):

                break

            target_lin_y_speed = 0

            drone_cmd.twist.linear.x = target_lin_x_speed
            drone_cmd.twist.linear.y = target_lin_y_speed
            drone_cmd.twist.linear.z = -target_lin_z_speed

            drone_cmd.twist.angular.x = target_ang_x_speed
            drone_cmd.twist.angular.y = target_ang_y_speed
            drone_cmd.twist.angular.z = target_ang_z_speed
            uav_pub.publish(drone_cmd)

        while float(dist_rear.range) < float(4):
           
            
            if float(dist_rear.range) < float(4):
                
                
                target_lin_y_speed = 0
                target_lin_z_speed = 5
                



            elif float(dist_rear.range) > float(8):

                break
            target_lin_x_speed = 0

            drone_cmd.twist.linear.x = target_lin_x_speed
            drone_cmd.twist.linear.y = target_lin_y_speed
            drone_cmd.twist.linear.z = -target_lin_z_speed

            drone_cmd.twist.angular.x = target_ang_x_speed
            drone_cmd.twist.angular.y = target_ang_y_speed
            drone_cmd.twist.angular.z = target_ang_z_speed
            uav_pub.publish(drone_cmd)
        while not rospy.is_shutdown():
            
            
            target_lin_z_speed = 0
            target_lin_x_speed = 0
            target_lin_y_speed = 0
            

            drone_cmd.twist.linear.x = target_lin_x_speed
            drone_cmd.twist.linear.y = target_lin_y_speed
            drone_cmd.twist.linear.z = -target_lin_z_speed
            uav_pub.publish(drone_cmd)
            break
def get_rotation(msg):
    global roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
def pose(uav_pose):
    
    return uav_pose.position
def takeoff(uxv_name,uav_pub):
    
    
    
    uav_pos_z = uxv_pose_.position.z
    target_lin_x_speed = 0.0
    target_lin_y_speed = 0.0
    target_lin_z_speed = 0.0
    target_ang_x_speed = 0.0
    target_ang_y_speed = 0.0
    target_ang_z_speed = 0.0

    drone_cmd = VelCmd()
    drone_cmd.twist.linear.x = 0
    drone_cmd.twist.linear.y = 0
    drone_cmd.twist.linear.z = 0
    drone_cmd.twist.angular.x = 0
    drone_cmd.twist.angular.y = 0
    drone_cmd.twist.angular.z = 0
    

    while not rospy.is_shutdown():
       
        
        uav_pos_x = uxv_pose_.position.x
        uav_pos_y = uxv_pose_.position.y
        uav_pos_z = uxv_pose_.position.z
       

        height = float(-30-(int(uxv_name[4])*3))

        if uav_pos_z > height:

            
           
            target_lin_z_speed = 5
            
        else:
            if target_lin_z_speed == 0:
                break
            else:
                
                
                target_lin_z_speed = 0
                

        drone_cmd.twist.linear.x = target_lin_x_speed
        drone_cmd.twist.linear.y = target_lin_y_speed
        drone_cmd.twist.linear.z = -target_lin_z_speed

        drone_cmd.twist.angular.x = target_ang_x_speed
        drone_cmd.twist.angular.y = target_ang_y_speed
        drone_cmd.twist.angular.z = target_ang_z_speed
        uav_pub.publish(drone_cmd)
def dist_cal(x1=0.00,y1=0.00,x2=0.00,y2=0.00):
    return math.sqrt(((abs(x1-x2)**2)+(abs(y1-y2)**2)))
def readFile(filename):
    filehandle = open(filename)
    
    filehandle.close()
def find_speed_x(target_pos):
    uav_pos_x = uxv_pose_.position.x
    target_pos_x = float(target_pos['x'])
    
    if uav_pos_x<=0 and target_pos_x<=0:
        if uav_pos_x>target_pos_x:
            return -5
        if uav_pos_x<target_pos_x:
            return 5
    if uav_pos_x<=0 and target_pos_x>=0:
        if target_pos_x>uav_pos_x:
            return 5
    if uav_pos_x>=0 and target_pos_x>=0:
        if uav_pos_x>target_pos_x:
            return -5
        if uav_pos_x<target_pos_x:
            return 5
    if uav_pos_x>=0 and target_pos_x<=0:
        if uav_pos_x>target_pos_x:
            return -5
def find_speed_y(target_pos):
    uav_pos_y = uxv_pose_.position.y
    target_pos_y = float(target_pos['y'])
    
    if target_pos_y<=0 and uav_pos_y<=0:
        if target_pos_y<uav_pos_y:
            return -5
        if target_pos_y>uav_pos_y:
            return 5
    if target_pos_y<=0 and uav_pos_y>=0:
        if target_pos_y<uav_pos_y:
            return -5
    if target_pos_y>=0 and uav_pos_y>=0:
        if target_pos_y<uav_pos_y:
            return -5
        if target_pos_y>uav_pos_y:
            return 5
    if target_pos_y>=0 and uav_pos_y<=0:
        if target_pos_y>uav_pos_y:
            return 5


def go_to_position(target_yaw,uav_pub,target_pos):
    
       

    target_lin_x_speed = 0.0
    target_lin_y_speed = 0.0
    target_lin_z_speed = 0.0
    target_ang_x_speed = 0.0
    target_ang_y_speed = 0.0
    target_ang_z_speed = 0.0

    drone_cmd = VelCmd()
    drone_cmd.twist.linear.x = 0
    drone_cmd.twist.linear.y = 0
    drone_cmd.twist.linear.z = 0
    drone_cmd.twist.angular.x = 0
    drone_cmd.twist.angular.y = 0
    drone_cmd.twist.angular.z = 0
    while not rospy.is_shutdown():
        uav_pos_x = uxv_pose_.position.x
        uav_pos_z = uxv_pose_.position.z
        
        if float(dist_bottom.range) < float(4) or float(dist_top.range) < float(4) or uav_pos_z < (float(world_boundaries['zmax']) + float(5)) or float(dist_front.range) < float(4) or float(dist_left.range) < float(4) or float(dist_right.range) < float(4) or float(dist_rear.range) < float(4) :
            collision_avoidance(uav_pub)
        if diffenance(target_pos['x'],uav_pos_x)>1:
            order = find_speed_x(target_pos)
            target_lin_x_speed = order
            
            
            #print(pose(uxv_pose_))
            
        else:
            if target_lin_x_speed == 0:
                break
            else:
                
                
                target_lin_x_speed = 0
                #print(pose(uxv_pose_))
                

        drone_cmd.twist.linear.x = target_lin_x_speed
        drone_cmd.twist.linear.y = target_lin_y_speed
        drone_cmd.twist.linear.z = -target_lin_z_speed

        drone_cmd.twist.angular.x = target_ang_x_speed
        drone_cmd.twist.angular.y = target_ang_y_speed
        drone_cmd.twist.angular.z = target_ang_z_speed
        uav_pub.publish(drone_cmd)
    while not rospy.is_shutdown():
        uav_pos_z = uxv_pose_.position.z
        uav_pos_y = uxv_pose_.position.y
        
        
        if diffenance(target_pos['y'],uav_pos_y)>1:
            if float(dist_bottom.range) < float(4) or float(dist_top.range) < float(4) or uav_pos_z < (float(world_boundaries['zmax']) + float(5)) or float(dist_front.range) < float(4) or float(dist_left.range) < float(4) or float(dist_right.range) < float(4) or float(dist_rear.range) < float(4) :
                collision_avoidance(uav_pub)
            ordery = find_speed_y(target_pos)
            target_lin_y_speed = ordery
            
            


            #print(pose(uxv_pose_))
            
        else:
            if target_lin_y_speed == 0:
                break
            else:
                
                
                target_lin_y_speed = 0
                

        drone_cmd.twist.linear.x = target_lin_x_speed
        drone_cmd.twist.linear.y = target_lin_y_speed
        drone_cmd.twist.linear.z = -target_lin_z_speed

        drone_cmd.twist.angular.x = target_ang_x_speed
        drone_cmd.twist.angular.y = target_ang_y_speed
        drone_cmd.twist.angular.z = target_ang_z_speed
        uav_pub.publish(drone_cmd)
    while not rospy.is_shutdown():
        uav_pos_z = uxv_pose_.position.z
        
        
        if diffenance(target_pos['z'],uav_pos_z)>3:
            
            target_lin_z_speed = -5
            
            

            #print(pose(uxv_pose_))
            #print(vels(target_lin_z_speed, target_lin_x_speed, target_lin_y_speed, target_ang_z_speed))
        else:
            if target_lin_z_speed == 0:
                break
            else:
                
                
                target_lin_z_speed = 0
                #print(pose(uxv_pose_))
                #print(vels(target_lin_z_speed, target_lin_x_speed, target_lin_y_speed, target_ang_z_speed))

        drone_cmd.twist.linear.x = target_lin_x_speed
        drone_cmd.twist.linear.y = target_lin_y_speed
        drone_cmd.twist.linear.z = -target_lin_z_speed

        drone_cmd.twist.angular.x = target_ang_x_speed
        drone_cmd.twist.angular.y = target_ang_y_speed
        drone_cmd.twist.angular.z = target_ang_z_speed
        uav_pub.publish(drone_cmd)
    #turnnnn
    """
    while not rospy.is_shutdown():
        tty.setraw(sys.stdin.fileno())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print(yaw_instant,target_yaw)
        if round(yaw_instant,2)!=round(target_yaw,2):

            tty.setraw(sys.stdin.fileno())
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            target_ang_z_speed = 0.4
            print(pose(uxv_pose_))
            print(vels(target_lin_z_speed, target_lin_x_speed, target_lin_y_speed, target_ang_z_speed))
        else:
            if target_ang_z_speed == 0:
                break
            else:
                tty.setraw(sys.stdin.fileno())
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
                target_ang_z_speed = 0
                print(pose(uxv_pose_))
                print(vels(target_lin_z_speed, target_lin_x_speed, target_lin_y_speed, target_ang_z_speed))

        drone_cmd.twist.linear.x = target_lin_x_speed
        drone_cmd.twist.linear.y = target_lin_y_speed
        drone_cmd.twist.linear.z = -target_lin_z_speed

        drone_cmd.twist.angular.x = target_ang_x_speed
        drone_cmd.twist.angular.y = target_ang_y_speed
        drone_cmd.twist.angular.z = target_ang_z_speed
        uav_pub.publish(drone_cmd)
    
    # goooo
    while not rospy.is_shutdown():
        uav_pos_x = uxv_pose_.position.x
        uav_pos_y = uxv_pose_.position.y
        uav_pos_z = uxv_pose_.position.z
        tty.setraw(sys.stdin.fileno())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        if dist_cal(uav_pos_x,uav_pos_y,target_pos['x'],target_pos['y'])>5:

            tty.setraw(sys.stdin.fileno())
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            target_lin_y_speed = 5
            print(pose(uxv_pose_))
            print(vels(target_lin_z_speed, target_lin_x_speed, target_lin_y_speed, target_ang_z_speed))
        else:
            if target_lin_y_speed == 0:
                break
            else:
                tty.setraw(sys.stdin.fileno())
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
                target_lin_y_speed = 0
                print(pose(uxv_pose_))
                print(vels(target_lin_z_speed, target_lin_x_speed, target_lin_y_speed, target_ang_z_speed))

        drone_cmd.twist.linear.x = target_lin_x_speed
        drone_cmd.twist.linear.y = target_lin_y_speed
        drone_cmd.twist.linear.z = -target_lin_z_speed

        drone_cmd.twist.angular.x = target_ang_x_speed
        drone_cmd.twist.angular.y = target_ang_y_speed
        drone_cmd.twist.angular.z = target_ang_z_speed
        uav_pub.publish(drone_cmd)"""

pois_list_ = []
pois_pose = Pose()
pois_req = PointOfInterest.requirement
pois_name = PointOfInterest.name
def poi_callback(data):
    global pois_list_

    for poisof in data.pois:
        if poisof.name == 'patient_17':
            global pois_req
            pois_req = poisof.requirement
def diffenance(a,b):
    return abs(float(a)-float(b))
def find_yaw(target_pos):
    target_pos_x = target_pos['x']
    target_pos_y = target_pos['y']
    uav_staring_point_x = uxv_pose_.position.x
    uav_staring_point_y = uxv_pose_.position.y

    hipo = dist_cal(target_pos_x,target_pos_y,uav_staring_point_x,uav_staring_point_y)
    y_dist = diffenance(target_pos_y,uav_staring_point_y)
    sin_hip = math.sin(math.radians(90))
    x = (sin_hip/hipo)*y_dist

    sin_y = math.asin(x)
    return sin_y
def vehicle_load(condition,uxv_name,hospital_name):
    if condition == 1:

        amount_number_for_load = 10
        cargo_type_for_load = 1
        source_poi_for_load = hospital_name
        rospy.wait_for_service('/karmasim_node/vehicle_load')
        req = rospy.ServiceProxy('/karmasim_node/vehicle_load', VehicleLoad)
        res = req(uxv_name, source_poi_for_load, cargo_type_for_load, amount_number_for_load)
        print('load result: %s\t message: %s ' % (res.success, res.message))
    elif condition == 2:

        amount_number_for_load = 10
        cargo_type_for_load = 2
        source_poi_for_load = 'headquarter_1'
        rospy.wait_for_service('/karmasim_node/vehicle_load')
        req = rospy.ServiceProxy('/karmasim_node/vehicle_load', VehicleLoad)
        res = req(uxv_name, source_poi_for_load, cargo_type_for_load, amount_number_for_load)
        print('load result: %s\t message: %s ' % (res.success, res.message))
def vehicle_unload(target_req,target_name,uxv_name):
    if target_req == 1:
        
        cargo_type = 1
        target_poi = str(target_name)

        amount_number = 1
        rospy.wait_for_service('/karmasim_node/vehicle_unload')

        req = rospy.ServiceProxy('/karmasim_node/vehicle_unload', VehicleUnload)

        res = req(uxv_name, target_poi, cargo_type, amount_number)
        print('unload result: %s\t message: %s ' % (res.success, res.message))

    elif target_req == 2:
        
        cargo_type = 2
        target_poi = target_name
        amount_number = 1
        rospy.wait_for_service('/karmasim_node/vehicle_unload')

        req = rospy.ServiceProxy('/karmasim_node/vehicle_unload', VehicleUnload)

        res = req(uxv_name, target_poi, cargo_type, amount_number)
        print('unload result: %s\t message: %s ' % (res.success, res.message))

def find_headquarter(patient_pos,headquarters):
    headquarter_pos_list=headquarters
    swap = 10000
    for headquarter_pos in headquarter_pos_list:
        distance =dist_cal(patient_pos['x'],patient_pos['y'],float(headquarter_pos['x']),float(headquarter_pos['y']))

        if swap > abs(distance):
            swap =distance
            headquarter_pos_ = headquarter_pos
    return headquarter_pos_

def find_charge_station(patient_pos,charge_stations):
    charge_station_pos_list=charge_stations
    swap = 10000
    for charge_station_pos in charge_station_pos_list:
        distance =dist_cal(patient_pos['x'],patient_pos['y'],float(charge_station_pos['x']),float(charge_station_pos['y']))

        if swap > abs(distance):
            swap =distance
            charge_station_pos_ = charge_station_pos
    return charge_station_pos

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

if __name__ == "__main__":
    rospy.init_node('contester_uav_node')

    #Relative path example
    script_dir = os.path.dirname(__file__)
    rel_path = '../res/ydc_formal.txt'
    abs_file_path = os.path.join(script_dir, rel_path)
    readFile(abs_file_path)
    uxv_name = rospy.get_param('~uxv_name', 'uav_1')
    uav_pub = rospy.Publisher('uav_cmd', VelCmd, queue_size=10)
    sample_msg_pub = rospy.Publisher('/karmasim_node/sample_message', SampleMessage, queue_size=10)
    
    contest_parameters = rospy.get_param('/scenario')
    world_boundaries = contest_parameters["world_boundaries"]
    
    hospitals = []
    headquarters=[]
    patients=[]
    charge_stations=[]
    asset_count = contest_parameters['asset_count']
    for i in range(int(asset_count)):
        if contest_parameters['asset_'+str(i)]['type']=='hospital':
            hospitals.append(contest_parameters['asset_'+str(i)])
        if contest_parameters['asset_'+str(i)]['type']=='headquarter':
            headquarters.append(contest_parameters['asset_'+str(i)])
        if contest_parameters['asset_'+str(i)]['type']=='patient':
            patients.append(contest_parameters['asset_'+str(i)])
        if contest_parameters['asset_'+str(i)]['type']=='charge_station':
            charge_stations.append(contest_parameters['asset_'+str(i)])


    rospy.Subscriber('/karmasim_node/uxv_states', UxvStates, uxv_states_callback)
    rospy.wait_for_service('/karmasim_node/vehicle_start')
    request = rospy.ServiceProxy('/karmasim_node/vehicle_start', VehicleStart)
    result = request(uxv_name)
    sub = rospy.Subscriber('/karmasim_node/'+str(uxv_name)+'/odom_local_ned', Odometry, get_rotation)
    rospy.Subscriber('/karmasim_node/'+uxv_name+'/distance/dist_bottom',Range,check_distance_bottom)
    rospy.Subscriber('/karmasim_node/'+uxv_name+'/distance/dist_front',Range,check_distance_front)
    rospy.Subscriber('/karmasim_node/'+uxv_name+'/distance/dist_left',Range,check_distance_left)
    rospy.Subscriber('/karmasim_node/'+uxv_name+'/distance/dist_rear',Range,check_distance_rear)
    rospy.Subscriber('/karmasim_node/'+uxv_name+'/distance/dist_right',Range,check_distance_right)
    rospy.Subscriber('/karmasim_node/'+uxv_name+'/distance/dist_top',Range,check_distance_top)
    rospy.Subscriber('/karmasim_node/points_of_interest',PointsOfInterest,PointsOfInterest_callback)
    for i in range(2):
        
        uxv_cargo = uxv_cargo
        rospy.sleep(2)
    for i in range(2):
        
        pois_list_patient= pois_list_patient
        rospy.sleep(2)

    concurrent_task_number= 10#int(contest_parameters['concurrent_task_number'])
    patients_name_list=[]
    print(concurrent_task_number,pois_list_patient)
    for i in range(concurrent_task_number):
        patients_name = pois_list_patient[i].name
        patients_name_list.append(patients_name)
    c = 0
    target_list_pos = []
    target_req_list=[]
    """for i in range(len(patients)-1):
        if c==concurrent_task_number:
            c=0
        if patients_name_list[c]==patients[i]['name']:
            pois_list_patient.append(patients[i])

        c+=1


    message = SampleMessage()
    target_list=[]
    
    for i in range(concurrent_task_number):
        target_req_list.append(pois_list_patient[i].requirement)
    target_req_list_first_copy = target_req_list[:]
    while 3 in target_req_list:
        target_req_list.remove(3)
    
    for i in range(len(target_req_list_first_copy)):
        
        if int(target_req_list_first_copy[i])==2:
            target_list.append(pois_list_patient[i])
    print(len(target_list),int(uxv_name[-1]))

    target_pos=[]
    for i in target_list:
        print(i)
        dist = int(dist_cal(i.pose.position.x,i.pose.position.y))
        swap = 2000
        if swap > dist and dist<100:
            swap = dist
            target_pos.append(i)
            print(dist)
            target_req = 2"""

    
    if uxv_name=='uav_1':

        target_pos = {'y': 21.25, 'x': 29.95, 'z': 0.5, 'type': 'patient', 'name': 'patient_10'}
    if uxv_name=='uav_2':
        target_pos = {'y': -94.24, 'x': -101.34, 'z': -8.01, 'type': 'patient', 'name': 'patient_5'}
        
    target_req = 2
    if uxv_name=='uav_3':

        request = rospy.ServiceProxy('/karmasim_node/vehicle_stop', VehicleStop)
        result = request(uxv_name)
    if uxv_name=='uav_4':

        request = rospy.ServiceProxy('/karmasim_node/vehicle_stop', VehicleStop)
        result = request(uxv_name)
    if uxv_name=='uav_5':

        request = rospy.ServiceProxy('/karmasim_node/vehicle_stop', VehicleStop)
        result = request(uxv_name)
    if uxv_name=='uav_6':

        request = rospy.ServiceProxy('/karmasim_node/vehicle_stop', VehicleStop)
        result = request(uxv_name)
    a=0
    """for patient in patients:
        if patient['name']==target_pos.name:
            target_pos_ =patient"""
    
    
    
    max_fuel = int(contest_parameters['uav_fuel_capacity'])
    percentage_of_fuel=(float(uxv_fuel_)/float(max_fuel))*100
    
    cargo_unload_distance = contest_parameters['cargo_unload_distance']
    cargo_load_distance = contest_parameters['cargo_load_distance']
    charging_point_distance = contest_parameters['charging_point_distance']
    #target_pos = {'x':101.34,'y':94.24,'z':-58.01,'requirement':2,'name':'patient_5'}
    print(target_pos,target_req)
    
    if int(target_req) == 1:#medicine ise
        if len(contest_parameters['uav_initial_cargo'])==0 or uxv_cargo==0:
            target_yaw=0
            takeoff(uxv_name,uav_pub)
            near_headquarter=find_headquarter(target_pos,headquarters)
            go_to_position(target_yaw,uav_pub,near_headquarter)
            vehicle_load(1,uxv_name,near_headquarter['name'])
            takeoff(uxv_name,uav_pub)
            go_to_position(target_yaw,uav_pub,target_pos)
            vehicle_unload(2,target_pos['name'],uxv_name)
            takeoff(uxv_name,uav_pub)
        else:
            target_yaw=0
            takeoff(uxv_name,uav_pub)
            go_to_position(target_yaw,uav_pub,target_pos)
            vehicle_unload(2,target_pos['name'],uxv_name)
            takeoff(uxv_name,uav_pub)

    if (target_req) == 2:#food ise
        if len(contest_parameters['uav_initial_cargo'])==0 or uxv_cargo==0:
            target_yaw=0
            takeoff(uxv_name,uav_pub)
            
            near_hospital=find_hospital(target_pos,hospitals)
            go_to_position(target_yaw,uav_pub,near_hospital)
            vehicle_load(1,uxv_name,near_hospital['name'])
            takeoff(uxv_name,uav_pub)
            go_to_position(target_yaw,uav_pub,target_pos)
            vehicle_unload(2,target_pos['name'],uxv_name)
            takeoff(uxv_name,uav_pub)
        else:
            target_yaw=find_yaw(target_pos)
            takeoff(uxv_name,uav_pub)
            go_to_position(target_yaw,uav_pub,target_pos)
            vehicle_unload(2,target_pos['name'],uxv_name)
            takeoff(uxv_name,uav_pub)

    if percentage_of_fuel<float(12):
        

        target_yaw=0
        takeoff(uxv_name,uav_pub)
        near_charge_station=find_charge_station(target_pos,charge_stations)
        go_to_position(target_yaw,uav_pub,near_charge_station)
        while not percentage_of_fuel>95:
            pass
    
    request = rospy.ServiceProxy('/karmasim_node/vehicle_stop', VehicleStop)
    result = request(uxv_name)


