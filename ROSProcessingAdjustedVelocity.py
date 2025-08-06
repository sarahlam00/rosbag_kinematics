import csv
import re
import math
import numpy as np
# import pandas as pd
from typing import List, Dict 
import matplotlib.pyplot as plt 


import gpxpy
import gpxpy.gpx

from geopy.distance import geodesic

import matplotlib.pyplot as plt

# gpx_file = open('test_1.gpx', 'r')

# gpx = gpxpy.parse(gpx_file)

# gps_coordinates = []
# cartesian_coordinates = []

# x_values = []
# y_values = []
# elevations = []

# for track in gpx.tracks:
#     for segment in track.segments:
#         for point in segment.points:
#             gps_coordinates.append((point.time.timestamp(), point.latitude, point.longitude, point.elevation))

# # Reference point (lat, lon)
# reference_latitude = gps_coordinates[0][1]
# reference_longitude = gps_coordinates[0][2]

#Constants
number_of_modules=6
suspension_arm_length=.2 #in meters 
steer_arm_length= 0.1#in meters
## Given a module state --> get the velocity
chassis_length = 1.2#total length in meters 
chassis_width = 0.3 #total chassis width in meters
suspension_angles=[0]*6
wheel_diameter=.254 #in meters
suspension_pivot_offset_list=[0]*number_of_modules
drive_gear_ratio=20.7
for i in range(number_of_modules):
    bay=math.floor(i/2)
    if i%2:
        suspension_pivot_offset_list[i]=[chassis_width/2,chassis_length/2-bay*chassis_length/(number_of_modules/2-1)]
    if not i%2:
        suspension_pivot_offset_list[i]=[-chassis_width/2,chassis_length/2-bay*chassis_length/(number_of_modules/2-1)]

#Opening file
filename = "tuesday_test_1.csv"  # File name
file_handle = open(filename, "r", encoding='utf8')
list=[]
table: List[Dict[str,float]] = []
df = csv.DictReader(file_handle)
num_of_legs = 6
num_of_DOF = 4
steer_position=0
total_motor_state=[]
total_time_array=[]
total_steer_list=[]
total_driveA_list=[]
total_driveB_list=[]
def get_vector(length, angle):
    vector=length*np.array([math.cos(angle),math.sin(angle)])
    return vector

def get_steer_pivot_point(suspension_angle_list,suspension_arm_length=0):
    #finding where the steer offsets are
    steer_pivot_offset=[0]*number_of_modules
    for i in range(number_of_modules):
        suspension_offset=suspension_arm_length*math.cos(2*np.pi*suspension_angle_list[i])
        if not(i%2):
            steer_pivot_offset[i]=np.array([suspension_pivot_offset_list[i][0]-suspension_offset,suspension_pivot_offset_list[i][1]])
        if i%2:
            steer_pivot_offset[i]=np.array([suspension_pivot_offset_list[i][0]+suspension_offset,suspension_pivot_offset_list[i][1]])
    return steer_pivot_offset

def get_contact_point(steer_angle_list,suspension_angle_list): 
    steer_pivot_point_list=get_steer_pivot_point(suspension_angle_list,suspension_arm_length)
    # print(steer_pivot_point_list)
    #Given suspesion and steer angle returns contact point of the wheel 
    contact_point_list=[0]*number_of_modules
    for i in range(number_of_modules):
        # print(2*math.pi*steer_angle_list[i])

        # print(get_vector(steer_arm_length,2*math.pi*steer_angle_list[i]))
        offset=get_vector(steer_arm_length,steer_angle_list[i])
        contact_point_list[i]=steer_pivot_point_list[i]+offset#get_vector(steer_arm_length, steer_angle_list[i])
    return contact_point_list


def get_motion(drive_vector,contact_point_list):
    #given steer/drive vectors
    state_matrix=[0]*2*number_of_modules
    module_matrix=[]
    for i in range(len(drive_vector)):
        for j in range(2):
            module_matrix.append(drive_vector[i][j])
    for k in range(2*number_of_modules):
        if k%2:
            state_matrix[k]=np.array([(k+1)%2,k%2,contact_point_list[math.floor(k/2)][(k+1)%2]])
        elif not k%2:
            state_matrix[k]=np.array([(k+1)%2,k%2,-contact_point_list[math.floor(k/2)][(k+1)%2]])
    inverse=np.linalg.pinv(state_matrix)
    velocity=np.matmul(inverse,module_matrix)
    return velocity




# Steps: make list of topics. interate through topics, make modules, parse through modeules to get velocity --> kinematics 
#Parsing through CSV file --> data formatting for each time, get [[mod1][mod2]...[mod6]] with each [mod]=4DOF 
for row in df:
    motor_state = [[0 for _ in range(num_of_DOF)] for _ in range(num_of_legs)]
    for item in row:
        if item=='__time':
            # print(row[item])
            total_time_array.append(float(row[item]))
        split_string=re.split(r'/',item)
        if split_string[-1]=='pos_estimate':
            DOF_list=re.findall(r'\d+', split_string[1])
            DOF=int(DOF_list[0])
            row_pos=DOF%10
            column_pos=math.floor(DOF/10)
            try:
                i=float(row[item])
                # print(i)
                motor_state[row_pos-1][column_pos-1]=i
            except ValueError:
                motor_state[row_pos-1][column_pos-1]=None
    # print(motor_state)
    total_motor_state.append(motor_state)
# print("motor", total_motor_state[16])

previous_motor_state=[[0 for _ in range(4)] for _ in range(num_of_legs)]
velocity_list_rev=[[0 for _ in range(6)] for _ in range(len(total_time_array))]
drive_mag_list=[[0 for _ in range(6)] for _ in range(len(total_time_array))]
steer_position_list=[[0 for _ in range(6)] for _ in range(len(total_time_array))]
# print(steer_position_list)
prev_velocity=0
velocity=0
steer_position=0
#Getting drive vector 
#Gets drive list --> for each time [mod1_vec, ..., mod6_vec]
# print(previous_motor_state)

#Previous_motor_state in the form of [motor_position, timestamp, drive_velocity, steer_position]

for t in range(len(total_time_array)):
    for module in range(num_of_legs):
        #Checking Drive
        if total_motor_state[t][module][2] is not None: 
            velocity=(total_motor_state[t][module][2]-previous_motor_state[module][0])/(total_time_array[t]-previous_motor_state[module][1])
            # velocityB=(total_motor_state[t][module][2]-previous_motor_state[module][0])/(total_time_array[t]-previous_motor_state[module][1])
            previous_motor_state[module][0]=total_motor_state[t][module][2]
            previous_motor_state[module][1]=total_time_array[t]
            previous_motor_state[module][2]=velocity
        elif total_motor_state[t][module][2] is None:
            velocity=previous_motor_state[module][2]      
            #Checking Steer  
        if total_motor_state[t][module][1] is not None:
            steer_position=total_motor_state[t][module][1]
            # print(steer_position)
            previous_motor_state[module][3]=steer_position
        elif total_motor_state[t][module][1] is None:
            steer_position=previous_motor_state[module][3]
        velocity_list_rev[t][module]=velocity
        steer_position_list[t][module]=steer_position
        if module%2: 
            drive_mag_list[t][module]=((velocity/drive_gear_ratio)*2*math.pi*(wheel_diameter/2)) #converting rps to rad/sec and then multiplying by radius 
            steer_position_list[t][module]=2*np.pi*steer_position
        elif not module%2: 
            drive_mag_list[t][module]=(((velocity/drive_gear_ratio)*2*math.pi*(wheel_diameter/2))) #converting rps to rad/sec and then multiplying by radius 
            steer_position_list[t][module]=np.pi-(2*np.pi*steer_position)
            
# print(len(steer_position_list))
# print(len(total_time_array))
# print(drive_mag_list)
def get_drive_vector(steer_position_list,drive_mag_list):
    steer_vector=[0]*number_of_modules
    drive_vector_list=[0]*number_of_modules
    for i in range(number_of_modules):
        steer_vector=get_vector(1, steer_position_list[i])        
        drive_vector_list[i]=[drive_mag_list[i]*steer_vector[1], drive_mag_list[i]*-steer_vector[0]]
    return drive_vector_list



robot_velocity=[0]*len(total_time_array)
robot_x_velocity=[0]*len(total_time_array)
robot_y_velocity=[0]*len(total_time_array)
robot_x_position=[0]*len(total_time_array)
robot_y_position=[0]*len(total_time_array)
position_x=0
position_y=0
for t in range(len(total_time_array)-1):
    time_step=total_time_array[t+1]-total_time_array[t]
    drive_vector=get_drive_vector(steer_position_list[t],drive_mag_list[t])
    contacts=get_contact_point(steer_position_list[t],suspension_angles)
    robot_velocity[t]=get_motion(drive_vector, contacts)
    robot_x_velocity[t]=robot_velocity[t][0]
    robot_y_velocity[t]=robot_velocity[t][1]
    position_x=position_x+robot_velocity[t][0]*time_step
    position_y=position_y+robot_velocity[t][1]*time_step
    robot_x_position[t]=position_x
    robot_y_position[t]=position_y

# print("x",robot_x_position)

# print("y",robot_y_velocity)


# test_list=get_drive_vector(steer_position_list[16],drive_mag_list[16])
# contacts=get_contact_point(steer_position_list[16],suspension_angles)
# pivots=get_steer_pivot_point(suspension_angles, suspension_arm_length)
# c=get_motion(test_list,contacts)
# print(robot_velocity)
# print(robot_x_position)
# print(robot_velocity)
drive1=[]
drive2=[]
drive3=[]
drive4=[]
drive5=[]
drive6=[]
# for i in range(len(total_time_array)):
#     drive1.append(drive_mag_list[i][0])
#     drive2.append(drive_mag_list[i][1])
#     drive3.append(drive_mag_list[i][2])
#     drive4.append(drive_mag_list[i][3])
#     drive5.append(drive_mag_list[i][4])
#     drive6.append(drive_mag_list[i][5])

for i in range(len(total_time_array)):
    drive1.append(velocity_list_rev[i][0])
    drive2.append(velocity_list_rev[i][1])
    drive3.append(velocity_list_rev[i][2])
    drive4.append(velocity_list_rev[i][3])
    drive5.append(velocity_list_rev[i][4])
    drive6.append(velocity_list_rev[i][5])

#Plotting positions for each module 
# for i in range(len(total_time_array)):
#     if total_motor_state[i][0] != None:
#         drive1.append(total_motor_state[i][0][3])
#     if total_motor_state[i][1] != None:
#         drive2.append(total_motor_state[i][1][3])
#     if total_motor_state[i][2] != None:
#         drive3.append(total_motor_state[i][2][3])
#     if total_motor_state[i][3] != None:
#         drive4.append(total_motor_state[i][3][3])
#     if total_motor_state[i][4] != None:
#         drive5.append(total_motor_state[i][4][3])
#     if total_motor_state[i][5] != None:
#         drive6.append(total_motor_state[i][5][3])




# print(drive1)
# print(c)
# fig, ax = plt.subplots(figsize=(20, 20))
list=[range(len(drive6))]
# print(len(drive1))
# plt.scatter(robot_x_position, robot_y_position)
plt.scatter(list,drive6, s=0.1)
# plt.scatter(total_time_ar/ray, )



# plt.scatter(total_time_array,drive_mag_list)
# plt.figure(figsize=(8, 8))
# print(len(steer_position_list))
# print(len(total_time_array))
# for i in range(6):
# print(robot_x_position[1],)
    # plt.scatter(total_time_array,steer_position_list)
# plt.scatter(robot_x_position,robot_y_position, s=10)
# print("y",robot_y_position)
# print("x", robot_x_position[-1])

# print("here",robot_x_position[len(total_time_array)-1], robot_y_position[len(total_time_array)-1])
# plt.scatter(robot_x_position[1],robot_y_position[1],c="g")
# plt.scatter(robot_x_position[len(total_time_array)-2],robot_x_position[len(total_time_array)-2],c="r")
# plt.scatter(robot_x_position[1:len(total_time_array)-2],robot_y_position[1:len(total_time_array)-2])
# plt.scatter(robot_x_position,robot_y_position)
# plt.scatter(total_time_array,robot_x_position)






# for time, latitude, longitude, elevation in gps_coordinates:

#     # Y (north-south)
#     north_m = geodesic((reference_latitude, reference_longitude), (latitude, reference_longitude)).meters
#     if latitude < reference_latitude:
#         north_m *= -1

#     # X (east-west)
#     east_m = geodesic((reference_latitude, reference_longitude), (reference_latitude, longitude)).meters
#     if longitude < reference_longitude:
#         east_m *= 1

#     cartesian_coordinates.append((time, east_m, north_m, elevation))

#     x_values.append(north_m)
#     y_values.append(east_m)
#     elevations.append(elevation)

# for point in cartesian_coordinates:
#     print(f"Time: {point[0]:.1f}, X: {point[1]:.3f} m, Y: {point[2]:.3f} m")

# # Plotting
# plt.plot(x_values, y_values, marker='o')



# x0, y0= [-chassis_width/2, chassis_width/2], [chassis_length/2,chassis_length/2]
# x1, y1= [-chassis_width/2, chassis_width/2], [-chassis_length/2,-chassis_length/2]
# x2,y2= [-chassis_width/2,-chassis_width/2], [chassis_length/2,-chassis_length/2]
# x3,y3= [chassis_width/2,chassis_width/2],[chassis_length/2,-chassis_length/2]
# plt.plot(x0,y0,x1,y1,x2,y2,x3,y3,c="black")
# for i in range(number_of_modules):
#     plt.plot(suspension_pivot_offset_list[i][0],suspension_pivot_offset_list[i][1],"bo")
#     plt.plot(pivots[i][0],pivots[i][1],"ro")
#     plt.plot(contacts[i][0],contacts[i][1],"go")
#     plt.plot([suspension_pivot_offset_list[i][0],pivots[i][0]],[suspension_pivot_offset_list[i][1],pivots[i][1]],"b")
#     plt.plot([pivots[i][0],contacts[i][0]],[pivots[i][1],contacts[i][1]],c="r")
#     ax.quiver(contacts[i][0],contacts[i][1],test_list[i][0],test_list[i][1], angles='xy', scale_units='xy', scale=1, color='b')




# plt.gca().set_aspect("equal")
plt.grid(True)

# Make axes equal
# plt.gca().set_aspect('equal', adjustable='box')
# plt.axis('equal')  # This ensures 1 unit on x == 1 unit on y

plt.grid()
plt.show()






# # Make axes equal
# plt.gca().set_aspect('equal', adjustable='box')
# plt.axis('equal')  # This ensures 1 unit on x == 1 unit on y

# plt.tight_layout()
# plt.show()















# print(velocity_list_rev[4500])
# print(drive_vector_list[4500])
# Now I have velolcity --> Translate to drive vectors 

# time & module position list --> Get velocity output (need module states [suspension pos, steer_pos, drive_velocity])--> need to get drive velocity
# num_of_states=3
# module_state=[[0 for _ in range(num_of_states)] for _ in range(num_of_legs)]
# total_module_state=[]
# for i in range(len(total_time_array)-1):
#     module_state=[[0 for _ in range(num_of_states)] for _ in range(num_of_legs)]
#     # print("time",total_time_array[i])
#     for j in range(num_of_legs):
#         time_step=total_time_array[i+1]-total_time_array[i]
#         velocity_motor_A=(total_motor_state[i+1][j][2]-total_motor_state[i][j][2])/time_step
#         velocity_motor_B=(total_motor_state[i+1][j][3]-total_motor_state[i][j][3])/time_step
        
#         average_velocity=(velocity_motor_A+velocity_motor_B)/2
        # if i==140:
        #     print("velocity",average_velocity)
        #     print("motor",total_motor_state[i])
        #     print("next",total_motor_state[i+1])
        #     print("time", total_time_array[i])
        #     print("a",velocity_motor_A)
        #     print("b",velocity_motor_B)
        #     print("delta", time_step)
        # module_state[j][2]=average_velocity
        
#         module_state[j][1]=total_motor_state[i][j][1]
#         # print("average",j, average_velocity)
#     # print('mod',module_state)
#     total_module_state.append(module_state)
# print("total",total_module_state[3625])
# print(total_time_array[3625])

# def get_velocity(position_list,time_list):
#     prev_pos=0
#     prev_time=0
#     velocity_list=[]
#     for i in range(len(position_list)):
#         if position_list[i] is not None: 
#             velocity=(position_list[i]-prev_pos)/(time_list[i]-prev_time)
#             prev_pos=position_list[i]
#             prev_time=time_list[i]
#             velocity_list.append([velocity,time_list[i]])
#         # print('hi')
#     return velocity_list

# position_list=[1,3,6, None, None, 7]
# time_list=[1,2,3,4,5,6]

# x=get_velocity(position_list, time_list)
# print(x)
