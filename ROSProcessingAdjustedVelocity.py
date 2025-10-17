import csv
import re
import math
import numpy as np
# import pandas as pd
from typing import List, Dict 
import matplotlib.pyplot as plt 
import matplotlib.ticker as ticker


import gpxpy
import gpxpy.gpx

from geopy.distance import geodesic

import matplotlib.pyplot as plt

gpx_file = open('test_1_30.gpx', 'r')

gpx = gpxpy.parse(gpx_file)

gps_coordinates = []
cartesian_coordinates = []

x_values = []
y_values = []
elevations = []

for track in gpx.tracks:
    for segment in track.segments:
        for point in segment.points:
            gps_coordinates.append((point.time.timestamp(), point.latitude, point.longitude, point.elevation))

# Reference point (lat, lon)
reference_latitude = gps_coordinates[0][1]
reference_longitude = gps_coordinates[0][2]
# Change mototr state to get velocity instead of position 
# 
#Constants
number_of_modules=6
suspension_arm_length=.2 #in meters 
steer_arm_length= 0.1#in meters
## Given a module state --> get the velocity
chassis_length = 1.2#total length in meters 
chassis_width = 0.3 #total chassis width in meters
suspension_angles=[0]*6
wheel_diameter=0.2921 #in meters
suspension_pivot_offset_list=[0]*number_of_modules
drive_gear_ratio=20.7
for i in range(number_of_modules):
    bay=math.floor(i/2)
    if i%2:
        suspension_pivot_offset_list[i]=[chassis_width/2,chassis_length/2-bay*chassis_length/(number_of_modules/2-1)]
    if not i%2:
        suspension_pivot_offset_list[i]=[-chassis_width/2,chassis_length/2-bay*chassis_length/(number_of_modules/2-1)]

#Opening file

filename = "test_1_1.csv"  # File name
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
cmd_vel_list=[]
total_motor_velocity_state=[]
prev_cmd_vel_x=0
prev_cmd_vel_y=0
total_commanded_motor_velocity_state=[]
total_commanded_motor_state=[]

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
    velocity_motor_state = [[0 for _ in range(num_of_DOF)] for _ in range(num_of_legs)]
    commanded_motor_state=[[0 for _ in range(num_of_DOF)] for _ in range(num_of_legs)]
    commanded_velocity_motor_state=[[0 for _ in range(num_of_DOF)] for _ in range(num_of_legs)]
    current_cmd_vel=[0, 0]
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
        if split_string[-1]=='vel_estimate':
                DOF_list=re.findall(r'\d+', split_string[1])
                DOF=int(DOF_list[0])
                row_pos=DOF%10
                column_pos=math.floor(DOF/10)
                try:
                    i=float(row[item])

                    velocity_motor_state[row_pos-1][column_pos-1]=i
                except ValueError:
                    velocity_motor_state[row_pos-1][column_pos-1]=None
        if item=='/cmd_vel/linear/x': 
            try: 
                i=float(row[item])
                current_cmd_vel[0]=i*.8*4
                prev_cmd_vel_x=i*.8*4
            except ValueError:
                current_cmd_vel[0]=prev_cmd_vel_x
        if item=='/cmd_vel/linear/y':
            try:
                i=float(row[item])
                current_cmd_vel[1]=i*.8
                prev_cmd_vel_y=i*.8
            except ValueError:
                current_cmd_vel[1]=prev_cmd_vel_y

    cmd_vel_list.append(current_cmd_vel)
    total_motor_state.append(motor_state)
    total_motor_velocity_state.append(velocity_motor_state)
# print("motor", total_motor_velocity_state)
# print(cmd_vel_list)
previous_motorA_state=0#[[0 for _ in range(3)] for _ in range(num_of_legs)]
previous_motorB_state=0#[[0 for _ in range(3)] for _ in range(num_of_legs)]
previous_steer_state=[0]*6
velocity_list_rev=[[0 for _ in range(6)] for _ in range(len(total_time_array))]
drive_mag_list=[[0 for _ in range(6)] for _ in range(len(total_time_array))]
steer_position_list=[[0 for _ in range(6)] for _ in range(len(total_time_array))]
# print(steer_position_list)
prev_velocity=0
velocityB=0
velocityA=0
steer_position=0
#Getting drive vector 
#Gets drive list --> for each time [mod1_vec, ..., mod6_vec]
# print(previous_motor_state)
velocity_list=[[0 for _ in range(6)] for _ in range(len(total_time_array))]
#Previous_motor_state in the form of [motor_position, timestamp, drive_velocity, steer_position]
#motor_state --> [timestamp,module,dof]
# for t in range(len(total_time_array)):
#     for i in range(number_of_modules):
#         if total_motor_state[t][i][2] is not None:
#             velocityA=(total_motor_state[t][i][2]-previous_motorA_state[i][1])/(total_time_array[t]-previous_motorA_state[i][0])
#             previous_motorA_state[i][0]=total_time_array[t]
#             previous_motorA_state[i][1]=total_motor_state[t][i][2]
#             previous_motorA_state[i][2]=velocityA
#         elif total_motor_state[t][i][2] is None:
#             velocityA=previous_motorA_state[i][2]
#         if total_motor_state[t][i][3] is not None:
#             velocityB=(total_motor_state[t][i][3]-previous_motorB_state[i][1])/(total_time_array[t]-previous_motorB_state[i][0])
#             previous_motorB_state[i][0]=total_time_array[t]
#             previous_motorB_state[i][1]=total_motor_state[t][i][3]
#             previous_motorB_state[i][2]=velocityB
#         elif total_motor_state[t][i][3] is None:
#             velocityB=previous_motorB_state[i][2]
#         velocity_list[t][i]=(((velocityA+velocityB)/2)/drive_gear_ratio)*np.pi*wheel_diameter

#         #finding Steer Position
#         if total_motor_state[t][i][1] is not None:
#             steer_position=total_motor_state[t][i][1]
#             previous_steer_state[i]=total_motor_state[t][i][1]
#         elif total_motor_state[t][i][1] is None: 
#             steer_position=previous_steer_state[i]
#         if i%2: 
#             steer_position_list[t][i]=2*np.pi*steer_position
#         elif i%2 is not None: 
#             steer_position_list[t][i]=np.pi+(2*np.pi*steer_position)
previous_motorA_state=[0]*6
previous_motorB_state=[0]*6
for t in range(len(total_time_array)):
    for i in range(number_of_modules):
        #Finding wheel velocity 
        if total_motor_velocity_state[t][i][2] is not None:
            velocityA=total_motor_velocity_state[t][i][2]
            previous_motorA_state[i]=velocityA
        elif total_motor_velocity_state[t][i][2] is None:
            velocityA=previous_motorA_state[i]
        if total_motor_velocity_state[t][i][3] is not None:
            velocityB=total_motor_velocity_state[t][i][3]
            previous_motorB_state[i]=velocityB
        elif total_motor_velocity_state[t][i][3] is None:
            velocityB=previous_motorB_state[i]
        # velocity_list[t][i]=velocityA
        velocity_list[t][i]=(((velocityA+velocityB)/2)/drive_gear_ratio)*np.pi*wheel_diameter
        # velocity_list[t][i]=(velocityA+velocityB)/2

        #finding Steer Position
        if total_motor_state[t][i][1] is not None:
            steer_position=total_motor_state[t][i][1]
            previous_steer_state[i]=total_motor_state[t][i][1]
        elif total_motor_state[t][i][1] is None: 
            steer_position=previous_steer_state[i]
        if i%2: 
            steer_position_list[t][i]=2*np.pi*steer_position
        elif i%2 is not None: 
            steer_position_list[t][i]=np.pi+(2*np.pi*steer_position)

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
commanded_position_x=0
commanded_position_y=0
robot_commanded_position_x=[0]*len(total_time_array)
robot_commanded_position_y=[0]*len(total_time_array)

for t in range(len(total_time_array)-1):
        time_step=total_time_array[t+1]-total_time_array[t]
        drive_vector=get_drive_vector(steer_position_list[t],velocity_list[t])
        contacts=get_contact_point(steer_position_list[t],suspension_angles)
        robot_velocity[t]=get_motion(drive_vector, contacts)
        robot_x_velocity[t]=robot_velocity[t][0]
        robot_y_velocity[t]=robot_velocity[t][1]
        position_x=position_x+robot_velocity[t][0]*time_step
        position_y=position_y+robot_velocity[t][1]*time_step
        robot_x_position[t]=position_x
        robot_y_position[t]=position_y
        delta_x_commanded=cmd_vel_list[t][0]*time_step
        delta_y_commanded=cmd_vel_list[t][1]*time_step
        commanded_position_x+=delta_x_commanded
        commanded_position_y+=delta_y_commanded
        robot_commanded_position_x[t]=commanded_position_x
        robot_commanded_position_y[t]=commanded_position_y
    

# print("x",robot_x_position)

# print("y",robot_y_velocity)


# test_list=get_drive_vector(steer_position_list[5],drive_mag_list[5])
# contacts=get_contact_point(steer_position_list[5],suspension_angles)
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
# print(velocityA)
# for i in range(len(total_time_array)):
#     drive1.append(steer_position_list[i][0])
#     drive2.append(steer_position_list[i][1])
#     drive3.append(steer_position_list[i][2])
#     drive4.append(steer_position_list[i][3])
#     drive5.append(steer_position_list[i][4])
#     drive6.append(steer_position_list[i][5])
# print("NUM", len(total_time_array))
# print(velocity_list)
# # print(total_time_array)
for i in range(len(total_time_array)):
    drive1.append(velocity_list[i][0])
    drive2.append(velocity_list[i][1])
    drive3.append(velocity_list[i][2])
    drive4.append(velocity_list[i][3])
    drive5.append(velocity_list[i][4])
    drive6.append(velocity_list[i][5])

# #Plotting positions for each module 
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

# for i in range(len(total_time_array)):
#     if total_motor_velocity_state[i][0] != None:
#         drive1.append(total_motor_velocity_state[i][0][3])
#     if total_motor_velocity_state[i][1] != None:
#         drive2.append(total_motor_velocity_state[i][1][3])
#     if total_motor_velocity_state[i][2] != None:
#         drive3.append(total_motor_velocity_state[i][2][3])
#     if total_motor_velocity_state[i][3] != None:
#         drive4.append(total_motor_velocity_state[i][3][3])
#     if total_motor_velocity_state[i][4] != None:
#         drive5.append(total_motor_velocity_state[i][4][3])
#     if total_motor_velocity_state[i][5] != None:
#         drive6.append(total_motor_velocity_state[i][5][3])


# print(drive6)

# print(drive1)
# print(c)
# fig, ax = plt.subplots(figsize=(20, 20))
# list=[range(len(drive6))]
# print(len(drive2))
# fig = plt.figure(figsize=(4,6))
plt.scatter(robot_x_position, robot_y_position,marker='.',c="tab:red", label="Forward Kinematics Traj.")
# plt.scatter(total_time_array,drive1, s=3, c="r")
# plt.scatter(total_time_array,drive2, s=3, c="orange")
# plt.scatter(total_time_array,drive3, s=3, c="y")
# plt.scatter(total_time_array,drive4, s=3, c="g")
# plt.scatter(total_time_array,drive5, s=3, c="b")
# plt.scatter(total_time_array,drive6, s=3, c="purple")

# plt.scatter(total_time_array, robot_y_velocity)



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
# plt.scatter(total_time_array,robot_y_position, c="red")






time_values=[]
for time, latitude, longitude, elevation in gps_coordinates:

    # Y (north-south)
    north_m = geodesic((reference_latitude, reference_longitude), (latitude, reference_longitude)).meters
    if latitude < reference_latitude:
        north_m *= -1

    # X (east-west)
    east_m = geodesic((reference_latitude, reference_longitude), (reference_latitude, longitude)).meters
    if longitude < reference_longitude:
        east_m *= 1

    cartesian_coordinates.append((time, east_m, north_m, elevation))

    x_values.append(north_m)
    y_values.append(east_m)
    time_values.append(time)
    elevations.append(elevation)

# for point in cartesian_coordinates:
    # print(f"Time: {point[0]:.1f}, X: {point[1]:.3f} m, Y: {point[2]:.3f} m")
# print(len(time_values))
# # print(len(total_time_array))
error_x_list=[]
error_y_list=[]
total_error_list=[]
distance_travelled=[]
distance_counter=0
odom_start_x=0
odom_start_y=0
gps_start_x=0
gps_start_y=0
counter=0
distance_steps=[]
previous_distance=0
relative_error_list_x=[]
relative_error_list_y=[]
relative_error_list=[]
distance_travelled_shorten=0
for index, t in enumerate(total_time_array):
    if counter>len(time_values)-1:
        counter=len(time_values)-1
    if t == time_values[counter]:
        if distance_counter>2:
            distance_travelled_shorten+=distance_counter
            distance_steps.append(distance_travelled_shorten)
            gps_delta_x=gps_start_x-x_values[counter]
            gps_delta_y=gps_start_y-y_values[counter]
            odom_delta_x=odom_start_x-robot_x_position[index]
            odom_delta_y=odom_start_y-robot_y_position[index]
            relative_error_x=odom_delta_x-gps_delta_x
            relative_error_y=odom_delta_y-gps_delta_y
            total_relative_error=math.sqrt(relative_error_x**2+relative_error_y**2)
            relative_error_list_y.append(relative_error_y)
            relative_error_list_x.append(relative_error_x)
            relative_error_list.append(total_relative_error)
            odom_start_x=robot_x_position[index]
            odom_start_y=robot_y_position[index]
            gps_start_x=x_values[counter]
            gps_start_y=y_values[counter]


            
            distance_counter=0

        
        total_distance=math.sqrt(x_values[counter]**2+y_values[counter]**2)
        error_x=x_values[counter]-robot_x_position[index]
        error_y=y_values[counter]-robot_y_position[index]
        total_error=math.sqrt(error_x**2+error_y**2)
        error_x_list.append(error_x)
        error_y_list.append(error_y)
        total_error_list.append(total_error)
        distance_travelled.append(total_distance)
        # print(previous_distance, total_distance)
        delta_distance=total_distance-previous_distance
        # print(delta_distance)
        previous_distance=total_distance
        counter+=1
        distance_counter+=delta_distance
        
print("RPE", relative_error_list)

# print(distance_steps)
# print(relative_error_list)
print("RSME",sum(total_error_list)/(len(total_error_list)))
# print(len(error_x_list))
# print()
# Plotting
plt.scatter(x_values, y_values, marker=".", c="tab:blue", label="GPS Traj")
plt.scatter(robot_commanded_position_x,robot_commanded_position_y, marker = '.',c="tab:green", label="Commanded Traj.")



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



ax=plt.gca()
ax.set(xlim=(-2,10),ylim=(-2,22))
ax.xaxis.set_major_locator(ticker.MultipleLocator(2))
ax.yaxis.set_major_locator(ticker.MultipleLocator(2))

# plt.gca().set_aspect("equal")
plt.grid(True)

# Make axes equal
# plt.gca().set_aspect('equal', adjustable='box')
# plt.axis('square')


# plt.axis('equal', 'box')  # This ensures 1 unit on x == 1 unit on y
# print("check")

plt.grid(visible=True)
plt.title("Constant Heading Test A")
# plt.xlim(-2,10)
# plt.ylim(-2,22)
# plt.legend()
plt.xlabel("x (meters)")
plt.ylabel("y (meters)")
ax.set_aspect('equal')
# plt.margins(x=.25)
plt.figure()
plt.plot(distance_travelled,error_x_list, c='r')
plt.plot(distance_travelled,error_y_list, c='blue')
plt.plot(distance_travelled, total_error_list, c='green')
plt.figure()
plt.scatter(distance_steps, relative_error_list_x, c="red")
plt.scatter(distance_steps, relative_error_list_y, c="green")
plt.scatter(distance_steps, relative_error_list, c="blue")

# plt.scatter(time_values, error_x_list)
plt.show()





# # Make axes equal
# plt.gca().set_aspect('equal', adjustable='box')
# plt.axis('equal')  # This ensures 1 unit on x == 1 unit on y

# plt.tight_layout()
# plt.show()