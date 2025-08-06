import csv
import re
import math
# import pandas as pd
from typing import List, Dict 
filename = "test.csv"  # File name
file_handle = open(filename, "r", encoding='utf8')
list=[]
table: List[Dict[str,float]] = []
df = csv.DictReader(file_handle)
num_of_legs = 6
num_of_DOF = 4
total_motor_state=[]
total_time_array=[]
previous_motor_state=[[0 for _ in range(num_of_DOF)] for _ in range(num_of_legs)]
# print(previous_states)
# Steps: make list of topics. interate through topics, make modules, parse through modeules to get velocity --> kinematics 
for row in df:
    motor_state = [[0 for _ in range(num_of_DOF)] for _ in range(num_of_legs)]
    for item in row:
        if item=='__time':
            total_time_array.append(float(row[item]))
        split_string=re.split(r'/',item)
        if split_string[-1]=='pos_estimate':
            DOF_list=re.findall(r'\d+', split_string[1])
            DOF=int(DOF_list[0])
            # print(DOF)
            row_pos=DOF%10
            column_pos=math.floor(DOF/10)
            try:
                i=float(row[item])
                motor_state[row_pos-1][column_pos-1]=i
                previous_motor_state[row_pos-1][column_pos-1]=i
            except ValueError:
                motor_state[row_pos-1][column_pos-1]=None#previous_motor_state[row_pos-1][column_pos-1] 
    total_motor_state.append(motor_state)
# print(total_motor_state)

previous_motor_state=[[0 for _ in range(3)] for _ in range(num_of_legs)]
# print("prev",previous_motor_state)
velocity_list=[[0 for _ in range(6)] for _ in range(len(total_time_array))]
prev_velocity=0
velocity=0
# print(len(total_time_array))
for t in range(len(total_time_array)):
    for module in range(num_of_legs):
        if total_motor_state[t][module][2] is not None: 
            # print('play',total_motor_state[t][module][2])
            # print('fe',previous_motor_state[module][0])
            velocityA=(total_motor_state[t][module][2]-previous_motor_state[module][0])/(total_time_array[t]-previous_motor_state[module][1])
            # velocityB=(total_motor_state[t][module][2]-previous_motor_state[module][0])/(total_time_array[t]-previous_motor_state[module][1])
            previous_motor_state[module][0]=total_motor_state[t][module][2]
            previous_motor_state[module][1]=total_time_array[t]
            previous_motor_state[module][2]=velocity
            # prev_velocity=velocity
            # print(module, previous_velocity)
        if total_motor_state[t][module][2] is None:
            velocity=previous_motor_state[module][2]
    # print("stae",total_motor_state[t])
    # print("time",total_time_array[t])
    # print(previous_motor_state[module][2])
        velocity_list[t][module]=velocity
# print(velocity_list[3644])
print("velocty", velocity_list)
# print('state',total_motor_state[3644])
# print('time',total_time_array[3644])
# # time & module position list --> Get velocity output (need module states [suspension pos, steer_pos, drive_velocity])--> need to get drive velocity
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

