import numpy as np
import skfuzzy as fuzz
import skfuzzy.membership as mf
import math

def calculate_Position(dorlion_data,enemy_data,enemy_odom):
    value = None
    dorlion_pos = [dorlion_data.x,dorlion_data.y]
    print(dorlion_pos)
    enemy_pos = [enemy_data.x,enemy_data.y]
    print(enemy_pos)
    enemy_yaw = quaterionToRads(enemy_odom)
    area = find_area(enemy_yaw)

    #bölge1
    if dorlion_pos[0] <= enemy_pos[0] and dorlion_pos[1] >= enemy_pos[1]:
        print("1")
        if area == 1:
            value = normalized_value(enemy_yaw,270,360,0.0001,0.4)
        elif area == 2:
            value = normalized_value(enemy_yaw,0,90,0.2,0.6)
        elif area == 3:
            value = normalized_value(enemy_yaw,180,270,0.4,0.8)
        else:
            value = normalized_value(enemy_yaw,90,180,0.6,1.0)
            
    #bölge 2        
    elif (dorlion_pos[0] <= enemy_pos[0]) and (dorlion_pos[1] <= enemy_pos[1]):
        print("2")
        if area == 1:
            value = normalized_value(enemy_yaw,270,360,0.2,0.6)
        elif area == 2:
            value = normalized_value(enemy_yaw,0,90,0.0001,0.4)
        elif area == 3:
            value = normalized_value(enemy_yaw,180,270,0.6,1.0)
        else:
            value = normalized_value(enemy_yaw,90,180,0.4,0.8)
    
    #bölge3  
    elif dorlion_pos[0] >= enemy_pos[0] and dorlion_pos[1] >= enemy_pos[1]:
        print("3")
        if area == 1:
            value = normalized_value(enemy_yaw,270,360,0.2,0.6)
        elif area == 2:
            value = normalized_value(enemy_yaw,0,90,0.6,1.0)
        elif area == 3:
            value = normalized_value(enemy_yaw,180,270,0.0001,0.4)
        else:
            value = normalized_value(enemy_yaw,90,180,0.2,0.6)
    
    #bölge4 
    elif dorlion_pos[0] >= enemy_pos[0] and dorlion_pos[1] <= enemy_pos[1]:
        print("4")
        if area == 1:
            value = normalized_value(enemy_yaw,270,360,0.6,1.0)
        elif area == 2:
            value = normalized_value(enemy_yaw,0,90,0.4,0.8)
        elif area == 3:
            value = normalized_value(enemy_yaw,180,270,0.2,0.6)
        else:
            value = normalized_value(enemy_yaw,90,180,0.0001,0.4)
    if value <= 0.0:
        value = 0.001
    if value > 0.9:
        value = 0.89999999
    print(value)
    return value
    
    
def normalized_value(value,old_min,old_max,new_min,new_max):
    
    return ((value-old_min)/(old_max-old_min))*(new_max-new_min) + new_min

def find_area(angle):
    
    if angle<=90:
        return 2
    elif angle <=180:
        return 4
    elif angle <=270:
        return 3
    else:
        return 1
    
def quaterionToRads(data):
    
    x = data.pose.pose.orientation.x
    y = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w

    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yawZActual = math.atan2(t3, t4)
    if yawZActual < 0:
        yawZActual = 2*math.pi + yawZActual

    return math.degrees(yawZActual)