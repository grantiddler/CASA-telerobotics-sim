from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

# Create a type store to use if the bag has no message definitions.
typestore = get_typestore(Stores.ROS2_HUMBLE)


base_friction = [2.0, 0.050, 0.015]
friction_step_size = 0.05
friction_steps = 10

   
width = 0.21779 * 2
radius = .05


def get_friction(var, sweep):
    friction = base_friction.copy()
    friction[var] = base_friction[var] * (1 + friction_step_size * (sweep - friction_steps))
    
    return friction

def parse_data(path, velocities = {}):
    bagpath = Path(path)

    control = str([0,0])
   # Create reader instance and open for reading.
    with AnyReader([bagpath], default_typestore=typestore) as reader:
        connections = reader.connections
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            
            

            if(connection.msgtype=="sensor_msgs/msg/JointState"):
                swept_param = int(connection.topic[-4:-3])
                sweep_itr = int(connection.topic[-3:-1])
                
                if(control not in velocities):
                    velocities[control] = {}
                if(swept_param not in velocities[control]):
                    for i in range(swept_param + 1):
                        if(i not in velocities[control]):
                            velocities[control][i] = {}
                    
                if(sweep_itr not in velocities[control][swept_param]):
                    # print(f"{sweep_itr} is not in {control} {swept_param} {velocities[control][swept_param].keys()}")
                    for i in range(sweep_itr + 1):
                        if(i not in velocities[control][swept_param]):
                            velocities[control][swept_param][i] = {"pos": [], "vel": []}
                
                velocities[control][swept_param][sweep_itr]["pos"].append(msg.position)
                velocities[control][swept_param][sweep_itr]["vel"].append(msg.velocity)
            elif(connection.topic == "/control"):
                control = str([msg.x, msg.y])

            

        
    return velocities

def average_slip(velocities):
   
    slip = {}
    for h in velocities:
        slip[h] = {}
        for i in velocities[h]:
            slip[h][i] = {}
            for j in velocities[h][i]:
                

                slip_trans = []
                slip_tan = []
                slip_rot = []
                
                
                for k in range(len(velocities[h][i][j]["vel"])):
                    vel = velocities[h][i][j]["vel"][k]
                    pos = velocities[h][i][j]["pos"][k]
                    
                    r = R.from_quat(vel[-4:])
                    
                    heading = r.as_euler('xyz')[0]
                    
                    v_ideal = radius * (float(vel[0]) + float(vel[1]) + float(vel[2]) + float(vel[3])) / 4
                    v_tan = np.cos(heading) * float(vel[4]) - np.sin(heading) * float(vel[5])
                    v_trans =  np.sin(heading) * float(vel[4]) + np.cos(heading) * float(vel[5])
                    
                    
                    omega_ideal = radius * (float(vel[0]) + float(vel[1]) - float(vel[2]) - float(vel[3])) / (2 * width)
                    omega_real = float(vel[-2])

                    rotational_slip = omega_real - omega_ideal
                    tangential_slip = v_tan - v_ideal
                    transverse_slip = v_trans

                    slip_trans.append(transverse_slip)
                    slip_tan.append(tangential_slip)
                    slip_rot.append(rotational_slip)
                    

                slip[h][i][j] = {"friction": get_friction(i,j),"transverse": np.mean(slip_trans), "tangential": np.mean(slip_tan), "rotational": np.mean(slip_rot)}

    return slip


def plot_slip(slip, slip_type, subplot):
    for i in range(slip):
        data = []
        friction_variation = []
        for j in slip[i]:
            data.append(slip[i][j][slip_type])
            friction_variation.append((j - friction_steps) * friction_step_size * 100)
        subplot.plot(friction_variation, np.abs(data))
        
   
    return
 
slip = average_slip(parse_data("bags/point_turn"))

for i in slip:
    print(i)
    for j in slip[i]:
        print(f"   {j}")
        for k in slip[i][j]:
            print(f"      {k}")