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
                # print(msg.velocity)
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
                
                ideal_vel = []
                ideal_rot = []

                for k in range(len(velocities[h][i][j]["vel"])):
                    vel = velocities[h][i][j]["vel"][k]
                    pos = velocities[h][i][j]["pos"][k]
                    
                    r = R.from_quat(pos[-4:])
                    # print(r.as_euler('xyz'))
                    # print(vel)
                    
                    heading = r.as_euler('xyz')[0]
                    
                    v_ideal = radius * (float(vel[0]) + float(vel[1]) + float(vel[2]) + float(vel[3])) / 4
                    v_trans = np.cos(heading) * float(vel[4]) - np.sin(heading) * float(vel[5])
                    v_tan =  (np.sin(heading) * float(vel[4]) + np.cos(heading) * float(vel[5]))
                    
                    
                    omega_ideal = radius * (float(vel[0]) + float(vel[1]) - float(vel[2]) - float(vel[3])) / (2 * width)
                    omega_real = float(vel[-1])
                    
                    # print(f"{h} {v_ideal} {omega_ideal}")

                    rotational_slip = omega_real #- omega_ideal
                    tangential_slip = v_tan # - v_ideal
                    transverse_slip = v_trans

                    slip_trans.append(transverse_slip)
                    slip_tan.append(tangential_slip)
                    slip_rot.append(rotational_slip)
                    
                    ideal_vel.append(v_ideal)
                    ideal_rot.append(omega_ideal)
                    

                slip[h][i][j] = {"friction": get_friction(i,j),"transverse": np.mean(slip_trans), "tangential": np.mean(slip_tan), "rotational": np.mean(slip_rot), "tangential ideal": np.mean(ideal_vel), "rotational ideal": np.mean(ideal_rot)}

    return slip


def plot_velocity(slip, param_swept = None):
    n = 0
    size = len(slip.keys())
    for i in slip:
        # print(i)
        if(i == "[0, 0]" or i == "[0.0, 0.0]" ):
            size -= 1
            continue
        
        transverse = []
        tangential = []
        rotational = []
        ideal_vel = []
        ideal_rot = []
        
        for j in slip[i]:
            if(param_swept != None and (param_swept != j)):
                continue
            friction = []
            transverse_temp = []
            tangential_temp = []
            rotational_temp = []
            ideal_vel_temp = []
            ideal_rot_temp = []
                    
            
            for k in slip[i][j]:
                transverse_temp.append(slip[i][j][k]["transverse"])
                tangential_temp.append(slip[i][j][k]["tangential"])
                rotational_temp.append(slip[i][j][k]["rotational"])
                ideal_rot_temp.append(slip[i][j][k]["rotational ideal"])
                ideal_vel_temp.append(slip[i][j][k]["tangential ideal"])
                friction.append(slip[i][j][k]["friction"][j])
                
                
                # print(slip[i][j][k]["friction"][j])

            transverse.append(transverse_temp)
            tangential.append(tangential_temp)
            rotational.append(rotational_temp)
            ideal_vel.append(ideal_vel_temp)
            ideal_rot.append(ideal_rot_temp)
            
                
        sub = plt.subplot(3, size, 1 + n)
        sub.set_title(i)
        plt.plot(friction, (np.transpose(transverse)))
        sub.sharey(plt.subplot(3, size, 1))
        
        sub = plt.subplot(3, size, 1 + size + n)
        plt.plot(friction, (np.transpose(tangential)))
        plt.plot(friction, (np.transpose(ideal_vel)))
        sub.sharey(plt.subplot(3, size, 1 + size))
        
        
        
        sub = plt.subplot(3, size, 1 + 2 * size + n)
        plt.plot(friction, np.abs(np.transpose(rotational)))
        plt.plot(friction, np.abs(np.transpose(ideal_rot)))
        sub.sharey(plt.subplot(3, size, 1 + 2 * size))
        
        
        
        n += 1
        sub = plt.subplot(3, size, 1)
        sub.set_ylabel("transverse velocity (m/s)")
        
        sub = plt.subplot(3, size, 1 + size)
        sub.set_ylabel("tangential velocity (m/s)")
        
        sub = plt.subplot(3, size, 1 + 2 * size)
        sub.set_ylabel("angular velocity (rad/s)")
    plt.show()
    
    
    


slip = average_slip(parse_data("data/bag6"))
plot_velocity(slip, 0)
