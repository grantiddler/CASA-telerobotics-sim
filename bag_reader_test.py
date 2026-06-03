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

def parse_data(path):
    bagpath = Path(path)
    velocities = {}
    for i in range(3):
        velocities[i] = {}
        for j in range(2 * friction_steps + 1):
            velocities[i][j] ={"pos": [], "vel": []}
       
   # Create reader instance and open for reading.
    with AnyReader([bagpath], default_typestore=typestore) as reader:
        connections = reader.connections
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            if(connection.msgtype=="sensor_msgs/msg/JointState"):
                velocities[int(connection.topic[-4:-3])][int(connection.topic[-3:-1])]["pos"].append(msg.position)
                velocities[int(connection.topic[-4:-3])][int(connection.topic[-3:-1])]["vel"].append(msg.velocity)
            

        
    return velocities

def average_slip(velocities):
   
   slip = {}
   
   for i in velocities.keys():
      slip[i] = {}
      for j in velocities[i]:
         
         
         
        slip_trans = []
        slip_tan = []
        slip_rot = []
        
        
        for k in range(len(velocities[i][j]["vel"])):
            vel = velocities[i][j]["vel"][k]
            pos = velocities[i][j]["pos"][k]
            
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
            

        slip[i][j] = {"friction": get_friction(i,j),"transverse": np.mean(slip_trans), "tangential": np.mean(slip_tan), "rotational": np.mean(slip_rot)}

   return slip


def plot_slip(slip, slip_type, subplot):
    for i in range(3):
        data = []
        friction_variation = []
        for j in slip[i]:
            data.append(slip[i][j][slip_type])
            friction_variation.append((j - friction_steps) * friction_step_size * 100)
        subplot.plot(friction_variation, np.abs(data))
        
   
    return
 
slip = average_slip(parse_data("bags/one_side"))


slip_types = ["rotational", "transverse", "tangential"]
units = ["rad/s", "m/s", "m/s"]
for i in range(3):
    sub = plt.subplot(3, 3, 1 + i)
    # sub.set_ylim(0,0.5)
    sub.set_title(f"{slip_types[i]} slip")
    sub.set_xlabel("percent variation of parameter")
    sub.set_ylabel(f"average magnitude of slip ({units[i]})")
    plot_slip(slip, slip_types[i], sub)


slip = average_slip(parse_data("bags/point_turn"))

slip_types = ["rotational", "transverse", "tangential"]
units = ["rad/s", "m/s", "m/s"]
for i in range(3):
    sub = plt.subplot(3, 3, 4 + i)
    # sub.set_ylim(0,0.5)
    sub.set_title(f"{slip_types[i]} slip")
    sub.set_xlabel("percent variation of parameter")
    sub.set_ylabel(f"average magnitude of slip ({units[i]})")
    plot_slip(slip, slip_types[i], sub)
    
slip = average_slip(parse_data("bags/run_forward"))

slip_types = ["rotational", "transverse", "tangential"]
units = ["rad/s", "m/s", "m/s"]
for i in range(3):
    sub = plt.subplot(3, 3, 7 + i)
    # sub.set_ylim(0,0.5)
    sub.set_title(f"{slip_types[i]} slip")
    sub.set_xlabel("percent variation of parameter")
    sub.set_ylabel(f"average magnitude of slip ({units[i]})")
    plot_slip(slip, slip_types[i], sub)
    
plt.show()
