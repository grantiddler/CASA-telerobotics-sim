from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

bagpath = Path('bags/test4')

# Create a type store to use if the bag has no message definitions.
typestore = get_typestore(Stores.ROS2_HUMBLE)


base_friction = [2.0, 0.050, 0.015]
friction_step_size = 0.05
friction_steps = 2

def get_friction(var, sweep):
    friction = base_friction.copy()
    friction[var] = base_friction[var] * (1 + friction_step_size * (sweep - friction_steps))
    
    return friction


velocities = {}
# Create reader instance and open for reading.
with AnyReader([bagpath], default_typestore=typestore) as reader:
    connections = reader.connections
    for connection, timestamp, rawdata in reader.messages(connections=connections):
         msg = reader.deserialize(rawdata, connection.msgtype)
         if(connection.msgtype=="sensor_msgs/msg/JointState"):  
            print(connection.topic[-4:])
            print(f"variable swept: {int(connection.topic[-4:-3])} | sweep: {int(connection.topic[-3:-1])} | instance :  {int(connection.topic[-1])}")
            print(get_friction(int(connection.topic[-4:-3]), int(connection.topic[-3:-1])))
            
            print(msg.name)
            print(msg.position)
            print(msg.velocity)
            
         print()
