from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

bagpath = Path('bags/bag0')

# Create a type store to use if the bag has no message definitions.
typestore = get_typestore(Stores.ROS2_HUMBLE)

# Create reader instance and open for reading.
with AnyReader([bagpath], default_typestore=typestore) as reader:
    connections = reader.connections
    for connection, timestamp, rawdata in reader.messages(connections=connections):
         msg = reader.deserialize(rawdata, connection.msgtype)
         if(connection.msgtype=="sensor_msgs/msg/JointState"):  
            print(connection.topic)
            print(msg.name)
            print(msg.position)
            print(msg.velocity)
            
         print()
