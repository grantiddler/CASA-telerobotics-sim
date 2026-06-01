import rosbag2_py


reader = rosbag2_py.SequentialReader()
storage_options = rosbag2_py.StorageOptions(
    uri='bag0',
    storage_id='sqlite3')
converter_options = rosbag2_py.ConverterOptions('', '')
reader.open(storage_options, converter_options)

i = 0
while reader.has_next():
    i += 1
    msg = reader.read_next()
    print(msg)
    # print(msg[1])

print(i)

