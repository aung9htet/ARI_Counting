"""Read original ROS messages without loading a model or exported dataset."""
import argparse
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def main():
    p=argparse.ArgumentParser(); p.add_argument('bag'); args=p.parse_args()
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag,storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
    types={t.name:get_message(t.type) for t in reader.get_all_topics_and_types()}
    count=0
    while reader.has_next():
        topic,data,record_ns=reader.read_next()
        message=deserialize_message(data,types[topic]); count+=1
        if topic=='/joint_states':
            print(record_ns,dict(zip(message.name,message.position)))
    print('Messages read:',count)


if __name__=='__main__': main()
