"""Stream a ROS bag to videos, CSV, event JSONL and causal NPZ observations."""
import argparse
import csv
import json
from pathlib import Path
import cv2
import numpy as np
import rosbag2_py
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from .validation import past_indices


def stamp_ns(stamp): return stamp.sec*1_000_000_000+stamp.nanosec


def export_episode(episode,output=None,rate=20,max_age_ms=100):
    episode=Path(episode); output=Path(output) if output else episode/'export'
    if rate<=0 or max_age_ms<=0: raise ValueError('rate and max age must be positive')
    if output.exists(): raise FileExistsError(f'{output} already exists; choose a new output directory')
    metadata=json.loads((episode/'episode.json').read_text())
    if not metadata.get('bag_finalized'):
        raise ValueError('episode is not finalized; stop the recorder or recover the bag before exporting')
    scene=json.loads((episode/'scene.json').read_text())
    output.mkdir(parents=True)
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(episode/'bag'),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
    types={topic.name:get_message(topic.type) for topic in reader.get_all_topics_and_types()}
    bridge=CvBridge(); writers={}; timestamps={name:[] for name in scene['cameras']}
    frame_records={name:[] for name in scene['cameras']}; states=[]; events=[]; event_times=[]
    images={f'/experiment/cameras/{name}/image_raw':name for name in timestamps}
    calibration={}
    events_path=output/'actions.jsonl'
    try:
        with events_path.open('w') as event_file:
            while reader.has_next():
                topic,data,bag_ns=reader.read_next()
                if topic not in images and topic!='/joint_states' and topic!='/experiment/action_events' and not topic.endswith('/camera_info'): continue
                msg=deserialize_message(data,types[topic])
                if topic in images:
                    name=images[topic]; image=bridge.imgmsg_to_cv2(msg,'bgr8')
                    if name not in writers:
                        h,w=image.shape[:2]
                        writer=cv2.VideoWriter(str(output/f'{name}.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),scene['cameras'][name]['fps'],(w,h))
                        if not writer.isOpened(): raise RuntimeError(f'cannot open MP4 writer for {name}')
                        writers[name]=writer
                    writers[name].write(image)
                    timestamps[name].append(stamp_ns(msg.header.stamp)); frame_records[name].append(bag_ns)
                elif topic=='/joint_states':
                    states.append({'stamp_ns':stamp_ns(msg.header.stamp),'bag_ns':bag_ns,'names':list(msg.name),
                                   'position':list(msg.position),'velocity':list(msg.velocity),'effort':list(msg.effort)})
                elif topic=='/experiment/action_events':
                    event=json.loads(msg.data); event['bag_ns']=bag_ns
                    events.append(event); event_times.append(event['stamp_ns']); event_file.write(json.dumps(event)+'\n')
                else:
                    from rosidl_runtime_py.convert import message_to_ordereddict
                    calibration[topic]=message_to_ordereddict(msg)
    finally:
        for writer in writers.values(): writer.release()
    if not states or any(not values for values in timestamps.values()):
        raise ValueError('episode is missing joint states or a required camera stream')
    # Concurrent command/cancellation workers can publish near-simultaneous
    # events in a different order from timestamp acquisition. Joint/camera
    # timestamp checks still detect simulation resets independently.
    events.sort(key=lambda event:(event['stamp_ns'],event['bag_ns']))
    event_times=[event['stamp_ns'] for event in events]
    events_path.write_text(''.join(json.dumps(event)+'\n' for event in events))
    for name,values in timestamps.items():
        past_indices(values,[])
        with (output/f'{name}_frames.csv').open('w',newline='') as file:
            csvwriter=csv.writer(file); csvwriter.writerow(['frame_index','source_stamp_ns','bag_stamp_ns'])
            csvwriter.writerows((i,source,record) for i,(source,record) in enumerate(zip(values,frame_records[name])))
    names=sorted({name for state in states for name in state['names']})
    joint_times=np.array([state['stamp_ns'] for state in states],dtype=np.int64)
    matrices={key:np.full((len(states),len(names)),np.nan) for key in ['position','velocity','effort']}
    for row,state in enumerate(states):
        for index,name in enumerate(state['names']):
            for key,matrix in matrices.items():
                if index<len(state[key]): matrix[row,names.index(name)]=state[key][index]
    with (output/'joint_states.csv').open('w',newline='') as file:
        writer=csv.writer(file)
        writer.writerow(['source_stamp_ns','bag_stamp_ns']+[f'{name}/{key}' for key in matrices for name in names])
        for row,state in enumerate(states):
            writer.writerow([state['stamp_ns'],state['bag_ns']]+[value for matrix in matrices.values() for value in matrix[row]])
    first=min([int(joint_times[0])]+[values[0] for values in timestamps.values()])
    last=max([int(joint_times[-1])]+[values[-1] for values in timestamps.values()])
    target=np.arange(first,last+1,round(1_000_000_000/rate),dtype=np.int64)
    indices,valid=past_indices(joint_times,target,round(max_age_ms*1_000_000))
    arrays={'stamp_ns':target,'joint_names':np.array(names),'joint_valid':valid,
            'joint_source_stamp_ns':np.where(valid,joint_times[np.maximum(indices,0)],-1)}
    for key,matrix in matrices.items():
        aligned=matrix[np.maximum(indices,0)].copy(); aligned[~valid]=np.nan
        arrays['joint_'+key]=aligned; arrays['joint_'+key+'_valid']=np.isfinite(aligned)
    for name,values in timestamps.items():
        frame_index,frame_valid=past_indices(values,target,round(max_age_ms*1_000_000))
        arrays[name+'_frame_index']=frame_index; arrays[name+'_valid']=frame_valid
        arrays[name+'_source_stamp_ns']=np.where(frame_valid,np.array(values,dtype=np.int64)[np.maximum(frame_index,0)],-1)
    if event_times:
        events_index,events_valid=past_indices(event_times,target,max_age_ns=np.iinfo(np.int64).max)
    else:
        events_index=np.full(len(target),-1,dtype=np.int64); events_valid=np.zeros(len(target),dtype=bool)
    arrays['action_event_index']=events_index; arrays['action_event_valid']=events_valid
    np.savez_compressed(output/'observations.npz',**arrays)
    (output/'calibration.json').write_text(json.dumps(calibration,indent=2))
    metadata['export']={'observation_rate':rate,'max_age_ms':max_age_ms,'alignment':'latest sample at or before observation time',
                        'video_codec':'mp4v','video_timestamps':'*_frames.csv is authoritative; MP4 has constant playback FPS',
                        'camera_frames':{name:len(values) for name,values in timestamps.items()},'joint_samples':len(states),
                        'observations':len(target),'action_events':len(events)}
    (output/'metadata.json').write_text(json.dumps(metadata,indent=2))
    return metadata['export']


def main():
    p=argparse.ArgumentParser(description=__doc__); p.add_argument('episode',type=Path)
    p.add_argument('--output',type=Path); p.add_argument('--rate',type=float,default=20); p.add_argument('--max-age-ms',type=float,default=100)
    args=p.parse_args(); print(json.dumps(export_episode(args.episode,args.output,args.rate,args.max_age_ms),indent=2))
