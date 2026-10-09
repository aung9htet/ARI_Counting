"""Model-free Python observation/control API for the experiment gateway."""
import copy
from collections import deque
import json
import threading
import time
import uuid
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from sensor_msgs.msg import Image, JointState, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
from .gateway import CAMERAS, ns


class ExperimentClient:
    def __init__(self, arm='right'):
        if arm not in ['left','right']: raise ValueError('arm must be left or right')
        self.arm=arm
        self.context=Context(); rclpy.init(context=self.context)
        self.node=Node('experiment_client_'+uuid.uuid4().hex[:8],context=self.context,parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)])
        self.condition=threading.Condition(); self.replies={}; self.pending_ids=set(); self.images={}; self.infos={}; self.joints=None
        self.joint_history=deque(maxlen=50); self.image_history={n:deque(maxlen=20) for n in CAMERAS}
        self.publisher=self.node.create_publisher(String,'/experiment/commands',100)
        self.node.create_subscription(String,'/experiment/results',self._result,100)
        self.node.create_subscription(JointState,'/joint_states',self._joints,qos_profile_sensor_data)
        for name in CAMERAS:
            self.node.create_subscription(Image,f'/experiment/cameras/{name}/image_raw',lambda msg,n=name:self._image(n,msg),QoSProfile(depth=5))
            self.node.create_subscription(CameraInfo,f'/experiment/cameras/{name}/camera_info',lambda msg,n=name:self._info(n,msg),QoSProfile(depth=5))
        self.bridge=CvBridge(); self.executor=SingleThreadedExecutor(context=self.context); self.executor.add_node(self.node)
        self.thread=threading.Thread(target=self.executor.spin,daemon=True); self.thread.start()

    def _result(self,msg):
        data=json.loads(msg.data)
        with self.condition:
            if data['id'] in self.pending_ids:
                self.replies[data['id']]=data; self.condition.notify_all()
    def _joints(self,msg):
        with self.condition:
            self.joints=msg; self.joint_history.append(msg)
    def _image(self,name,msg):
        with self.condition:
            self.images[name]=msg; self.image_history[name].append(msg)
    def _info(self,name,msg):
        with self.condition: self.infos[name]=msg

    def request(self,operation,timeout=60,**arguments):
        deadline=time.monotonic()+timeout
        while not self.publisher.get_subscription_count():
            if time.monotonic()>deadline: raise TimeoutError('experiment gateway not found; check ROS_DOMAIN_ID and launch')
            time.sleep(0.05)
        identifier=uuid.uuid4().hex
        payload=json.dumps({'id':identifier,'operation':operation,'arm':self.arm,**arguments},allow_nan=False)
        with self.condition: self.pending_ids.add(identifier)
        self.publisher.publish(String(data=payload))
        with self.condition:
            while identifier not in self.replies:
                remaining=deadline-time.monotonic()
                if remaining<=0:
                    self.pending_ids.discard(identifier)
                    raise TimeoutError(f'{operation} timed out; use cancel() before another motion')
                self.condition.wait(min(remaining,0.1))
            result=self.replies.pop(identifier)
            self.pending_ids.discard(identifier)
        if not result['success']: raise RuntimeError(result['error'])
        return result['result']

    def status(self): return self.request('status',timeout=5)

    def wait_ready(self,timeout=120):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            if self.status()['ready']:
                observation=self.get_observation()
                if observation['joint_valid'] and all(view['valid'] and view['camera_info'] is not None for view in observation['cameras'].values()): return
            time.sleep(0.2)
        raise TimeoutError('simulation did not become ready; inspect launch output')

    def get_observation(self,max_age_ms=100):
        query_ns=self.node.get_clock().now().nanoseconds
        with self.condition:
            # Align live input to the most recent common capture time. Rendering
            # can lag physics; expose that latency instead of mixing time steps.
            times=[query_ns]+[ns(history[-1].header.stamp) for history in self.image_history.values() if history]
            now=min(times)
        def latest(history):
            return next((msg for msg in reversed(history) if ns(msg.header.stamp)<=now),None)
        with self.condition:
            joints=copy.deepcopy(latest(self.joint_history))
            images={name:latest(history) for name,history in self.image_history.items()}; infos=self.infos.copy()
        fresh=lambda msg:msg is not None and 0<=now-ns(msg.header.stamp)<=max_age_ms*1_000_000
        cameras={}
        for name in CAMERAS:
            msg=images.get(name); valid=fresh(msg)
            cameras[name]={'image':self.bridge.imgmsg_to_cv2(msg,'rgb8').copy() if valid else None,
                           'stamp_ns':ns(msg.header.stamp) if msg else None,'valid':valid,
                           'camera_info':infos.get(name)}
        return {'stamp_ns':now,'query_stamp_ns':query_ns,'latency_ns':query_ns-now,
                'joint_state':joints if fresh(joints) else None,
                'joint_stamp_ns':ns(joints.header.stamp) if joints else None,
                'joint_valid':fresh(joints),'cameras':cameras}

    def command_joints(self,positions,duration=2.0):
        return self.request('joints',positions=[float(q) for q in positions],duration=float(duration),timeout=max(60,duration+30))

    def command_trajectory(self,waypoints):
        if not waypoints: raise ValueError('at least one waypoint is required')
        points=[{'positions':[float(q) for q in point['positions']],'time':float(point['time'])} for point in waypoints]
        return self.request('joints',waypoints=points,timeout=max(60,points[-1]['time']+30))

    def move_xyz(self,x,y,z,frame='base_link'):
        return self.request('xyz',position=[float(x),float(y),float(z)],frame=frame,timeout=90)

    def cancel(self): return self.request('cancel',timeout=15)
    def start_recording(self,instruction=''): return self.request('start_recording',instruction=instruction,timeout=30)
    def stop_recording(self): return self.request('stop_recording',timeout=35)

    def close(self):
        self.executor.shutdown(timeout_sec=2)
        self.thread.join(timeout=2); self.node.destroy_node(); self.context.shutdown()
    def __enter__(self): return self
    def __exit__(self,*args): self.close()
