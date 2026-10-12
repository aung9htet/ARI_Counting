"""Single command owner with collision checks, recording and auditable events.

Wire protocol: JSON requests/results on /experiment/commands and /experiment/results.
Images remain native ROS Image messages; they never pass through this protocol.
"""
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import uuid
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK, GetMotionPlan, GetStateValidity
from moveit_msgs.msg import RobotState, Constraints, JointConstraint, MoveItErrorCodes
from sensor_msgs.msg import JointState, Image, CameraInfo
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose_stamped
from ari_counting_sim.robot import joint_limits
from .validation import trajectory_samples, SimDeadline


CAMERAS = ['overhead','front','robot']


def ns(stamp):
    return stamp.sec*1_000_000_000+stamp.nanosec


def duration(value):
    n = round(value*1_000_000_000)
    return Duration(sec=n//1_000_000_000, nanosec=n%1_000_000_000)


class Gateway(Node):
    def __init__(self):
        super().__init__('experiment_gateway')
        for key, default in [('robot_description',''),('arm','right'),('scene_file',''),('output_dir','/data')]:
            self.declare_parameter(key, default)
        self.arm = self.get_parameter('arm').value
        self.limits = {s: joint_limits(self.get_parameter('robot_description').value, s) for s in ['left','right']}
        self.scene_path = Path(self.get_parameter('scene_file').value)
        self.scene = json.loads(self.scene_path.read_text())
        self.output = Path(self.get_parameter('output_dir').value)
        self.joints = None; self.images={}; self.infos={}; self.scene_ready=False
        self.lock = threading.Lock(); self.record_lock=threading.Lock(); self.cancel_event=threading.Event()
        self.active_goal=None; self.worker=None; self.recorder=None; self.episode=None; self.record_log=None
        self.seen = set()
        self.events=self.create_publisher(String,'/experiment/action_events',100)
        self.results=self.create_publisher(String,'/experiment/results',100)
        self.create_subscription(String,'/experiment/commands',self.command,100)
        self.create_subscription(JointState,'/joint_states',self.on_joints,qos_profile_sensor_data)
        self.create_subscription(Bool,'/experiment/scene_ready',self.on_scene,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        for name in CAMERAS:
            self.create_subscription(Image,f'/experiment/cameras/{name}/image_raw',lambda msg,n=name:self.images.__setitem__(n,msg),QoSProfile(depth=5))
            self.create_subscription(CameraInfo,f'/experiment/cameras/{name}/camera_info',lambda msg,n=name:self.infos.__setitem__(n,msg),QoSProfile(depth=5))
        self.arms={s:ActionClient(self,FollowJointTrajectory,f'/arm_{s}_controller/follow_joint_trajectory') for s in ['left','right']}
        self.validity=self.create_client(GetStateValidity,'/check_state_validity')
        self.ik=self.create_client(GetPositionIK,'/compute_ik')
        self.plan=self.create_client(GetMotionPlan,'/plan_kinematic_path')
        self.tf=Buffer(); self.listener=TransformListener(self.tf,self)
        self.get_logger().info('Experiment gateway ready; waiting for simulation data')

    def on_joints(self,msg): self.joints=msg
    def on_scene(self,msg): self.scene_ready=msg.data

    def emit(self,command,event,**extra):
        data={'schema_version':1,'stamp_ns':self.get_clock().now().nanoseconds,
              'command_id':command.get('id',''), 'mode':command.get('operation',''),
              'arm':command.get('arm',self.arm),'event':event,**extra}
        self.events.publish(String(data=json.dumps(data,allow_nan=False)))

    def respond(self,request,success,result=None,error=None):
        identifier=request.get('id','') if isinstance(request,dict) else ''
        if not isinstance(identifier,str): identifier=''
        self.results.publish(String(data=json.dumps({'id':identifier,'success':success,'result':result,'error':error},allow_nan=False)))

    def command(self,msg):
        request={}
        try:
            request=json.loads(msg.data)
            if not isinstance(request,dict) or not isinstance(request.get('id'),str) or not request['id']:
                raise ValueError('request needs a nonempty string id')
            if request['id'] in self.seen:
                raise ValueError('duplicate command id')
            self.seen.add(request['id'])
            op=request.get('operation')
            if op=='status':
                self.respond(request,True,self.status()); return
            if op=='cancel':
                self.cancel_event.set()
                self.emit(request,'cancel_requested')
                threading.Thread(target=self.cancel, args=(request,),daemon=True).start(); return
            if op in ['start_recording','stop_recording']:
                threading.Thread(target=self.record_command,args=(request,),daemon=True).start(); return
            if op not in ['joints','xyz']:
                raise ValueError('operation must be joints, xyz, cancel, status, start_recording or stop_recording')
            if not self.lock.acquire(blocking=False):
                raise RuntimeError('another arm command is active')
            self.cancel_event.clear()
            self.worker=threading.Thread(target=self.execute,args=(request,),daemon=True)
            self.worker.start()
        except Exception as exc:
            if isinstance(request,dict): self.emit(request,'rejected',error=str(exc))
            self.respond(request,False,error=str(exc))

    def status(self):
        now=self.get_clock().now().nanoseconds
        def fresh(msg, max_age=100_000_000): return msg is not None and -50_000_000 <= now-ns(msg.header.stamp) <= max_age
        ready = (self.scene_ready and fresh(self.joints) and all(fresh(self.images.get(n),500_000_000) and n in self.infos for n in CAMERAS)
                 and all(a.server_is_ready() for a in self.arms.values()) and self.validity.service_is_ready() and self.ik.service_is_ready() and self.plan.service_is_ready())
        return {'ready':bool(ready),'scene_ready':self.scene_ready,'recording':self.recorder is not None,
                'joint_fresh':fresh(self.joints), 'camera_fresh':{n:fresh(self.images.get(n),500_000_000) for n in CAMERAS},
                'camera_stamp_ns': {n:ns(self.images[n].header.stamp) if n in self.images else None for n in CAMERAS},
                'camera_info_received': {n:n in self.infos for n in CAMERAS},
                'episode':str(self.episode) if self.episode else None,'active':self.lock.locked(),
                'stamp_ns':now,'limits':self.limits}

    def wait(self,future,timeout=10,interruptible=True):
        deadline=time.monotonic()+timeout
        while not future.done():
            if interruptible and self.cancel_event.is_set(): raise RuntimeError('command cancelled')
            if time.monotonic()>deadline: raise TimeoutError('ROS response timed out')
            time.sleep(0.01)
        return future.result()

    def wait_sim(self,future,timeout):
        # Trajectories run on the simulation clock, which can be slower than wall time.
        deadline=SimDeadline(timeout)
        while not future.done():
            if deadline.expired(self.get_clock().now().nanoseconds): raise TimeoutError('controller result timed out')
            time.sleep(0.01)
        return future.result()

    def call(self,client,request,timeout=10):
        if not client.wait_for_service(timeout_sec=2): raise RuntimeError(f'{client.srv_name} unavailable')
        return self.wait(client.call_async(request),timeout)

    def state(self):
        if self.joints is None: raise RuntimeError('joint states unavailable')
        state=RobotState(); state.joint_state=copy.deepcopy(self.joints)
        return state

    def positions(self,side):
        state=self.state()
        values=dict(zip(state.joint_state.name,state.joint_state.position))
        return [values[j['name']] for j in self.limits[side]]

    def valid(self,state,side):
        request=GetStateValidity.Request(); request.robot_state=state; request.group_name=f'arm_{side}'
        result=self.call(self.validity,request)
        if not result.valid:
            contacts=[f'{c.contact_body_1}/{c.contact_body_2}' for c in result.contacts]
            raise ValueError('trajectory is in collision: '+', '.join(contacts))

    def joint_trajectory(self,request,side):
        start=self.positions(side)
        points=request.get('waypoints') or [{'positions':request['positions'],'time':float(request.get('duration',2.0))}]
        samples=trajectory_samples(start,points,self.limits[side])
        state=self.state(); names=[j['name'] for j in self.limits[side]]
        indexes=[state.joint_state.name.index(n) for n in names]
        for sample in samples:
            for index,value in zip(indexes,sample): state.joint_state.position[index]=value
            self.valid(state,side)
        trajectory=JointTrajectory(); trajectory.joint_names=names
        # Dense samples enforce the same smooth path that was collision-checked.
        previous=start; previous_t=0.0
        initial=JointTrajectoryPoint(); initial.positions=start; initial.velocities=[0.0]*5; initial.accelerations=[0.0]*5; initial.time_from_start=duration(0.0)
        trajectory.points.append(initial)
        for point in points:
            total=float(point['time']); dt=total-previous_t
            count=max(2,math.ceil(dt/0.05))
            for i in range(1,count+1):
                u=i/count; s=10*u**3-15*u**4+6*u**5
                ds=(30*u**2-60*u**3+30*u**4)/dt
                dds=(60*u-180*u**2+120*u**3)/(dt*dt)
                p=JointTrajectoryPoint()
                p.positions=[a+(b-a)*s for a,b in zip(previous,point['positions'])]
                p.velocities=[(b-a)*ds for a,b in zip(previous,point['positions'])]
                p.accelerations=[(b-a)*dds for a,b in zip(previous,point['positions'])]
                p.time_from_start=duration(previous_t+dt*u); trajectory.points.append(p)
            previous=list(point['positions']); previous_t=total
        return trajectory

    def xyz_trajectory(self,request,side):
        xyz=request.get('position')
        if not isinstance(xyz,list) or len(xyz)!=3 or any(not isinstance(x,(int,float)) or not math.isfinite(x) for x in xyz):
            raise ValueError('position must contain three finite XYZ coordinates')
        pose=PoseStamped(); pose.header.frame_id=request.get('frame','base_link'); pose.pose.orientation.w=1.0
        pose.pose.position.x,pose.pose.position.y,pose.pose.position.z=map(float,xyz)
        if pose.header.frame_id!='base_link':
            transform=self.tf.lookup_transform('base_link',pose.header.frame_id,rclpy.time.Time(),timeout=rclpy.duration.Duration(seconds=2))
            pose=do_transform_pose_stamped(pose,transform)
        req=GetPositionIK.Request(); req.ik_request.group_name=f'arm_{side}'; req.ik_request.robot_state=self.state()
        req.ik_request.ik_link_name=f'arm_{side}_hand_link'; req.ik_request.pose_stamped=pose
        req.ik_request.avoid_collisions=True; req.ik_request.timeout=duration(2)
        result=self.call(self.ik,req)
        if result.error_code.val!=MoveItErrorCodes.SUCCESS: raise ValueError(f'IK failed ({result.error_code.val}); target may be unreachable or in collision')
        plan=GetMotionPlan.Request(); plan.motion_plan_request.group_name=f'arm_{side}'
        plan.motion_plan_request.start_state=self.state(); plan.motion_plan_request.allowed_planning_time=5.0
        plan.motion_plan_request.num_planning_attempts=3; plan.motion_plan_request.max_velocity_scaling_factor=0.25
        plan.motion_plan_request.max_acceleration_scaling_factor=0.25
        names=[j['name'] for j in self.limits[side]]
        constraints=Constraints()
        for name in names:
            index=result.solution.joint_state.name.index(name)
            c=JointConstraint(); c.joint_name=name; c.position=result.solution.joint_state.position[index]
            c.tolerance_above=0.001; c.tolerance_below=0.001; c.weight=1.0
            constraints.joint_constraints.append(c)
        plan.motion_plan_request.goal_constraints=[constraints]
        planned=self.call(self.plan,plan,timeout=15).motion_plan_response
        if planned.error_code.val!=MoveItErrorCodes.SUCCESS: raise ValueError(f'motion planning failed ({planned.error_code.val})')
        return planned.trajectory.joint_trajectory

    def execute(self,request):
        try:
            self.emit(request,'requested',request=request)
            side=request.get('arm',self.arm)
            if side not in self.arms: raise ValueError('arm must be left or right')
            if not self.status()['ready']: raise RuntimeError('experiment not ready or observation data stale')
            trajectory=self.joint_trajectory(request,side) if request['operation']=='joints' else self.xyz_trajectory(request,side)
            if self.cancel_event.is_set(): raise RuntimeError('command cancelled')
            self.emit(request,'planned',trajectory=message_to_ordereddict(trajectory))
            goal=FollowJointTrajectory.Goal(); goal.trajectory=trajectory
            self.active_goal=self.wait(self.arms[side].send_goal_async(goal),interruptible=False)
            if not self.active_goal.accepted: raise RuntimeError('controller rejected trajectory')
            if self.cancel_event.is_set(): self.wait(self.active_goal.cancel_goal_async(),interruptible=False)
            self.emit(request,'accepted')
            result_future=self.active_goal.get_result_async()
            timeout=max(10,ns(trajectory.points[-1].time_from_start)/1e9+10)
            try:
                result=self.wait_sim(result_future,timeout)
            except TimeoutError:
                self.wait(self.active_goal.cancel_goal_async(),interruptible=False)
                self.wait(result_future,10,interruptible=False)
                raise
            if self.cancel_event.is_set(): raise RuntimeError('command cancelled; controller holding current position')
            if result.result.error_code!=0: raise RuntimeError(f'controller failure: {result.result.error_string} ({result.result.error_code})')
            measured=self.positions(side)
            self.emit(request,'completed',measured_positions=measured)
            self.respond(request,True,{'positions':measured})
        except Exception as exc:
            self.emit(request,'failed',error=str(exc)); self.respond(request,False,error=str(exc))
        finally:
            self.active_goal=None; self.lock.release()

    def cancel(self,request):
        try:
            goal=self.active_goal
            if goal and goal.accepted:
                result=self.wait(goal.cancel_goal_async(),interruptible=False)
                if not result.goals_canceling: raise RuntimeError('controller did not accept cancellation')
            self.respond(request,True,{'cancel_requested':True})
        except Exception as exc: self.respond(request,False,error=str(exc))

    def record_command(self,request):
        if not self.record_lock.acquire(blocking=False):
            self.respond(request,False,error='another recording operation is active'); return
        try:
            if request['operation']=='start_recording': result=self.start_recording(request)
            else: result=self.stop_recording()
            self.respond(request,True,result)
        except Exception as exc: self.respond(request,False,error=str(exc))
        finally: self.record_lock.release()

    def start_recording(self,request):
        if self.recorder is not None: raise RuntimeError('recording already active')
        if not self.status()['ready']: raise RuntimeError('experiment not ready or observation data stale')
        self.episode=self.output/'episodes'/uuid.uuid4().hex; self.episode.mkdir(parents=True)
        import shutil
        for name in ['scene.json','counting.world','robot.urdf','robot.srdf','controllers.yaml']:
            source=self.scene_path.parent/name
            if source.exists(): shutil.copy2(source,self.episode/name)
        revisions={}
        manifest=Path('/opt/ari/dependencies.repos')
        if manifest.exists():
            import yaml
            revisions=yaml.safe_load(manifest.read_text())
        import ari_counting_sim
        digest=hashlib.sha256()
        for package in [Path(__file__).parent,Path(ari_counting_sim.__file__).parent]:
            for source in sorted(package.glob('*.py')):
                digest.update(source.name.encode()); digest.update(source.read_bytes())
        metadata={'schema_version':1,'start_ns':self.get_clock().now().nanoseconds,'arm':request.get('arm',self.arm),
                  'instruction':request.get('instruction',''),'robot_model':'v2','ros_distro':os.environ.get('ROS_DISTRO'),
                  'scene_id':self.scene['scene_id'],'dependencies':revisions,'camera_settings':self.scene['cameras'],
                  'experiment_source_sha256':digest.hexdigest()}
        (self.episode/'episode.json').write_text(json.dumps(metadata,indent=2))
        topics=['/joint_states','/tf','/tf_static','/clock','/experiment/action_events','/experiment/scene','/experiment/scene_ready',
                '/arm_left_controller/controller_state','/arm_right_controller/controller_state']
        topics += [f'/experiment/cameras/{n}/{suffix}' for n in CAMERAS for suffix in ['image_raw','camera_info']]
        # Explicit reliable+transient_local TF static QoS captures publishers started before recorder.
        import yaml
        qos={'/tf_static':{'history':'keep_last','depth':100,'reliability':'reliable','durability':'transient_local'},
             '/experiment/scene':{'history':'keep_last','depth':1,'reliability':'reliable','durability':'transient_local'},
             '/experiment/scene_ready':{'history':'keep_last','depth':1,'reliability':'reliable','durability':'transient_local'}}
        for topic in topics:
            if topic.startswith('/experiment/cameras/'):
                qos[topic]={'history':'keep_last','depth':20,'reliability':'reliable','durability':'volatile'}
        qos_path=self.episode/'qos.yaml'; qos_path.write_text(yaml.safe_dump(qos))
        self.record_log=(self.episode/'recorder.log').open('w')
        self.recorder=subprocess.Popen(['ros2','bag','record','--use-sim-time','--qos-profile-overrides-path',str(qos_path),'-o',str(self.episode/'bag'),*topics],stdout=self.record_log,stderr=subprocess.STDOUT,start_new_session=True)
        deadline=time.monotonic()+20
        try:
            while time.monotonic()<deadline:
                if self.recorder.poll() is not None: raise RuntimeError('rosbag recorder exited; inspect recorder.log')
                if all(any(info.node_name=='rosbag2_recorder' for info in self.get_subscriptions_info_by_topic(topic)) for topic in ['/joint_states','/experiment/action_events']+[f'/experiment/cameras/{n}/image_raw' for n in CAMERAS]):
                    self.emit(request,'recording_started',episode=str(self.episode)); return {'episode':str(self.episode)}
                time.sleep(0.1)
            raise TimeoutError('recorder subscriptions did not become ready')
        except Exception:
            self.stop_recording(); raise

    def stop_recording(self,reason='requested'):
        if self.recorder is None: raise RuntimeError('no recording is active')
        process=self.recorder
        if process.poll() is None: os.killpg(process.pid,signal.SIGINT)
        try: process.wait(timeout=25)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid,signal.SIGTERM); process.wait(timeout=5)
        self.recorder=None
        if self.record_log: self.record_log.close(); self.record_log=None
        path=self.episode/'episode.json'; metadata=json.loads(path.read_text())
        metadata.update({'end_ns':self.get_clock().now().nanoseconds,'recorder_exit_code':process.returncode,
                         'bag_finalized':(self.episode/'bag/metadata.yaml').exists(),
                         'end_reason':reason,'motion_interrupted':self.lock.locked()})
        path.write_text(json.dumps(metadata,indent=2))
        if not metadata['bag_finalized']:
            raise RuntimeError('bag was not finalized; inspect recorder.log')
        return {'episode':str(self.episode),'bag_finalized':metadata['bag_finalized']}

    def close(self):
        self.cancel_event.set()
        if self.active_goal:
            try: self.active_goal.cancel_goal_async()
            except Exception: pass  # ROS may already be shutting down.
        if self.recorder:
            try: self.stop_recording(reason='experiment_shutdown')
            except Exception as exc: self.get_logger().error(f'Recording shutdown failed: {exc}')


def main():
    rclpy.init(); node=Gateway()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.close(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
