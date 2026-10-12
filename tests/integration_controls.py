"""Run explicitly against the simulator; exercises real ROS services/actions."""
import argparse
import copy
import json
from pathlib import Path
import threading
import time
import math
import cv2
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import RobotState, CollisionObject, PlanningScene
from moveit_msgs.srv import GetPositionFK, GetStateValidity, ApplyPlanningScene
from ari_counting_tools.client import ExperimentClient


def call(client,service,request):
    assert service.wait_for_service(timeout_sec=5)
    future=service.call_async(request)
    deadline=time.monotonic()+15
    while not future.done():
        assert time.monotonic()<deadline,'service timeout'
        time.sleep(0.01)
    return future.result()


def measured(client,names):
    msg=client.get_observation()['joint_state']
    assert msg is not None
    values=dict(zip(msg.name,msg.position))
    return [values[name] for name in names]


def fk(client,state,link=None,frame="base_link"):
    service=client.node.create_client(GetPositionFK,'/compute_fk')
    req=GetPositionFK.Request(); req.header.frame_id=frame
    req.fk_link_names=[link or f'arm_{client.arm}_hand_link']; req.robot_state=state
    result=call(client,service,req); assert result.error_code.val==1
    client.node.destroy_client(service)
    p=result.pose_stamped[0].pose.position
    return [p.x,p.y,p.z]


def check_collision(client,names):
    state=RobotState(); state.joint_state=copy.deepcopy(client.get_observation()['joint_state'])
    center=fk(client,state,link='arm_right_5_link')
    service=client.node.create_client(ApplyPlanningScene,'/apply_planning_scene')
    obj=CollisionObject(); obj.id='validation_obstacle'; obj.header.frame_id='base_link'; obj.operation=CollisionObject.ADD
    sphere=SolidPrimitive(); sphere.type=SolidPrimitive.SPHERE; sphere.dimensions=[0.15]
    pose=Pose(); pose.orientation.w=1.0; pose.position.x,pose.position.y,pose.position.z=center
    obj.primitives=[sphere]; obj.primitive_poses=[pose]
    req=ApplyPlanningScene.Request(); req.scene=PlanningScene(); req.scene.is_diff=True; req.scene.world.collision_objects=[obj]
    assert call(client,service,req).success
    try:
        before=measured(client,names); target=list(before); target[0]+=0.05
        try: client.command_joints(target,3)
        except RuntimeError as exc: assert 'collision' in str(exc),str(exc)
        else: raise AssertionError('colliding joint trajectory accepted')
        assert max(abs(a-b) for a,b in zip(before,measured(client,names)))<0.01
        print('PASS: temporary MoveIt obstacle rejected without movement',flush=True)
    finally:
        obj.operation=CollisionObject.REMOVE
        assert call(client,service,req).success
        client.node.destroy_client(service)


def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--collision-only',action='store_true')
    parser.add_argument('--save-previews',type=Path,help='directory for the three current camera views')
    args=parser.parse_args()
    with ExperimentClient() as client:
        client.wait_ready()
        status=client.status(); assert not status['active']
        limits=status['limits']['right']; names=[limit['name'] for limit in limits]
        baseline=measured(client,names)
        if args.collision_only:
            check_collision(client,names); return
        assert all(limit['lower']<=q<=limit['upper'] for limit,q in zip(limits,baseline))
        observation=client.get_observation()
        assert all(view['valid'] for view in observation['cameras'].values())
        if args.save_previews:
            args.save_previews.mkdir(exist_ok=True,parents=True)
            for name,view in observation['cameras'].items():
                cv2.imwrite(str(args.save_previews/f'{name}.png'),cv2.cvtColor(view['image'],cv2.COLOR_RGB2BGR))
            print('PASS: saved current camera views to',args.save_previews,flush=True)
        episode=client.start_recording('IK integration validation')
        try:
            target=list(baseline); target[0]+=0.1
            client.command_joints(target,2)
            state=RobotState(); state.joint_state=copy.deepcopy(client.get_observation()['joint_state'])
            position=fk(client,state,frame="world")
            client.command_joints(baseline,2)
            result=client.move_xyz(*position,frame="world")
            state.joint_state=copy.deepcopy(client.get_observation()['joint_state'])
            actual=fk(client,state,frame="world"); error=math.dist(position,actual)
            assert error<0.01,(position,actual,error)
            print('PASS: reachable position-only IK; position error metres:',error,flush=True)
            client.command_joints(baseline,3)
            errors=[]
            target=list(baseline); target[0]+=0.25
            def long_command():
                try: client.command_joints(target,6)
                except RuntimeError as exc: errors.append(str(exc))
            worker=threading.Thread(target=long_command); worker.start()
            deadline=time.monotonic()+20
            while abs(measured(client,names)[0]-baseline[0])<0.01:
                assert time.monotonic()<deadline,'motion did not start'
                time.sleep(0.1)
            try: client.command_joints(baseline,2)
            except RuntimeError as exc: assert 'active' in str(exc)
            else: raise AssertionError('concurrent command accepted')
            client.cancel(); worker.join(timeout=15)
            assert not worker.is_alive() and errors and 'cancelled' in errors[0]
            held=measured(client,names); time.sleep(0.5)
            assert max(abs(a-b) for a,b in zip(held,measured(client,names)))<0.01
            print('PASS: concurrent rejection, active cancellation and hold',flush=True)
            client.command_joints(baseline,3)
            check_collision(client,names)
        finally:
            print('Recording finalized:',client.stop_recording(),flush=True)
        print('IK_EPISODE='+episode['episode'],flush=True)


if __name__=='__main__': main()
