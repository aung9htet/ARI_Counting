"""Publish camera TF and apply exactly the generated collision scene to MoveIt."""
import json
import math
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import TransformStamped, Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool, String
from tf2_ros import StaticTransformBroadcaster


def quaternion(rpy):
    r,p,y = [v/2 for v in rpy]
    return [math.sin(r)*math.cos(p)*math.cos(y)-math.cos(r)*math.sin(p)*math.sin(y),
            math.cos(r)*math.sin(p)*math.cos(y)+math.sin(r)*math.cos(p)*math.sin(y),
            math.cos(r)*math.cos(p)*math.sin(y)-math.sin(r)*math.sin(p)*math.cos(y),
            math.cos(r)*math.cos(p)*math.cos(y)+math.sin(r)*math.sin(p)*math.sin(y)]


class SceneBridge(Node):
    def __init__(self):
        super().__init__('experiment_scene_bridge')
        self.declare_parameter('scene_file', '')
        self.scene = json.loads(Path(self.get_parameter('scene_file').value).read_text())
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.metadata = self.create_publisher(String, '/experiment/scene', qos)
        self.ready = self.create_publisher(Bool, '/experiment/scene_ready', qos)
        self.metadata.publish(String(data=json.dumps(self.scene)))
        self.ready.publish(Bool(data=False))
        self.broadcaster = StaticTransformBroadcaster(self)
        transforms = []
        for name in ['overhead','front']:
            c = self.scene['cameras'][name]
            for parent, target, xyz, rpy in [('world', f'{name}_camera_link', c['position'], c['rpy']),
                                            (f'{name}_camera_link', c['frame'], [0,0,0], [-math.pi/2,0,-math.pi/2])]:
                tf = TransformStamped(); tf.header.frame_id=parent; tf.child_frame_id=target
                tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, xyz)
                q = quaternion(rpy)
                tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = q
                transforms.append(tf)
        self.broadcaster.sendTransform(transforms)
        self.apply = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.pending = None
        self.timer = self.create_timer(0.5, self.tick)

    def tick(self):
        if self.pending:
            if self.pending.done():
                result = self.pending.result()
                self.pending = None
                if result and result.success:
                    self.ready.publish(Bool(data=True)); self.timer.cancel()
                    self.get_logger().info('Table and balls applied to MoveIt planning scene')
            return
        if not self.apply.service_is_ready():
            return
        scene = PlanningScene(); scene.is_diff=True
        for obj in self.scene['table_boxes'] + self.scene['objects']:
            collision = CollisionObject(); collision.header.frame_id='world'; collision.id=obj['id']; collision.operation=CollisionObject.ADD
            shape = SolidPrimitive()
            if 'radius' in obj:
                shape.type=SolidPrimitive.SPHERE; shape.dimensions=[obj['radius']]
            else:
                shape.type=SolidPrimitive.BOX; shape.dimensions=obj['size']
            pose = Pose(); pose.orientation.w=1.0
            pose.position.x, pose.position.y, pose.position.z = obj['position']
            collision.primitives=[shape]; collision.primitive_poses=[pose]
            scene.world.collision_objects.append(collision)
        request = ApplyPlanningScene.Request(); request.scene=scene
        self.pending = self.apply.call_async(request)


def main():
    rclpy.init(); node=SceneBridge()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
