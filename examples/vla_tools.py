"""Model-free integration skeleton: observations in, explicit tools out."""
from ari_counting_tools.client import ExperimentClient


def main():
    with ExperimentClient(arm='right') as robot:
        robot.wait_ready()
        episode=robot.start_recording('Observe the coloured balls on the table.')
        try:
            observation=robot.get_observation()
            assert observation['joint_valid']
            print({name:item['image'].shape if item['valid'] else None for name,item in observation['cameras'].items()})
            # Future VLA code consumes observation and chooses tool calls here.
            # robot.command_joints([q1, q2, q3, q4, q5], duration=2.0)
            # robot.move_xyz(x, y, z, frame='base_link')
            # Ground-truth ball counts intentionally are not in observation.
        finally:
            robot.stop_recording()
        print(episode)


if __name__=='__main__': main()
