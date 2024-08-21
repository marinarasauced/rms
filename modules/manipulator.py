
import numpy as np
import time

from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS # type: ignore


def get_vx_bot(robot_model):
    """
    Create and return an instance of the InterbotixManipulatorXS for a given robot model.

    This function initializes an InterbotixManipulatorXS object with the specified robot model,
    arm group name, and gripper name. It is used to set up the manipulator with the appropriate
    configuration for the given robot.

    Args:
        robot_model (str): The model name of the robot to be used. This should match one of the predefined models supported by the InterbotixManipulatorXS.

    Returns:
        InterbotixManipulatorXS: An instance of the InterbotixManipulatorXS configured with the specified robot model, arm group, and gripper name.

    Example:
        bot = get_vx_bot("vx300s")
        # This creates an InterbotixManipulatorXS instance for the "vx300s" robot model.
    """
    bot = InterbotixManipulatorXS(
        robot_model=robot_model,
        group_name="arm",
        gripper_name="gripper"
    )
    return bot


def move_vx_bot_to_viewpoint(bot, viewpoint, offset, wait_duration):
    """
    
    """
    tx = offset.x - viewpoint.position.x
    ty = offset.y - viewpoint.position.y
    tz = offset.z - viewpoint.position.z
    ry = np.arctan2(tz, np.sqrt(tx ** 2 + ty ** 2))
    bot.arm.set_ee_pose_components(x=tx, y=ty, z=tz, pitch=ry)
    tic = time.time()
    while time.time() < tic + wait_duration:
        pass


def move_vx_bot_to_home(bot):
    """
    
    """
    bot.arm.go_to_home_pose()


def move_vx_bot_to_sleep(bot):
    """
    
    """
    bot.arm.go_to_sleep_pose()