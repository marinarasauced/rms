
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
    dx = offset.x - viewpoint.position.x
    dy = offset.y - viewpoint.position.y
    dz = offset.z - viewpoint.position.z
    roll = 0.0
    if dy != 0:
        roll = np.pi - np.arctan2(dy, dz)
    pitch = -np.arctan2(dz, np.sqrt(dx ** 2 + dy ** 2))
    yaw = np.arctan2(dy, dx)
    bot.arm.set_ee_pose_components(
        x=viewpoint.position.x,
        y=viewpoint.position.y,
        z=viewpoint.position.z,
        roll=roll,
        pitch=pitch,
        yaw=yaw
    )
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


def grip_vx_bot_at_viewpoint(bot, viewpoint, start="open", end="close"):
    """

    """
    if start == "open":
        bot.gripper.open()
    elif start == "close":
        pass
    bot.gripper.open()
    bot.arm.set_ee_pose_components(
        x=viewpoint.position.x,
        y=viewpoint.position.y,
        z=viewpoint.position.z
    )
    if end == "close":
        bot.gripper.close()
    elif end == "open":
        bot.gripper.open()
