from pyniryo import *

robot = NiryoRobot("169.254.200.200")

try:
    robot.set_learning_mode(False)

    joints = robot.get_joints()

    print(
        "\n"
        f"joints = JointsPosition({joints[0]:.4f}, "
        f"{joints[1]:.4f}, "
        f"{joints[2]:.4f}, "
        f"{joints[3]:.4f}, "
        f"{joints[4]:.4f}, "
        f"{joints[5]:.4f})"
        "\n"
    )

finally:
    robot.close_connection()