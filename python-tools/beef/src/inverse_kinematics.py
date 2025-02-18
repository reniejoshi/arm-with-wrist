import math
from enum import Enum

from utils import CartesianVec as P, WindmillVec as W
from constants import *

# Arm Orientation


class ArmOrientation(Enum):
    """The target orientation of the arm relative to the pivot when solving for inverse kinematics."""

    UP = 0
    DOWN = 1

    def opposite(self):
        return ArmOrientation.UP if self == ArmOrientation.DOWN else ArmOrientation.DOWN


# Inverse Kinematics


def inverse_kinematics(
    end_effector_position: P, arm_orientation: ArmOrientation
) -> tuple[bool, W] | None:
    """Computes the height and angle for the windmill given an end effector position and a target arm orientation (relative to the pivot).

    Args:
        end_effector_position (CartesianVec): The end effector's position
        arm_orientation (ArmOrientation): The target orientation of the arm

    Returns:
        tuple[bool, WindmillVec] | None: (Whether the solution is valid, the solution) or None if the `end_effector_position` is invalid.
    """

    if not validate_end_effector_position(end_effector_position):
        return None

    arm_angle = math.acos(end_effector_position.x / ARM_LENGTH)
    if arm_orientation == ArmOrientation.DOWN:
        arm_angle = -arm_angle
    height = end_effector_position.y - math.sin(arm_angle) * ARM_LENGTH

    state = W(height, arm_angle)
    return (validate_windmill_state(state), state)


def can_switch_orientations(end_effector_heading: float) -> bool:
    """Whether, given heading, the end effector is able to transition from it's previous solution to the other one.

    Requirements:
    - Arm is horizontal to the pivot.

    Args:
        end_effector_heading (float): The direction the end effector is moving (tangential to the pivot) in radians

    Returns:
        bool: Whether the direction can be changed
    """

    # Checks whether the heading of the end effector is vertical (meaning the arm is horizontal)
    result = math.fabs(math.sin(end_effector_heading)) > 0.99975
    return result


def valid_solutions_at_end_effector_position(
    end_effector_position: P,
) -> dict[ArmOrientation, W]:
    """Computes the valid solutions for the end effector position for both arm orientations.

    Args:
        end_effector_position (CartesianVec): The end effector's position

    Returns:
        dict[ArmOrientation, WindmillVec]: A dictionary of valid arm orientations to their solutions.
    """

    arm_up_valid, up_solution = inverse_kinematics(
        end_effector_position, ArmOrientation.UP
    ) or (False, None)
    arm_down_valid, down_solution = inverse_kinematics(
        end_effector_position, ArmOrientation.DOWN
    ) or (False, None)

    return ({ArmOrientation.UP: up_solution} if arm_up_valid else {}) | (
        {ArmOrientation.DOWN: down_solution} if arm_down_valid else {}
    )


# Validation


def validate_windmill_state(windmill_state: W) -> bool:
    """Validates whether the solution from inverse kinematics is physically possible.

    Args:
        windmill_state (WindmillVec): The inverse kinematics solution

    Returns:
        bool: Whether the solution is valid
    """

    return ELEVATOR_MIN_POSE <= windmill_state.height <= ELEVATOR_MAX_POSE


def validate_end_effector_position(end_effector_position: P) -> bool:
    """Validates whether the end effector position is physcially possible.

    Args:
        end_effector_position (CartesianVec): The end effector's position

    Returns:
        bool: Whether the end effector position is valid
    """

    x, y = end_effector_position.x, end_effector_position.y

    return (
        not (x > COLLECTOR_STOWED_MIN_X and y < COLLECTOR_HEIGHT)
        and (-ARM_LENGTH <= x <= ARM_LENGTH)
        and (
            END_EFFECTOR_MIN_POSE
            <= y
            <= (math.tan(math.acos(x / ARM_LENGTH)) * x + ELEVATOR_MAX_POSE)
        )
    )
