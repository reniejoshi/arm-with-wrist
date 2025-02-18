from __future__ import annotations

import time as T
from dataclasses import dataclass

import json

from wpimath.trajectory import (
    Trajectory as WPILibTrajectory,
    TrajectoryConfig,
    TrajectoryGenerator,
)
from wpimath.spline import Spline3
from wpimath.geometry import Translation2d

from inverse_kinematics import *
from utils import CartesianVec as P, WindmillVec as W, lerp

from loguru import logger

# Constants


@dataclass
class Error:
    time: float
    message: str


RENDERING_RESOLUTION = 100.0
VERIFICATION_RESOLUTION = 100.0
"""Samples per second of trajectory-time."""

# Constraints


@dataclass
class _TrajectoryConstraints:
    """Physical constraints for the trajectory."""

    # TODO: Cartesian constraints are not an ideal method of constraining
    #       the feasability of a trajectory due to a disconnect from the
    #       actual mechanism.

    # windmill_max_velocity: W
    # """The max (height, angle) velocity constraints of the windmill."""
    # windmill_max_acceleration: W
    # """The max (height, angle) acceleration constraints of the windmill."""


# Trajectory


class Trajectory:
    """A time-parameterized trajectory for a rotational arm on an elevator carriage."""

    dirty = False
    error: Error | None = None
    last_generation_time: float = 0

    constraints: TrajectoryConfig
    """Physical constraints for the trajectory."""

    start_tangent: Translation2d = Translation2d(0, 0.01)
    """A tangent indicating the direction and strength of the initial point."""
    end_tangent: Translation2d = Translation2d(0, 0.01)
    """A tangent indicating the direction and strength of the final point."""

    points: list[Translation2d] = []
    """The list of (x, y) pairs to generate the trajectory from."""

    orientations: list[tuple[float, ArmOrientation]] = []
    """The list of arm orientation transitions."""

    trajectory: WPILibTrajectory | None = None
    """The backing WPILib trajectory generated with quintic hermite splines via WPILib's TrajectoryGenerator."""

    unvalidated_trajectory: WPILibTrajectory | None = None
    """The backing WPILib trajectory without validation to allow displaying errors."""

    # Initialization

    def __init__(
        self,
        constraints: TrajectoryConfig,
    ):
        self.constraints = constraints

    def regenerate_trajectory(self) -> bool:
        """Regenerates the trajectory, required before any usage.

        Returns:
            bool: Whether or not the generation succeeded.
        """

        if len(self.points) < 2:
            self.dirty = False
            self.trajectory = None
            self.error = None
            return True

        if not self.dirty and self.is_valid():
            self.dirty = False
            self.error = None
            return True

        time = T.time()
        self.dirty = False

        start_vector = Spline3.ControlVector(
            (self.points[0].x, self.start_tangent.x),
            (self.points[0].y, self.start_tangent.y),
        )
        end_vector = Spline3.ControlVector(
            (self.points[-1].x, self.end_tangent.x),
            (self.points[-1].y, self.end_tangent.y),
        )

        self.unvalidated_trajectory = TrajectoryGenerator.generateTrajectory(
            start_vector, self.points[1:-1], end_vector, self.constraints
        )
        result = compute_arm_orientation_changes_and_validate(
            self.unvalidated_trajectory
        )
        if type(result) == Error:
            logger.debug(
                f"Failed to generate trajectory at {result.time} seconds because {result.message}."
            )
            self.error = result
            return False
        else:
            self.error = None

        self.last_generation_time = T.time() - time

        self.orientations = result
        self.trajectory = self.unvalidated_trajectory

        return True

    # Setters

    def add_point(self, position: P, index: int | None = None) -> int:
        """Adds a new point to the trajectory.

        Args:
            position (CartesianVec): The position of the point
            angle_degrees (float): The angle of the point
            index (int | None, optional): An optional index to insert the point at. Defaults to None.

        Returns:
            int: The index of the new point
        """

        self.dirty = True

        if len(self.points) == 0:
            self.points.append(position.to_translation_2d())
            return 0

        distances = self.compute_distances_to_points(position)
        closest_point = min(distances, key=distances.get)

        position = position.to_translation_2d()

        distance_to_previous = (
            position.distance(self.points[closest_point - 1])
            if closest_point > 0
            else 1e99
        )
        distance_to_next = (
            position.distance(self.points[closest_point + 1])
            if closest_point == len(distances) - 2
            else 1e99
        )

        if distance_to_previous > distance_to_next:
            index = closest_point
        else:
            index = closest_point + 1

        self.points.insert(index, position)
        return index

    def delete_point(self, index: int):
        # Preconditions
        assert 0 <= index < len(self.points)

        self.dirty = True
        del self.points[index]

    def set_point_position(self, index: int, position: P):
        """Sets the position of the given pose.

        Args:
            index (int): The index of the point
            position (CartesianVec): The new position
        """

        # Preconditions
        assert 0 <= index < len(self.points)

        self.points[index] = Translation2d(position.x, position.y)

        self.dirty = True

    def set_start_tangent(self, tangent: Translation2d):
        """Sets the tangent of the initial point.

        Args:
            tangent (Translation2d): The tangent
        """

        # Preconditions
        assert len(self.points) > 0

        self.start_tangent = tangent
        self.dirty = True

    def set_end_tangent(self, tangent: Translation2d):
        """Sets the tangent of the final point.

        Args:
            tangent (Translation2d): The tangent
        """

        # Preconditions
        assert len(self.points) > 1

        self.end_tangent = tangent
        self.dirty = True

    # Getters

    def get_index_and_distance_to_closest_point_to_position(
        self, position: P
    ) -> tuple[int, float] | None:
        if len(self.points) == 0:
            return None

        distances = self.compute_distances_to_points(position)
        index = min(distances, key=distances.get)
        return (index, distances[index])

    def compute_distances_to_points(self, position: P) -> dict[int, float]:
        position = position.to_translation_2d()
        return {i: position.distance(p) for i, p in enumerate(self.points)}

    def discretize(self, resolution=RENDERING_RESOLUTION) -> list[list[float]]:
        """Discretize the unvalidated trajectory end effector positions."""

        if not self.trajectory:
            return []

        positions = []
        for time in range(0, int(resolution * self.get_duration()) + 1):
            time = float(time) / resolution
            position = self.sample_end_effector_position(time, bypass_validation=True)
            positions.append([position.x, position.y])

        return positions

    def sample_end_effector_position(
        self, time: float, bypass_validation: bool = False
    ) -> P:
        """Samples the position of the end effector at time.

        Args:
            time (seconds): The time to sample
            bypass_validation (bool): Whether to ignore preconditions and use the unvalidated trajectory.

        Returns:
            CartesianVec: The position of the end effector
        """

        # Preconditions
        if not bypass_validation:
            self.check_usage_requirements(time)

        # Sample the backing trajectory
        sample = (
            self.trajectory if not bypass_validation else self.unvalidated_trajectory
        ).sample(time)
        return P.from_pose(sample.pose)

    def sample_windmill_state(self, time: float) -> W | None:
        """Samples the state of the windmill at time.

        Args:
            time (seconds): The time to sample

        Returns:
            WindmillVec: The state of the windmill
        """

        # Preconditions
        self.check_usage_requirements(time)

        # Sample the position of the end effector and orientation of the arm
        end_effector_position = self.sample_end_effector_position(time)
        arm_orientation = self.sample_arm_orientation(time)

        valid, solution = inverse_kinematics(
            end_effector_position, arm_orientation
        ) or (False, None)
        if not valid:
            return None  # This method cannot be called with an invalid trajectory, any trigger of this is a bug.

        return solution

    def sample_arm_orientation(self, time: float) -> ArmOrientation:
        """Retrieves the orientation of the arm at `time`.

        Args:
            time (seconds): The time to sample

        Returns:
            ArmOrientation: The orientation of the arm
        """

        # Preconditions
        self.check_usage_requirements(time)

        # Search for the indice of the orientation _past_ the time
        # and return the previous orientation, or the last orientation.
        for i in range(1, len(self.orientations) + 1):
            if i == len(self.orientations) or self.orientations[i][0] >= time:
                return self.orientations[i - 1][1]

    def get_error(self) -> Error | None:
        return self.error

    def get_duration(self) -> float:
        """The duration of the trajectory with the given constraints.

        Returns:
            seconds: The duration
        """

        # Preconditions
        self.check_usage_requirements()

        return self.trajectory.totalTime()

    def is_valid(self) -> bool:
        return not self.is_empty() and self.error == None and not self.dirty

    def is_empty(self) -> bool:
        return self.trajectory == None

    def get_generation_time(self) -> float:
        return self.last_generation_time

    # Serialization

    def serialize(self):
        return {
            "config": {
                "max_velocity": self.constraints.maxVelocity(),
                "max_acceleration": self.constraints.maxAcceleration(),
            },
            "orientations": [(t, o.name) for t, o in self.orientations],
            "start_tangent": [self.start_tangent.x, self.start_tangent.y],
            "end_tangent": [self.end_tangent.x, self.end_tangent.y],
            "points": [(p.x, p.y) for p in self.points],
            "duration": self.get_duration(),
        }

    def deserialize(data: dict) -> Trajectory:
        logger.debug(f"Deserializing trajectory from {data}")

        trajectory = Trajectory(
            TrajectoryConfig(
                data["config"]["max_velocity"], data["config"]["max_acceleration"]
            )
        )
        trajectory.orientations = [
            (t, ArmOrientation[o]) for t, o in data["orientations"]
        ]
        trajectory.start_tangent = Translation2d(*data["start_tangent"])
        trajectory.end_tangent = Translation2d(*data["end_tangent"])
        trajectory.points = [Translation2d(*p) for p in data["points"]]

        trajectory.dirty = True

        return trajectory

    # Helpers

    def check_usage_requirements(self, time: float | None = None):
        assert not self.dirty, "attempted to sample a dirty trajectory"
        assert self.trajectory, "attempted to sample without a valid trajectory"
        if time:
            assert (
                0 <= time <= self.get_duration()
            ), "attempted to sample outside of the duration of the trajectory"


# Validation


def compute_arm_orientation_changes_and_validate(
    trajectory: WPILibTrajectory,
) -> list[tuple[float, ArmOrientation]] | Error:
    """Computes the points at which the trajectory needs to switch IK solutions.

    Includes the initial direction.

    Returns:
        list[tuple[float, ArmOrientation]] | tuple[float, str]:
            If valid, a list corresponding to periods of the trajectory for the designated orientation,
            otherwise, the time at which the trajectory is first invalid.
    """

    # Solve for the initial direction
    initial_end_effector_position = P.from_pose(trajectory.sample(0).pose)
    initial_valid_solutions = valid_solutions_at_end_effector_position(
        initial_end_effector_position
    )

    solutions_count = len(initial_valid_solutions.keys())
    if solutions_count == 2:
        logger.warning(
            "Initial end effector position has both solutions as valid, please adjust until an override for automatic orientation is implemented."
        )
        del initial_valid_solutions[ArmOrientation.DOWN]
    elif solutions_count == 0:
        logger.error("No solutions exist!")
        return Error(0.0, "")

    if ArmOrientation.UP in initial_valid_solutions:
        initial_orientation = ArmOrientation.UP
    else:
        initial_orientation = ArmOrientation.DOWN

    # Validate points along the trajectory & find orientation changes

    # Due to us not having a closed-form solution to the trajectory as it is represented currently,
    # we have to sample the trajectory over it's duration at some timestep.

    both_valid_switched = False

    orientations = [(0, initial_orientation)]
    previous_orientation = initial_orientation
    for time in range(0, int(VERIFICATION_RESOLUTION * trajectory.totalTime())):
        time = float(time) / VERIFICATION_RESOLUTION
        sample = trajectory.sample(time)
        end_effector_position = P.from_pose(sample.pose)

        # Compute the solutions for the position
        solutions = valid_solutions_at_end_effector_position(end_effector_position)
        solutions_count = len(solutions.keys())
        if solutions_count == 0:
            return Error(
                time,
                f"No valid solutions exist for end effector position {end_effector_position}!",
            )

        # Decide if we need to change directions
        can_switch = can_switch_orientations(sample.pose.rotation().radians())
        if can_switch:
            if solutions_count == 2 and not both_valid_switched:
                previous_orientation = previous_orientation.opposite()
                orientations.append((time, previous_orientation))
                both_valid_switched = True
            elif previous_orientation not in solutions:
                previous_orientation = previous_orientation.opposite()
                orientations.append((time, previous_orientation))
                both_valid_switched = False
        else:
            if previous_orientation not in solutions:
                return Error(
                    time, "Trajectory needs to switch directions but is not able to!"
                )

        windmill_state = solutions[previous_orientation]

        # Make sure the arm doesn't intersect with limits
        for u in range(0, int(VERIFICATION_RESOLUTION)):
            u = float(u) / VERIFICATION_RESOLUTION
            x = lerp(0, end_effector_position.x, u)
            y = lerp(windmill_state.height, end_effector_position.y, u)
            if not validate_end_effector_position(P(x, y)):
                return Error(time, "Arm intersects with the limits!")
    return orientations
