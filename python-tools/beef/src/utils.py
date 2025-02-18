from __future__ import annotations

import time as Time, math
from dataclasses import dataclass

from wpimath.geometry import (
    Pose2d as WPILibPose2d,
    Translation2d as WPILibTranslation2d,
)
import dearpygui.dearpygui as dpg
from typing import Any

from constants import *

# Types


# TODO: Replace this with Translation2d
@dataclass
class CartesianVec:
    x: float
    """The x component of the position in meters."""
    y: float
    """The y component of the position in meters."""

    # Interop

    def from_pose(pose2d: WPILibPose2d):
        return CartesianVec(pose2d.x, pose2d.y)

    def to_list(self) -> list[float]:
        return [self.x, self.y]

    def to_translation_2d(self) -> WPILibTranslation2d:
        return WPILibTranslation2d(self.x, self.y)

    # Methods

    def constrain(self) -> CartesianVec:
        return CartesianVec(
            math.copysign(max(0.0, min(math.fabs(self.x), ARM_LENGTH)), self.x),
            max(self.y, END_EFFECTOR_MIN_POSE),
        )

    # Overrides

    def __str__(self):
        return f"({self.x:.3f}, {self.y:.3f})"


@dataclass
class WindmillVec:
    height: float
    """The height of the elevator in meters."""
    theta: float
    """The rotation of the arm in radians."""


# Bounded Timer


class BoundedTimer:
    """A timer whose value saturates at bounds."""

    # Member Variables

    current_time: float = 0.0
    """ The current time. """

    min_time: float = 0.0
    """ The minimum time. """
    max_time: float
    """ The maximum time. """

    last_rt_update_time: float | None = None
    """The last update time in seconds for real-time updates."""

    scale: float = 1

    # Initialization

    def __init__(
        self,
        max_time: float,
        current_time: float | None = None,
        min_time: float | None = None,
    ):
        self.min_time = min_time or 0.0
        self.max_time = max_time

        self.set(current_time or self.min_time)  # Sync the current_time or min_time

    # Setters

    def reset_rt(self):
        self.last_rt_update_time = Time.time()

    def advance_rt(self):
        """Updates the timer by the time since the last call to this method."""

        time = Time.time()
        if self.last_rt_update_time:
            dt = time - self.last_rt_update_time
            self.advance(dt)

        self.last_rt_update_time = time

    def advance(self, dt: float):
        """Advances the timer.

        :param dt: The delta time in seconds
        """

        self.set(self.current_time + dt * self.scale)

    def set(self, time: float):
        """Sets the time of the timer.

        :param time: The time in seconds
        """

        self.current_time = max(self.min_time, min(time, self.max_time))

    def set_max(self, max_time: float):
        self.max_time = max_time

        if self.current_time > self.max_time:
            self.current_time = self.max_time

    def set_min(self, min_time: float):
        self.min_time = min_time

        if self.current_time < self.min_time:
            self.current_time = self.min_time

    def set_scale(self, scale: float):
        self.scale = scale

    def reset(self):
        self.set(self.min_time)

    # Getters
    #   - No need for epsilons as the value saturates at it's bounds

    def get(self) -> float:
        return self.current_time

    def at_max(self) -> bool:
        return self.current_time == self.max_time

    def at_min(self) -> bool:
        return self.current_time == self.min_time


# Tag


def last_tag(use_registry: bool = True):
    return Tag(dpg.last_item(), use_registry)


class Tag:
    """A dearpygui helper class to make working with objects easier."""

    tag: int | str
    """ A unique-id referring to a dearpygui object. """

    handler_registry: int | str | None = None

    # Initialization

    def __init__(self, tag: int | str, use_registry: bool = True):
        self.tag = tag
        if use_registry:
            self.handler_registry = dpg.add_item_handler_registry()
            dpg.bind_item_handler_registry(self.tag, self.handler_registry)

    # Context

    @dpg.contextmanager
    def handler_context(self):
        assert self.handler_registry
        try:
            dpg.push_container_stack(self.handler_registry)
            yield self.handler_registry
        finally:
            dpg.pop_container_stack()

    def __enter__(self):
        dpg.push_container_stack(self.tag)

    def __exit__(self, *_):
        dpg.pop_container_stack()

    # Values

    def get(self, key: str) -> Any | None:
        conf = dpg.get_item_configuration(self.tag)
        return conf[key] if key in conf else None

    def get_position(self) -> list[float]:
        return dpg.get_item_pos(self.tag)

    def set(self, key: str, value: Any):
        match key:
            case "value":
                self.set_value(value)
            case _:
                dpg.configure_item(self.tag, **{key: value})

    def set_value(self, value: Any):
        dpg.set_value(self.tag, value)

    def set_position(self, position: list[float]):
        dpg.set_item_pos(self.tag, position)

    # Overrides

    def __setitem__(self, key, value):
        self.set(key, value)

    def __getitem__(self, key):
        return self.get(key)


# Maths


def lerp(a, b, t):
    return (1 - t) * a + t * b


# DPG


def toggle_tags_visibility(tags: list[Tag]):
    for tag in tags:
        tag.set("show", not tag.get("show"))
