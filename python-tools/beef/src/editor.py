import math
from pathlib import Path

from utils import CartesianVec as P, BoundedTimer
from constants import *
from trajectory import *

from wpilib import SmartDashboard
from ntcore import NetworkTableInstance

import dearpygui.dearpygui as dpg
from theming import *

from loguru import logger

import _logging

_logging.configure()

# App

APP_NAME = "Beef's Editor (Extra Fancy!!) v2"

DEBUGGING_WINDOW_WIDTH = 400
DELETE_DISTANCE = 0.1

# Application


def toggle_fullscreen():
    dpg.toggle_viewport_fullscreen()


def save_application_state():
    with open(".beef", "w") as f:
        x, y = dpg.get_viewport_pos()
        w, h = dpg.get_viewport_width(), dpg.get_viewport_height()
        f.write(f"{x} {y} {w} {h}")


def load_application_state() -> list[int] | None:
    import os

    if not os.path.exists(".beef"):
        return None

    with open(".beef", "r") as f:
        x, y, w, h = f.read().split(" ")
        return [int(x), int(y), int(w), int(h)]


# Files

SAVE_DIR = Path.cwd().absolute()
while not SAVE_DIR.joinpath(".wpilib").exists():
    SAVE_DIR = SAVE_DIR.parent.absolute()
    logger.debug(f"No .wpilib found in {SAVE_DIR}, going up.")
SAVE_DIR = SAVE_DIR.joinpath("src/main/deploy/beef/")
SAVE_DIR.mkdir(parents=True, exist_ok=True)
logger.info(f"Saving trajectories to {SAVE_DIR}")


def get_trajectories() -> list[Path]:
    return [f for f in SAVE_DIR.glob("*") if f.is_file() and f.suffix == ".traj"]


class Editor:
    # State

    trajectory = Trajectory(TrajectoryConfig(1, 1))
    point_indices: list[Tag] = []

    # Animation

    playing = False
    timer = BoundedTimer(max_time=0.0)

    # Initialization

    current_file: Path | None = None

    debugging = False
    debugging_features = {
        "p2c": {"enabled": False, "tags": []},
        "metrics": {"enabled": False, "tags": []},
    }

    plot_cursor_position = [0, 0]

    def __init__(self):
        # TODO: Load previous trajectory

        # UI
        # fmt: off

        # - Window

        self.ui_window = Tag(dpg.add_window(no_resize=True, no_move=True, pos=[0, 0], no_title_bar=True), use_registry=False,)
        self.ui_debugging_window = Tag(dpg.add_window(show=self.debugging, no_resize=True, no_move=True, pos=[0, 0], width=DEBUGGING_WINDOW_WIDTH, horizontal_scrollbar=True, no_title_bar=True,), use_registry=False,)

        # - Dialogs

        self.ui_save_as_dialog = Tag(dpg.add_file_dialog(default_path=SAVE_DIR.absolute().__str__(), show=False, callback=self.__cb_save_as, cancel_callback=self.__cb_hide_save_as))
        with self.ui_save_as_dialog: dpg.add_file_extension("Trajectories (*.traj){.traj}", color=get_catppuccin_color("peach"),)

        # - Plot

        with self.ui_window:
            self.ui_plot = Tag(dpg.add_plot(**self.plot_widget_configuration))
            clear_plot_background(self.ui_plot)

        with self.ui_plot:
            self.ui_plot_axis_x = Tag(dpg.add_plot_axis(axis=dpg.mvXAxis, label="x (meters)", **self.plot_axis_configuration,))
            self.ui_plot_axis_y = Tag(dpg.add_plot_axis(axis=dpg.mvYAxis, label="z (meters)", **self.plot_axis_configuration,))

        with self.ui_plot_axis_y: self.ui_plot_ground = Tag(dpg.add_inf_line_series(x=[PIVOT_TO_GROUND], horizontal=True))

        # - Windmill
        
        with self.ui_plot:
            # Trajectory
            self.ui_trajectory = Tag(dpg.draw_polyline([], thickness=self.plot_line_thickness_meters,))
            self.ui_elevator = Tag(dpg.draw_circle(center=[0, 0], radius=0.1016 / 2, thickness=self.plot_line_thickness_meters, color=get_catppuccin_color("blue"),))
            self.ui_arm = Tag(dpg.draw_line(p1=[0, 0], p2=[0, 0], thickness=self.plot_line_thickness_meters, color=get_catppuccin_color("blue"),))
            self.ui_end_effector = Tag(dpg.draw_circle(center=[0, 0], radius=0.03, thickness=self.plot_line_thickness_meters, color=get_catppuccin_color("blue"),))
            
            # Error
            self.ui_ghost_trajectory = Tag(dpg.draw_polyline([], thickness=self.plot_line_thickness_meters, color=[255, 0, 0, 100],))
            self.ui_error_indicator = Tag(dpg.draw_circle(center=[0, 0], radius=0.005, fill=get_catppuccin_color("red"), color=get_catppuccin_color("red"), thickness=0.0, show=False,))

        # - Action Bar

        with self.ui_window: self.ui_action_bar_wrapper = Tag(dpg.add_table(header_row=False, borders_innerH=False, borders_outerH=False, borders_innerV=False, borders_outerV=False,))

        with self.ui_action_bar_wrapper:
            dpg.add_table_column(no_resize=True, init_width_or_weight=2)
            dpg.add_table_column(no_resize=True, init_width_or_weight=2)
            dpg.add_table_column(no_resize=True, init_width_or_weight=1)

            self.ui_action_bar = Tag(dpg.add_table_row())

        with self.ui_action_bar:
            self.ui_trajectory_controls = Tag(dpg.add_group(horizontal=True))
            self.ui_animation_controls = Tag(dpg.add_group(horizontal=True))
            self.ui_time_scale_controls = Tag(dpg.add_group(horizontal=True))

        with self.ui_trajectory_controls:
            self.ui_trajectory_selector = Tag(dpg.add_combo([f.name for f in get_trajectories()] + (["<unnamed>"] if self.current_file else []), fit_width=True, default_value=self.get_current_file_name(), callback=self.__cb_select_trajectory))
            
            dpg.add_button(label="Refresh", callback=self.__cb_refresh_trajectories)
            dpg.add_button(label="Save", callback=self.__cb_save)
            dpg.add_button(label="Save As", callback=self.__cb_show_save_as)
            dpg.add_button(label="New", callback=self.__cb_new)

        with self.ui_animation_controls:
            dpg.add_text("Time (s)")
            self.ui_play_button = Tag(dpg.add_button(label="Play", callback=self.__cb_toggle_playing))
            self.ui_time_slider = Tag(dpg.add_slider_float(clamped=True, min_value=0, max_value=0, width=-1, callback=self.__cb_set_time,))

        with self.ui_time_scale_controls:
            dpg.add_text("Time Scale")
            self.ui_time_scale_slider = Tag(dpg.add_slider_float(clamped=True, min_value=-1, max_value=1, default_value=1, width=-1, callback=self.__cb_set_time_scale,))

        # - Debug
            
        with self.ui_plot:
            self.ui_debug_p2c_point = Tag(dpg.add_drag_point(color=get_catppuccin_color("mauve"), callback=self.__debug_p2c_point))
            self.debugging_features["p2c"]["tags"].append(self.ui_debug_p2c_point)
            toggle_tags_visibility(self.debugging_features["p2c"]["tags"])
            
        with self.ui_debugging_window:
            dpg.bind_item_font(self.ui_debugging_window.tag, SMALL_FONT_TAG)
            
            heading = dpg.add_text("[Debugger]", color=get_catppuccin_color("peach"))
            dpg.bind_item_font(heading, FONT_TAG)
            
            with Tag(dpg.add_collapsing_header(label="[State]", default_open=True)): 
                self.ui_debug_state = Tag(dpg.add_text(""))
                with dpg.group(horizontal=True):
                    dpg.add_button(label = "Test Serialization", callback=self.serialize)
                    dpg.add_button(label = "Deserialize from Console", callback=lambda: self.deserialize(input("JSON: ")))

            with Tag(dpg.add_collapsing_header(label="[Visualizations]", default_open=False)): 
                dpg.add_button(label="[ ] Toggle Point-to-Cursor Distance Visualization", callback=self.__cb_debug_feature_generator("p2c", self.__debug_p2c_point))
    
            with Tag(dpg.add_collapsing_header(label="[Tooling]", default_open=False)): 
                dpg.add_button(label="Metrics", callback=self.__cb_debug_feature_generator("metrics", dpg.show_metrics))

        # fmt: on

        self.draw_robot()
        self.draw_end_effector_limits()

        logger.debug("Created UI.")

        # Register event handlers
        self.register_global_handlers()
        self.register_item_handlers()

        logger.debug("Registered global and item-specific bindings.")

    # Handlers

    # - Registration

    def register_global_handlers(self):
        # Resize widgets on viewport resized
        dpg.set_viewport_resize_callback(self.resize_widgets)

        # Save the application state on exit
        dpg.set_exit_callback(save_application_state)

        with dpg.handler_registry():
            dpg.add_key_release_handler(
                dpg.mvKey_Spacebar, callback=self.__cb_toggle_playing
            )

            dpg.add_key_release_handler(
                dpg.mvKey_Tilde, callback=self.__cb_toggle_debugging
            )

            dpg.add_key_release_handler(dpg.mvKey_D, callback=self.delete_closest_point)

            dpg.add_key_release_handler(dpg.mvKey_F11, callback=toggle_fullscreen)
            dpg.add_key_release_handler(dpg.mvKey_F4, callback=dpg.stop_dearpygui)

    def register_item_handlers(self):
        with self.ui_plot.handler_context():
            dpg.add_item_clicked_handler(callback=self.__cb_plot_click)

    # - Implementation

    def add_drag_point(
        self, x: float, y: float, force_index_no_insert: int | None = None
    ):
        pos = P(x, y).constrain()

        # Create the point on the trajectory
        if force_index_no_insert == None:
            closest = (
                self.trajectory.get_index_and_distance_to_closest_point_to_position(pos)
            )

            index = self.trajectory.add_point(
                pos,
                max(1, closest[0]) if closest else closest,
            )
        else:
            logger.debug(f"No insertion, forcing index of {force_index_no_insert}")
            index = force_index_no_insert

        logger.debug(f"Created draggable point at ({x:.3f}, {y:.3f}).")

        # Add the point to the plot and bind the user_data to it's corresponding index in the trajectory
        tag = Tag(
            dpg.add_drag_point(
                parent=self.ui_plot.tag,
                color=get_catppuccin_color("lavender"),
                default_value=pos.to_list(),
                show_label=True,
                label=index,
                user_data=index,
                callback=self.handle_point_drag,
            )
        )
        logger.debug(
            f"Draggable point with tag {tag.tag} has been associated to index {index}."
        )

        # If it wasn't inserted at the end of the trajectory
        self.point_indices.insert(index, tag)
        self.sync_tag_indices()

        # Update the plot with the new trajectory
        self.draw_trajectory()

    def handle_point_drag(self, sender, _, index):
        # Retrieve the plot-relative position of the point.
        pos_ = P(*dpg.get_value(sender))
        pos = pos_.constrain()
        if pos_ != pos:
            logger.debug(f"Position has been constrained to {pos}.")

        # Set the position of the point in the trajectory
        self.trajectory.set_point_position(index, pos)

        # Sync constraints back to the point
        dpg.set_value(sender, pos.to_list())
        self.draw_trajectory()

    def delete_closest_point(self, ignore_distance=False):
        closest = self.trajectory.get_index_and_distance_to_closest_point_to_position(
            P(*dpg.get_plot_mouse_pos())
        )

        if not closest:
            return

        index, distance = closest

        if not ignore_distance and distance > DELETE_DISTANCE:
            return

        dpg.delete_item(self.point_indices.pop(index).tag)
        self.trajectory.delete_point(index)
        self.sync_tag_indices()

        self.draw_trajectory()

        logger.debug(f"Deleted point {index}")

    def sync_tag_indices(self):
        for i, tag in enumerate(self.point_indices):
            tag.set("user_data", i)
            tag.set("label", i)

    def resize_widgets(self):
        w, h = dpg.get_viewport_client_width(), dpg.get_viewport_client_height()

        self.ui_window["height"] = h
        self.ui_debugging_window["height"] = h

        self.ui_window["width"] = w - DEBUGGING_WINDOW_WIDTH if self.debugging else w
        self.ui_window.set_position(
            [
                DEBUGGING_WINDOW_WIDTH if self.debugging else 0,
                self.ui_window.get_position()[1],
            ]
        )

        self.ui_plot["height"] = h - 50

    # Callbacks

    def __cb_debug_feature_generator(self, feature: str, setup=None):
        def __cb(sender):
            if setup:
                setup()

            self.debugging_features[feature]["enabled"] = not self.debugging_features[
                feature
            ]["enabled"]
            toggle_tags_visibility(self.debugging_features[feature]["tags"])

            sender = Tag(sender)
            if sender["label"].startswith("["):
                sender["label"] = (
                    "[x]" if self.debugging_features[feature]["enabled"] else "[ ]"
                ) + sender["label"][3:]

        return __cb

    def __cb_plot_click(self, _, app_data: tuple[int, int]):
        """Handle a click on the editor's plot (not on a draggable point).

        - `shift + left-click`: Creates a new point before the closest point.
        """

        button, _ = app_data

        match button:
            case dpg.mvMouseButton_Left:
                if dpg.is_key_down(dpg.mvKey_ModShift):
                    self.add_drag_point(*dpg.get_plot_mouse_pos())

    def __cb_toggle_playing(self):
        self.set_playing(not self.playing, reset=True)

    def __cb_set_time_scale(self, _, time_scale: float):
        """Sets the time scale of the timer.

        Args:
            _: Sender's tag (ignored)
            time_scale (float): The time scale
        """

        self.timer.set_scale(time_scale)

    def __cb_set_time(self, _, time):
        """
        Sets the time of the trajectory simulation.

        - `time` is clamped to the trajectory's duration.

        :param time: The time in seconds
        """

        self.timer.set(time)
        self.sync_time()

    def __cb_refresh_trajectories(self):
        self.ui_trajectory_selector.set(
            "items",
            [f.name for f in get_trajectories()]
            + (["<unnamed>"] if not self.current_file else []),
        )

    def __cb_select_trajectory(self, _, file_name):
        file = SAVE_DIR.joinpath(file_name)
        self.__cb_save()
        self.open(file)

    def __cb_toggle_debugging(self):
        self.debugging = not self.debugging
        self.ui_debugging_window.set("show", self.debugging)

        self.resize_widgets()

    def __cb_new(self):
        self.__cb_save()
        self.clear_trajectory()
        self.clear_current_file()

    def __cb_save(self):
        if not self.current_file:
            return
        self.save()

    def __cb_show_save_as(self):
        self.ui_save_as_dialog.set("show", True)

    def __cb_hide_save_as(self):
        self.ui_save_as_dialog.set("show", False)

    def __cb_save_as(self, _, data):
        self.set_current_file(data["file_name"])
        self.save()

    # Drawing

    def draw_trajectory(self):
        # Attempt to regenerate the trajectory if it has changed
        if (
            self.trajectory.dirty
            and self.trajectory.regenerate_trajectory()
            and self.trajectory.is_valid()
        ):
            self.sync_duration()

        error = self.trajectory.get_error()

        # Convert the trajectory into a list of (x, y) pairs
        trajectory_points = self.trajectory.discretize()

        # If there was an error, show the error visualization and return
        if error:
            self.ui_ghost_trajectory.set("points", trajectory_points)

            error_point = self.trajectory.sample_end_effector_position(error.time, True)
            self.ui_error_indicator.set("center", error_point.to_list())
            self.ui_error_indicator.set("show", True)
            self.ui_error_indicator.set("label", error.message)

            return
        else:
            self.ui_ghost_trajectory.set("points", [])
            self.ui_error_indicator.set("show", False)

        self.publish()

        # Display the trajectory
        self.ui_trajectory.set("points", trajectory_points)

        if self.trajectory.is_empty():
            self.reset_windmill()

        # Draw the windmill
        self.draw_windmill()

        # Debugging Features
        self.__debug_p2c_point()

        self.update_debug()

    def draw_windmill(self):
        if not self.trajectory.is_valid():
            return

        # Get current time
        time = self.timer.get()

        # Calculate the state of the windmill at the time
        end_effector_position = self.trajectory.sample_end_effector_position(
            time
        ).to_list()
        height = self.trajectory.sample_windmill_state(time).height

        if height == None:
            return

        # Display the windmill
        self.ui_end_effector.set("center", end_effector_position)
        self.ui_elevator.set("center", (0, height))
        self.ui_arm.set("p1", (0, height))
        self.ui_arm.set("p2", end_effector_position)

    def reset_windmill(self):
        self.ui_end_effector.set("center", [0, 0])
        self.ui_elevator.set("center", (0, 0))
        self.ui_arm.set("p1", (0, 0))
        self.ui_arm.set("p2", [0, 0])

    # Setters

    def set_playing(self, playing: bool, reset=False):
        """Sets the playing state of the editor

        Args:
            playing (bool): Whether the editor is running the trajectory
            reset (bool, optional): Whether or not to reset the time to the start if it reaches the end. Defaults to False.
        """

        if not self.trajectory.is_valid():
            playing = False

        # If reset is enabled and the trajectory is done, then stop playing and reset the timer to 0
        if reset and self.timer.at_max():
            self.timer.reset()

            # Update the state of the windmill
            self.draw_windmill()

            playing = False

        self.playing = playing
        self.timer.reset_rt()

        # Change the label of the button based on the state
        self.ui_play_button.set("label", "Pause" if self.playing else "Play")

        if self.playing:
            logger.debug(f"Playing from {self.timer.get():.3f} seconds.")
        else:
            logger.debug(f"Paused at {self.timer.get():.3f} seconds.")

    def sync_duration(self):
        """Synchronizes the time slider's max value to be the duration of the trajectory."""

        if not self.trajectory.is_valid:
            return

        # Get the duration of the trajectory
        duration = self.trajectory.get_duration()

        # Sync to the timer and slider
        self.timer.set_max(duration)
        self.ui_time_slider.set("max_value", duration)

    def sync_time(self):
        """Synchronize the UI with the timer."""

        # Redraw the windmill
        self.draw_windmill()

        # Set the slider to the value of the timer
        self.ui_time_slider.set("value", self.timer.get())

    def clear_trajectory(self):
        while len(self.trajectory.points) > 0:
            self.delete_closest_point(ignore_distance=True)

        logger.info("Trajectory cleared.")

        self.trajectory = Trajectory(TrajectoryConfig(1, 1))

        self.reset_windmill()
        self.draw_trajectory()

    # Persistence

    def serialize(self) -> str:
        if not self.trajectory.is_valid():
            logger.warning("Cannot serialize an invalid trajectory.")
            return

        serialized_trajectory = self.trajectory.serialize()

        data = json.dumps(serialized_trajectory)

        return data

    def deserialize(self, data: str) -> bool:
        data = json.loads(data)

        try:
            trajectory = Trajectory.deserialize(data)
        except Exception as e:
            logger.error(
                f"Failed to deserialize trajectory because it '{e}', please ensure the file's contents are valid."
            )
            return False

        self.clear_trajectory()
        self.trajectory = trajectory

        return True

    def save(self):
        data = self.serialize()
        if not data:
            logger.warning("Cannot save empty trajectory.")
            return

        with self.current_file.open("w") as f:
            f.write(data)

        logger.info(f"Saved trajectory to {self.current_file}")

        self.__cb_refresh_trajectories()

    def open(self, file: Path):
        with file.open() as f:
            success = self.deserialize(f.read())

        if not success:
            return

        self.current_file = file

        for i, point in enumerate(self.trajectory.points):
            self.add_drag_point(point.x, point.y, i)

        self.trajectory.regenerate_trajectory()

        self.sync_duration()
        self.timer.reset()

        self.draw_trajectory()

    def publish(self):
        if self.trajectory.is_valid():
            SmartDashboard.putString("BEEF/Serialized", self.serialize())

    # Update

    def update(self):
        self.update_debug()

        self.publish()

        if self.playing and self.trajectory.is_valid():
            if self.timer.at_max():
                logger.debug("Completed trajectory playback.")
                self.set_playing(False, reset=False)
            self.timer.advance_rt()
            self.sync_time()

    def update_debug(self):
        if self.debugging:
            # Editor State

            # - Editor
            editor_state = f"Editor:\n\tTime: {self.timer.get():.5f}\n\tTime Scale: {self.timer.scale:.3f}\n\tMouse Position: {self.plot_cursor_position}\n"

            # - Trajectory
            trajectory_state = (
                f"Trajectory:\n"
                + f"\tEmpty? {self.trajectory.is_empty()}\n"
                + f"\tDirty? {self.trajectory.dirty}\n"
                + f"\tPoints:\n\t\t{"\n\t\t".join([f"({p.x:.5f}, {p.y:.5f}) [{self.point_indices[i].tag}]" for i, p in enumerate(self.trajectory.points)])}\n"
                + f"\tGeneration Time: {self.trajectory.get_generation_time()} seconds"
            )

            self.ui_debug_state["value"] = editor_state + trajectory_state

    # Helpers

    def get_current_file_name(self) -> str:
        return "<unnamed>" if not self.current_file else self.current_file.name

    def set_current_file(self, file: str):
        self.current_file = SAVE_DIR.joinpath(file)
        self.__cb_refresh_trajectories()
        self.ui_trajectory_selector.set_value(file)

    def clear_current_file(self):
        self.current_file = None
        self.__cb_refresh_trajectories()
        self.ui_trajectory_selector.set_value("<unnamed>")

    # Debug

    def __debug_p2c_point(self):
        if not self.debugging_features["p2c"]["enabled"]:
            return

        tags = self.debugging_features["p2c"]["tags"]

        position = dpg.get_value(self.ui_debug_p2c_point.tag)
        line_count = len(self.point_indices)

        while len(tags) - 1 > line_count:
            tag = tags[-1]
            dpg.delete_item(tag)
            del tag

        while len(tags) - 1 < line_count:
            tags.append(
                Tag(
                    dpg.draw_line(
                        [0, 0],
                        [0, 0],
                        parent=self.ui_plot.tag,
                        thickness=self.plot_line_thickness_meters,
                        color=get_catppuccin_color("mauve"),
                    )
                )
            )

        for i, p in enumerate(self.trajectory.points):
            # TODO: OOB exception when adding new points
            tags[i + 1]["p1"] = position
            tags[i + 1]["p2"] = [p.x, p.y]

            distance = p.distance(Translation2d(*position))

    # Configuration

    plot_widget_configuration = {
        "equal_aspects": True,
        # "no_menus": True,
        "width": -1,
        "height": -1,
    }
    plot_axis_configuration = {"no_side_switch": True, "no_highlight": True}

    plot_line_thickness_meters = 0.002

    # Limits

    def draw_end_effector_limits(self):
        x = []
        y = []

        # Physical Limitations
        x.append(-ARM_LENGTH)
        y.append(END_EFFECTOR_MIN_POSE)

        x.append(COLLECTOR_STOWED_MIN_X)
        y.append(END_EFFECTOR_MIN_POSE)

        x.append(COLLECTOR_STOWED_MIN_X)
        y.append(COLLECTOR_HEIGHT)

        x.append(ARM_LENGTH)
        y.append(COLLECTOR_HEIGHT)

        x.append(ARM_LENGTH)
        y.append(ELEVATOR_MAX_POSE)

        # Max extension
        for t in range(0, 100):
            t = float(t) / 100 * math.pi
            x.append(math.cos(t) * ARM_LENGTH)
            y.append(math.sin(t) * ARM_LENGTH + ELEVATOR_MAX_POSE)

        x.append(-ARM_LENGTH)
        y.append(ELEVATOR_MAX_POSE)

        x.append(-ARM_LENGTH)
        y.append(END_EFFECTOR_MIN_POSE)

        with self.ui_plot_axis_y:
            set_plot_series_line_color(Tag(dpg.add_line_series(x, y)), "red", 127)

    def draw_robot(self):
        with self.ui_plot_axis_y:
            # Bumpers
            set_plot_series_line_color(
                Tag(
                    dpg.add_line_series(
                        [BUMPER_REV, BUMPER_FWD, BUMPER_FWD, BUMPER_REV, BUMPER_REV],
                        [
                            BUMPER_MAX_Y,
                            BUMPER_MAX_Y,
                            BUMPER_MIN_Y,
                            BUMPER_MIN_Y,
                            BUMPER_MAX_Y,
                        ],
                    )
                ),
                "green",
                127,
            )

        with self.ui_plot:
            # Elevator Top
            dpg.draw_circle(
                (0, ELEVATOR_MAX_POSE),
                0.03,
                color=get_catppuccin_color("red", 127),
            )

            # Wheels
            dpg.draw_circle(
                (WHEEL_BASE_FWD, WHEEL_PIVOT),
                WHEEL_RADIUS,
                color=get_catppuccin_color("text", 127),
            )
            dpg.draw_circle(
                (WHEEL_BASE_REV, WHEEL_PIVOT),
                WHEEL_RADIUS,
                color=get_catppuccin_color("text", 127),
            )
            dpg.draw_arrow(
                (0.1 - CENTER_X_TO_ELEVATOR, (BUMPER_MAX_Y + BUMPER_MIN_Y) / 2),
                (-0.1 - CENTER_X_TO_ELEVATOR, (BUMPER_MAX_Y + BUMPER_MIN_Y) / 2),
                size=0.02,
                thickness=0.005,
                color=get_catppuccin_color("text"),
            )

        with self.ui_plot_axis_y:
            # Elevator
            set_plot_series_line_color(
                Tag(
                    dpg.add_line_series(
                        [
                            ELEVATOR_X_REV,
                            ELEVATOR_X_FWD,
                            ELEVATOR_X_FWD,
                            ELEVATOR_X_REV,
                            ELEVATOR_X_REV,
                        ],
                        [
                            ELEVATOR_HEIGHT,
                            ELEVATOR_HEIGHT,
                            BUMPER_MAX_Y,
                            BUMPER_MAX_Y,
                            ELEVATOR_HEIGHT,
                        ],
                    )
                ),
                "yellow",
                127,
            )


@logger.catch
def main():
    global FULLSCREEN

    dpg.create_context()
    dpg.setup_dearpygui()
    global_theme()

    logger.debug("DPG setup and configured.")

    editor = Editor()

    # Windowing

    # - Load previous application state

    state = load_application_state()
    if state:
        x, y, w, h = state
        dpg.create_viewport(title=APP_NAME, width=w, height=h, x_pos=x, y_pos=y)
    else:
        dpg.create_viewport(title=APP_NAME)

    inst = NetworkTableInstance.getDefault()

    logger.debug("Starting NT4 client for attachment to 2046.")
    inst.setServerTeam(2046)
    inst.startClient4("BEEF Editor")

    if not inst.isConnected():
        logger.debug("Starting NT4 client for attachment to simulation.")
        inst.setServer("localhost")

    dpg.show_viewport()

    logger.info("BEEFv2 Started!")
    while dpg.is_dearpygui_running():
        editor.update()
        dpg.render_dearpygui_frame()
    dpg.destroy_context()


if __name__ == "__main__":
    main()
