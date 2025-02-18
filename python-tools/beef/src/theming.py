import dearpygui.dearpygui as dpg
from catppuccin import PALETTE

from utils import *

# Constants

FONT = "0xProto-Regular"
FONT_SIZE = 16
SMALL_FONT_SIZE = 14
FLAVOR = PALETTE.mocha

FONT_TAG = "FONT"
SMALL_FONT_TAG = "SMOL"

# Functions


def get_catppuccin_color(
    color: str, alpha=255, flavor=FLAVOR
) -> tuple[float, float, float, float]:
    """Get a color from the selected Catppuccin flavor."""
    rgb = flavor.colors.__dict__[color].rgb
    return rgb.r, rgb.g, rgb.b, alpha


def get_catppuccin_color_hex(
    color: str, alpha: float | None = None, flavor=FLAVOR
) -> str:
    """Get a color from the selected Catppuccin flavor formatted as hex."""
    return flavor.colors.__dict__[color].hex + (f"{alpha:x}" if alpha else "")


def configure_global_fonts():
    """Configures the default global font for dearpygui."""

    from pathlib import Path

    global FONT_TAG, SMALL_FONT_TAG

    with dpg.font_registry():
        # Load font and register it as the global font
        dpg.add_font(
            Path(__file__).parent.__str__() + f"/../assets/fonts/{FONT}.ttf",
            FONT_SIZE,
            tag=FONT_TAG,
        )
        dpg.add_font(
            Path(__file__).parent.__str__() + f"/../assets/fonts/{FONT}.ttf",
            SMALL_FONT_SIZE,
            tag=SMALL_FONT_TAG,
        )
    dpg.bind_font(FONT_TAG)


def configure_global_theme():
    """Configures global theming properites for dearpygui, such as colors."""

    with dpg.theme() as theme:
        with dpg.theme_component(dpg.mvAll):
            dpg.add_theme_color(dpg.mvThemeCol_WindowBg, get_catppuccin_color("base"))
            dpg.add_theme_color(dpg.mvThemeCol_MenuBarBg, get_catppuccin_color("base"))
            dpg.add_theme_color(dpg.mvThemeCol_Text, get_catppuccin_color("lavender"))
    dpg.bind_theme(theme)


def clear_plot_background(plot_tag: Tag):
    """Removes the background of a plot widget."""

    with dpg.theme() as theme:
        with dpg.theme_component(dpg.mvPlot):
            dpg.add_theme_color(dpg.mvThemeCol_FrameBg, [0, 0, 0, 0])
    dpg.bind_item_theme(plot_tag.tag, theme)


def set_plot_series_line_color(tag: Tag, color: str, alpha=255):
    """Sets the line color of a series on a plot."""

    with dpg.theme() as theme:
        with dpg.theme_component(dpg.mvAll):
            dpg.add_theme_color(
                dpg.mvPlotCol_Line,
                get_catppuccin_color(color, alpha),
                category=dpg.mvThemeCat_Plots,
            )
    dpg.bind_item_theme(tag.tag, theme)


def global_theme():
    """Configures global theming and scales the application 'properly'"""

    configure_global_fonts()
    configure_global_theme()

    # Hi-DPI
    import ctypes, sys

    if sys.platform.startswith("win"):
        ctypes.windll.shcore.SetProcessDpiAwareness(4)
