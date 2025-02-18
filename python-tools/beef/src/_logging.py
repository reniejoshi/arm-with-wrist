import sys, datetime

from loguru import logger
from theming import get_catppuccin_color_hex

# Logger

START_TIME = datetime.datetime.now(datetime.UTC)

__peach = get_catppuccin_color_hex("peach")
__yellow = get_catppuccin_color_hex("yellow")
__lavender = get_catppuccin_color_hex("lavender")
__red = get_catppuccin_color_hex("red")


def configure():
    logger.remove()
    logger.add(
        sys.stdout,
        colorize=True,
        level="DEBUG",
        format=f"<fg {__peach}>[</><b><fg {__lavender}>{{file}}</> <fg {__yellow}>{{function}}</> <fg {__red}>{{thread}}</></><fg {__peach}>]</> <lvl>{{level}}</> <fg {__lavender}>-</> <lvl>{{message}}</>",
        diagnose=True,
        enqueue=True,
    )
