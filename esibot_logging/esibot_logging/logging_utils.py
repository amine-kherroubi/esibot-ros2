"""
Centralized logging utilities for EsiBot.

- Colorized console logs for standard Python logging.
- Single entry point for both ROS 2 node loggers and Python loggers.
"""

from __future__ import annotations

import logging
import os
import sys
from typing import Optional

_DEFAULT_FORMAT = "%(asctime)s | %(levelname)s | %(name)s: %(message)s"
_DEFAULT_DATEFMT = "%H:%M:%S"

_COLOR_RESET = "\033[0m"
_LEVEL_COLORS = {
    logging.DEBUG: "\033[38;5;244m",  # grey
    logging.INFO: "\033[32m",  # green
    logging.WARNING: "\033[33m",  # yellow
    logging.ERROR: "\033[31m",  # red
    logging.CRITICAL: "\033[1;37;41m",  # white on red background
}

_CONFIGURED = False


class ColorFormatter(logging.Formatter):
    def __init__(self, fmt: str, datefmt: str, use_color: bool) -> None:
        super().__init__(fmt=fmt, datefmt=datefmt)
        self.use_color = use_color

    def format(self, record: logging.LogRecord) -> str:
        if self.use_color:
            color = _LEVEL_COLORS.get(record.levelno)
            if color:
                original_levelname = record.levelname
                record.levelname = f"{color}{record.levelname}{_COLOR_RESET}"
                try:
                    return super().format(record)
                finally:
                    record.levelname = original_levelname
        return super().format(record)


def _coerce_level(level: Optional[str | int]) -> int:
    if isinstance(level, int):
        return level
    if not level:
        return logging.INFO
    if isinstance(level, str):
        name = level.strip().upper()
        return logging._nameToLevel.get(name, logging.INFO)
    return logging.INFO


def _should_use_color(value: Optional[bool]) -> bool:
    if value is not None:
        return bool(value)
    if os.getenv("NO_COLOR") or os.getenv("ESIBOT_NO_COLOR"):
        return False
    if os.getenv("FORCE_COLOR") or os.getenv("ESIBOT_FORCE_COLOR"):
        return True
    return sys.stderr.isatty()


def setup_logging(
    level: Optional[str | int] = None,
    *,
    use_color: Optional[bool] = None,
    fmt: Optional[str] = None,
    datefmt: Optional[str] = None,
    force: bool = False,
) -> None:
    """
    Configure root Python logging with a colorized console handler.

    This should be called once by application entrypoints (nodes/scripts).
    """
    global _CONFIGURED
    if _CONFIGURED and not force:
        return

    root = logging.getLogger()
    level = _coerce_level(
        level or os.getenv("ESIBOT_LOG_LEVEL") or os.getenv("LOG_LEVEL") or logging.INFO
    )

    fmt = fmt or _DEFAULT_FORMAT
    datefmt = datefmt or _DEFAULT_DATEFMT
    use_color = _should_use_color(use_color)

    handler = None
    for existing in root.handlers:
        if getattr(existing, "_esibot_logging", False):
            handler = existing
            break

    if handler is None:
        if root.handlers and not force:
            root.setLevel(level)
            _CONFIGURED = True
            return
        handler = logging.StreamHandler()
        handler._esibot_logging = True  # type: ignore[attr-defined]
        root.addHandler(handler)

    handler.setLevel(level)
    handler.setFormatter(ColorFormatter(fmt=fmt, datefmt=datefmt, use_color=use_color))
    root.setLevel(level)
    _CONFIGURED = True


def get_logger(name: Optional[str] = None, *, node=None):
    """
    Return a logger. If `node` is provided, returns the ROS 2 logger from that node.
    Otherwise returns a standard Python logger.
    """
    if node is not None:
        return node.get_logger()
    return logging.getLogger(name)
