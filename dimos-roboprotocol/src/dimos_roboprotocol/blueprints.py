"""Entry-point blueprints (group `dimos.blueprints`)."""
from dimos.core.coordination.blueprints import autoconnect

from .module import RoboteleBridge

xgo_lite = autoconnect(RoboteleBridge.blueprint())
