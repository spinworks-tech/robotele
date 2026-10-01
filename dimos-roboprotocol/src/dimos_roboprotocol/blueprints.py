"""Entry-point blueprints (group `dimos.blueprints`)."""
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.global_config import global_config
from dimos.visualization.vis_module import vis_module

from .module import RoboteleBridge
from .rerun_views import RERUN_CONFIG

xgo_lite = autoconnect(
    RoboteleBridge.blueprint(),
    vis_module(viewer_backend=global_config.viewer, rerun_config=RERUN_CONFIG),
)
