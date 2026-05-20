# Sphinx configuration for a standalone build of the mujoco_ros2_control docs.
#
# This is a stripped down version of the upstream docs configuration just for building
# locally.

import os

project = "mujoco_ros2_control"
master_doc = "index"
source_suffix = ".rst"
language = "en"
exclude_patterns = [
    "_build",
    ".pixi",
    "build",
    "install",
    "log",
    "Thumbs.db",
    ".DS_Store",
    "**/CHANGELOG.rst",
]
pygments_style = "sphinx"

extensions = [
    "sphinx.ext.intersphinx",
    "sphinx.ext.todo",
    "sphinx_rtd_theme",
    "sphinx_copybutton",
]

todo_include_todos = True

html_theme = "sphinx_rtd_theme"
html_theme_options = {
    "collapse_navigation": True,
    "sticky_navigation": True,
    "navigation_depth": -1,
}

# Hardcoding to jazzy just for the build
ros_distro = os.environ.get("ROS_DISTRO", "jazzy")

macros = {
    "DISTRO": ros_distro,
    "DISTRO_TITLE": ros_distro.title(),
    "DISTRO_TITLE_FULL": {
        "humble": "Humble Hawksbill",
        "iron": "Iron Irwini",
        "jazzy": "Jazzy Jalisco",
        "kilted": "Kilted Kaiju",
        "rolling": "Rolling Ridley",
    }.get(ros_distro, ros_distro.title()),
    "REPOS_FILE_BRANCH": "master" if ros_distro == "rolling" else ros_distro,
}


def expand_macros(app, docname, source):
    """Replace {KEY} placeholders in rst source with values from *macros*."""
    result = source[0]
    for key, value in app.config.macros.items():
        result = result.replace(f"{{{key}}}", value)
    source[0] = result


def setup(app):
    app.add_config_value("macros", {}, True)
    app.connect("source-read", expand_macros)
