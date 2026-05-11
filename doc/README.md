# Docs Build

While the docs are generally built by [control.ros.org](https://github.com/ros-controls/control.ros.org), we provide an isolated docs build environment for updating and rendering docs locally.

From this directory, install the pixi environment and compile:

```bash
# Install the environment
pixi install --frozen

# Build the docs
pixi run build

# Build the docs and rebuild on changes
pixi run live-build

# Clean out the build artifacts and symlinks
pixi run clean
```
