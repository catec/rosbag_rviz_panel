# Rosbag RViz Panel (ROS2-FOXY)

## Overview

*This is still an **EARLY VERSION**, so it currently lacks some of the features available in the ros-noetic version of the 'main' branch.*

## Dependencies installation

---

### Qt5 (By default on Ubuntu 20.04)
```
sudo apt-get install qt5-default
```

### ROS (Robot Operative System)

Follow up the ROS distro installation guide: [Foxy](https://docs.ros.org/en/foxy/Installation.html)

### (Optional) Clang-Format 16

```
sudo add-apt-repository 'deb http://apt.llvm.org/focal/ llvm-toolchain-focal main'
sudo apt update
sudo apt install clang-format-16
```

## Usage

1. Clone this repository into your ROS workspace and compile it using cmake:
    ```
    mkdir -p your_ros_ws/src && your_ros_ws/src && git clone -b foxy https://github.com/fada-catec/rosbag_rviz_panel.git

    cd .. && colcon build --packages-select rosbag_rviz_panel
    ```

1. Source the workspace and launch RViz:

    ```bash
    source install/setup.zsh && rviz2
    ```

2. Add the custom panel to your RViz layout:

    - Click on the "Panels" tab in RViz.
    - Select "Add New Panel" and choose it from the list.

3. Load a compatible rosbag2 file using the controls in the custom panel.

4. Interact with the progress bar to navigate within the rosbag.

## Help / Contribution

* Contact: **José Manuel González Marín** (jmgonzalez@catec.aero)

![CATEC](./docs/CATEC-ATLAS.png)

* Found a bug? Create an ISSUE!

* Do you want to contribute? Create a PULL-REQUEST!

---
---

![MEME](./docs/meme.gif)