# Rosbag RViz Panel (ROS2-FOXY)

## Overview

This package provides a custom RViz panel for playing rosbags with enhanced functionalities. The panel allows users to play rosbags forwards and adjust play speeds, while also displaying some relevant information such as the date of the messages or the total size of the bag. Additionally, it features a custom progress bar that users can interact with.

![Alt Text](docs/rviz_view.gif)

## Features

- **Play Controls:** Play or pause the data at will.
- **Speed Adjustment:** Change the play speed, from 0.5x to 10x.
- **Information Display:**
  - Timestamp of the current playhead location.
  - Date and time of the current playhead location.
  - Number of seconds the current playhead location is from the beginning of the bag.
  - Playback speed.
  - Total size of the bag.
- **Interactive Progress Bar:** Users can interact with the custom progress bar to navigate within the rosbag.

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