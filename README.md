# CGAIK

>This page is actively being edited

Closed-form geometric inverse kinematics (IK) solver based on conformal geometric algebra (CGA), a powerful mathematical framework for representing geometry. 

CGA, in a simple and elegant way, provides numerous geometric insights into the robot that are difficult or nearly impossible to achieve using traditional matrix methods. 

The IK solver have been implemented in ROS 2 C++ and tested on two robots:
- 6-DoF collaborative robotic arm (TM5-700 and TM5-900)
- 3-DoF spherical parallel manipulator (SPM).

## Table of Contents

- [CGAIK](https://github.com/wei-hsuan-cheng/cga_ik.git)
    - [Table of Contents](#table-of-contents)
    - [Installation](#installation)
    - [Demo](#demo)

## Installation

```bash
cd ~/ros2_ws/src && git clone https://github.com/wei-hsuan-cheng/cga_ik.git

cd ~/ros2_ws && rosdep update && rosdep install --from-paths src --ignore-src -r -y

cd ~/ros2_ws && colcon build --packages-select cga_ik && . install/setup.bash
```

## Demo

Run the closed-form IK solver demo codes for the two robots:
```bash
# Source your workspace
cd ~/ros2_ws && . install/setup.bash

# For cobot_6dof
ros2 launch cga_ik visualise_cobot_6dof.launch.py

# For spm_3dof
ros2 launch cga_ik visualise_spm_3dof.launch.py
```

Then, you will see the robots visualised in RViz2.
<p align="center">
  <table>
    <tr>
      <td align="center">
        <img src="/images/cobot_6dof_rviz.png" height="250px"/>
        <br/>
        <sub><b>6-DoF Collaborative Arm</b></sub>
      </td>
      <td align="center">
        <img src="/images/spm_3dof_rviz.png" height="250px"/>
        <br/>
        <sub><b>3-DoF Spherical Parallel Manipulator</b></sub>
      </td>
    </tr>
  </table>
</p>




