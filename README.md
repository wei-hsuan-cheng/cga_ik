# CGAIK

>This page is actively being edited

Closed-form geometric inverse kinematics (IK) solver based on conformal geometric algebra (CGA), a powerful mathematical framework for representing geometry. 

CGA, in a simple and elegant way, provides numerous geometric insights into the robot that are difficult or nearly impossible to achieve using traditional matrix methods. 

The IK solver have been implemented in ROS 2 C++ and tested on two robots:
- 6-DoF collaborative arms (TM5-700 and TM5-900)
- 3-DoF spherical parallel manipulator (SPM).

## Table of Contents

- [CGAIK](https://github.com/wei-hsuan-cheng/cga_ik.git)
    - [Table of Contents](#table-of-contents)
    - [Installation](#installation)
    - [Demo](#demo)
    - [Bibliography](#bibliography)
    - [Acknowledgements](#acknowledgements)
    - [Contact](#contact)

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

## Bibliography
- CGA class:
  - [1] https://github.com/enkimute/ganja.js/blob/master/codegen/cpp/cga.cpp
- Cobot IK solution:
  - [1] https://www.mic-journal.no/ABS/MIC-2016-1-6.asp/
  - [2] https://www.sciencedirect.com/science/article/pii/S0094114X22001045
  - [3] https://slides.com/hugohadfield/game2020
- SPM IK solution:
  - [1] https://slides.com/hugohadfield/game2020
  - [2] https://enkimute.github.io/ganja.js/examples/coffeeshop.html#2DmBscfSXO

## Acknowledgements

- **ganja.js**: The CGA class is an implementation adapted from the C++ template provided by [ganja.js](https://github.com/enkimute/ganja.js), an open-source geometric algebra library.
- **Hugo's slides at GAME2020**: A lot of notations and implementations in this work is inspired by the work of [Hugo Hadfield](https://hh409.user.srcf.net) and his [slides at GAME2020](https://slides.com/hugohadfield/game2020).
- **Robot Math Utils**: Some utilities from [Robot Math Utils](https://github.com/wei-hsuan-cheng/robot_math_utils) are used.
- **Eigen Library**: This library heavily relies on the Eigen library for linear algebra operations.

## Contact

- **Author**: Wei-Hsuan Cheng [(johnathancheng0125@gmail.com)](mailto:johnathancheng0125@gmail.com)
- **Homepage**: [wei-hsuan-cheng](https://wei-hsuan-cheng.github.io)
- **GitHub**: [wei-hsuan-cheng](https://github.com/wei-hsuan-cheng)






