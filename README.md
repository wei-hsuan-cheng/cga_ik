# CGAIK

>This page is actively being edited

Closed-form geometric inverse kinematics (IK) solver based on conformal geometric algebra (CGA), a powerful mathematical framework for representing geometry. 

CGA, in a simple and elegant way, provides numerous geometric insights into the robot that are difficult or nearly impossible to achieve using traditional matrix methods. 

The IK solver have been implemented in ROS 2 humble C++ and tested on two robots:
- 6-DoF collaborative arms (TM5-700 and TM5-900)
- 3-DoF spherical parallel manipulator (SPM, or agile eye).

## Table of Contents

- [CGAIK](https://github.com/wei-hsuan-cheng/cga_ik.git)
    - [Table of Contents](#table-of-contents)
    - [Installation](#installation)
    - [Demo](#demo)
    - [Bibliography](#bibliography)
    - [Acknowledgements](#acknowledgements)
    - [Contact](#contact)

## Installation

Clone and build the [`cga_ik_action_interfaces`](https://github.com/wei-hsuan-cheng/cga_ik_action_interfaces) pkg first, where the `cga_ik` pkg relies on it:
```bash
cd ~/ros2_ws/src && git clone https://github.com/wei-hsuan-cheng/cga_ik_action_interfaces.git

cd ~/ros2_ws && rosdep update && rosdep install --from-paths src --ignore-src -r -y

cd ~/ros2_ws && colcon build --packages-select cga_ik_action_interfaces && . install/setup.bash
```


Then, clone and build the `cga_ik` pkg:
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

# Visualise cobot_6dof and spm_3dof
ros2 launch cga_ik visualise_cobot_6dof.launch.py
ros2 launch cga_ik visualise_spm_3dof.launch.py

# Control the spm_3dof (action server)
ros2 launch cga_ik control_spm.launch.py use_fake_hardware:=true # false if control real robot
ros2 action send_goal /spm cga_ik_action_interfaces/action/SPM "{start: true}" # Action client
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
  - [1] [Christian Perwass. Geometric Algebra with Applications in Engineering. Springer Series in Geometry and Computing. Springer, 2009. isbn: 9783540890676.](https://link.springer.com/content/pdf/10.1007/978-3-540-89068-3.pdf)
  - [2] [ganja.js](https://github.com/enkimute/ganja.js)
- Cobot IK solution:
  - [1] [A. Kleppe and O. Egeland. Inverse kinematics for industrial robots using conformal geometric algebra. Modeling, Identification and Control. 2016.](https://www.mic-journal.no/ABS/MIC-2016-1-6.asp/)
  - [2] [I. Zaplana, H. Hadfield, and J. Lasenby, “Closed-form solutions for the inverse kinematics of serial robots using conformal geometric algebra,” Mech. Mach. Theory, vol. 173, Jul. 2022, Art. no. 104835.](https://www.sciencedirect.com/science/article/pii/S0094114X22001045)
  - [3] [Slides at GAME2020: Robots, Ganja and Screw Theory](https://slides.com/hugohadfield/game2020)
- SPM IK solution:
  - [1] [S. Sadeqi, S. P. Bourgeois, E. J. Park, and S. Arzanpour, "Design and performance analysis of a 3-RRR spherical parallel manipulator for hip exoskeleton applications," J. Rehabil. Assistive Technol. Eng., vol. 4, Sep. 2017, Art. no. 2055668317697596.](https://journals.sagepub.com/doi/abs/10.1177/2055668317697596)
  - [2] [S. Bai, M. R. Hansen, and T. O. Andersen, “Modelling of a special class of spherical parallel manipulators with Euler parameters,” Robotica, vol. 27, no. 2, pp. 161–170, 2008.](https://www.cambridge.org/core/journals/robotica/article/modelling-of-a-special-class-of-spherical-parallel-manipulators-with-euler-parameters/5B8BE86739A98697E82C818660C020D4)
  - [3] [Slides at GAME2020: Robots, Ganja and Screw Theory](https://slides.com/hugohadfield/game2020)
  - [4] [JavaScript visualisation of 3-DoF agile eye](https://enkimute.github.io/ganja.js/examples/coffeeshop.html#2DmBscfSXO)
  

## Acknowledgements

- **ganja.js**: The CGA class is an implementation adapted from the C++ template provided by [ganja.js](https://github.com/enkimute/ganja.js), an open-source geometric algebra library.
- **Hugo's slides at GAME2020**: A lot of notations and implementations in this work are inspired by the work of [Hugo Hadfield](https://hh409.user.srcf.net) and his [slides at GAME2020](https://slides.com/hugohadfield/game2020).
- **Robot Math Utils**: Some utilities from [Robot Math Utils](https://github.com/wei-hsuan-cheng/robot_math_utils) are used.
- **Eigen Library**: This library heavily relies on the Eigen library for linear algebra operations.

## Contact

- **Author**: Wei-Hsuan Cheng [(johnathancheng0125@gmail.com)](mailto:johnathancheng0125@gmail.com)
- **Homepage**: [wei-hsuan-cheng](https://wei-hsuan-cheng.github.io)
- **GitHub**: [wei-hsuan-cheng](https://github.com/wei-hsuan-cheng)






