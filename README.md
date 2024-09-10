# Quadrotor Autopilot With Feed-Forward Control

## Hello there 👋

This Matlab-Simulink-deployed autopilot is designed as a replacement for the stock PX4 flight controller and guidance system. I implemented
full-state feed-forward control together with the minimum-snap real-time trajectory guidance algorithm. 

<p float="center">
  <img src="git_figures/figure_8_example.gif" width="48%" />
  <img align="right" src="git_figures/replanning_example.gif" width="48%" /> 
</p>

# Integration with PX4
This work is closely related to my other project, the control system deployment package for [Tilt-wing aircraft](https://github.com/YevheniiKovryzhenko/RRTV_TiltWing.git). 
This project is intended for use with my [PX4 Simulink I/O Framework](https://github.com/YevheniiKovryzhenko/PX4_SIMULINK_IO_Framework.git), and it will not work with the stock PX4.
You may also find [KGroundControl](https://github.com/YevheniiKovryzhenko/KGroundControl.git) useful for your communication-related tasks. 

Check out my tutorials on installing and configuring this framework on [YouTube](https://www.youtube.com/playlist?list=PLgxIoIw6ONumELvafBonHBceTzPIK5T5j)!

# Installation 
This is a standalone project that includes the control system and only requires installation of [PX4 Simulink I/O Framework](https://github.com/YevheniiKovryzhenko/PX4_SIMULINK_IO_Framework.git), you must retrieve the control system package from any of the following repositories.
Once done, all you need is to:
* Run the PX4_Simulink_IO_Quadcopter_Min_Snap_FF_control.prj file from Matlab to initialize everything.
* Use appropriate project shortcuts to launch GCS and Hardware-deployed control system.  

For control system deployment: 
1. In MATLAB, go to the "Project Shortcuts" tab and click on the "Open Hardware Model". This will open the correct, hardware-ready, Simulink model for controller deployment.
2. In Simulink, go to the "HARDWARE" tab and click on the right-most green icon "Build Deploy and Start". Follow the on-screen prompts to upload the compiled firmware to the Pixhawk board.

For Ground Control Station:
1. Make sure you have installed and correctly configured [KGroundControl](https://github.com/YevheniiKovryzhenko/KGroundControl.git).
2. Make sure you have all joysticks wired up correctly, otherwise comment out the TX block. You may need to manually re-configure the joystick and pedal setup whenever you unplug those or even reboot the computer.  
3. In MATLAB, go to the "Project Shortcuts" tab and click on the "Open GCS" tab. This will open the correct Simulink model for GCS.

# Contact
If you have any questions, please feel free to contact me, Yevhenii (Jack) Kovryzhenko, at yzk0058@auburn.edu.

# Credit
This work started during my Ph.D. at [ACELAB](https://etaheri0.wixsite.com/acelabauburnuni) at Auburn University, under the supervision of Dr. Ehsan Taheri.
For more details, check out my [KGroundControl](https://github.com/YevheniiKovryzhenko/KGroundControl.git) and [PX4 Simulink I/O Framework](https://github.com/YevheniiKovryzhenko/PX4_SIMULINK_IO_Framework.git) repositories that
were all part of this project. 

I am still in the process of publishing journal papers that have directly used this work, so I will keep this section actively updated. Feel free to credit [me](https://scholar.google.com/citations?user=P812qiUAAAAJ&hl=en) by citing any of my relevant works.
Some of the articles that directly used this work:
* Kovryzhenko, Y., Li, N., and Taheri, E., “A Control System Design and Implementation for Autonomous Quadrotors with Real-Time Re-Planning Capability,” Robotics, Vol. 13, No. 9, 2024, p. 136. https://doi.org/10.3390/robotics13090136
    ```
  	@article{kovryzhenko_control_2024,
    	title = {A Control System Design and Implementation for Autonomous Quadrotors with Real-Time Re-Planning Capability},
    	volume = {13},
    	copyright = {http://creativecommons.org/licenses/by/3.0/},
    	issn = {2218-6581},
    	url = {https://www.mdpi.com/2218-6581/13/9/136},
    	doi = {10.3390/robotics13090136},
    	language = {en},
    	number = {9},
    	urldate = {2024-09-09},
    	journal = {Robotics},
    	author = {Kovryzhenko, Yevhenii and Li, Nan and Taheri, Ehsan},
    	month = sep,
    	year = {2024},
    	keywords = {autopilot development, embedded computing, hardware pipeline, online re-planning, PX4, real-time control, UAS},
    	pages = {136},
    }
  	```
