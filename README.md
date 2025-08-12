# Baxter_Robot_Motion_Planning_Algorithm

### About

For my final project in my robotics class in grad school, I was tasked with creating a motion planning algorithm for a baxter robot arm which had 7 degrees of freedom (DoF). I created the algorithm using MATLAB. Each arm had to be individually manipulated such that the path of the robot end effector (hand)  was purely rotational and the orientaion abided by the constraints of SO(3) motion (in layman's terms, the orientation had to be logical. Think about how your hand stays fixed while opening a door or how it revolves while turning a door handle). The task I chose was to have the robot arm reach for a door handle, turn it, then swing the door open. The path generated was very accurate. Final results for position were accurate to within 1 mm and the orientation was accurate to 0.001 radians. A video of the generated path and the final report are uploaded in the main folder.
