# cobot
Repo for cobot research using C++ and Python. The current setup only works for Ubuntu 16.04 LTS and ROS Kinetic. 
We are planning to extend this work for Ubuntu 18.04 and ROS Melodic.

## Some thing I learned along the way:
When we want to connect an external monitor to a laptop, screen will freeze. The mouse cursor can still move but we 
cannot click anything on the screen. To fix this, we need to install a VGA driver. Here is the step:
1. Figure out what VGA card model we are using. Type in terminal
 ```
 lspci | grep VGA
 ```
   Output will be something like this:
 
 ```
 01:00.0 VGA compatible controller: NVIDIA Corporation GT218M [NVS 3100M] (rev a2)
 ```
 2. Go to NVIDIA Driver Download page [here](https://www.nvidia.com/Download/index.aspx) and provide information you 
 have above.
 ![NVIDIA SCREENSHOT1](assets/nvidia_screenshot1.png?raw=true "NVIDIA SCREENSHOT1")
 
 3. We then get driver version from the next screen. Note this version number because we will use it in the next step.
 ![NVIDIA SCREENSHOT2](assets/nvidia_screenshot2.png?raw=true "NVIDIA SCREENSHOT2")
 
 4. Install the correct driver. Type in terminal
 ```
 sudo apt-get install nvidia-340
 reboot
 ```
 That's it! Now we can use an external monitor without problem.
