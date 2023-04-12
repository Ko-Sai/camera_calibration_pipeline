# camera_calibration_pipeline

There are two python script files, one for calibration(camera_calibration.py) and another one for validating the output matrix(camera_validation.py).
These scripts need to be run in python3.

### Software Requirements

```pip3 install opencv-contrib-python```

### Hardware requirement

#### Setup

- a flat clear pattern checkerboard with (12x8) inner corners
 - a stand on which cameras can be mounted vertically
 - two stands on which size of 7cm aruco tag with (4x4x50) configuration can be sticked
 - a closed clean environment with uniform light condition for camera calibration
 
### Usage Instruction
 
 - You can mount 3 cameras on the stand vertically and do the calibration at the same time. 
 - You need to make sure the camera lens are clear, if not, you need to wipe them off.
 
 For camera calibration, you can follow the steps below.
 
#### Steps

1) First, generate aruco markers in this link (https://chev.me/arucogen/) . Use the parameters as shown below.

![](/images/aruco.png) 

And print two arcuo markers on A4 paper and stick each paper on each stand.

![](/images/two_stands.jpeg) 

2) Mount the cameras on the stand and plug the cameras into your computer one by one. 

![](/images/camera_mount.jpeg) 

3) Run the calibration script(camera_calibration.py). You can provide the arguments as shown in the photo. *(Normally you just need to provide the cameras' number that you want to do calibration, others are optional)*
1. Camera number for calibration
2. Name of directory of calibrated data 
3. Name of directory of each camera data
4. Name of output yaml file

***You must provide the camera number accordingly with the label on the camera in order.***

![](/images/calibration_script.png) 

4) After running the script, you can start the calibration. Basically, the images should cover different distances, different perspectives to get a better calibration.

![](/images/config_photos.png) 

![](/images/checkerboard_config.png) 


![](/images/table_config1.png) 
![](/images/table_config2.png) 

 5) After covering all necessary configurations, you need to press “space bar” to stop the recording. In the meantime, you need to move two stands with aruco marker to 50cm and 100cm distance respectively from the camera.***(Make sure the two stands dont overlap)***. Again, you need to press “space bar” to proceed to the next step.
 
 6) You can check the terminal for the information of what the script is doing at the moment. There are two validation methods implemented in the script. 
 i)The first one will be checking with arcuo markers' stand which you previously move in the sight of camera view. Once calibration is done, it will automatically validate the result.
 ii)The second one, you need to place the checkerboard at 150cm from the camera and pressed "a" to validate the result. You need to put the checkerboard in two extreme sides and in the middle. Then press "space bar" to stop the validation.
 
 ![](/images/terminal_result.png) 
 
 You can see whether the calibration matrix for each camera is good or not. According to the printing result, you can decide that either you keep the matrix or you redo the calibration.
 
 7) After calibration is done, you can run the validation script(camera_validation.py) to visually check the performance of the camera matrix. Here also, you can provide the same 4 arguments as in calibration script. You can also see aruco distance in the terminal printing. You can move one aruco marker stand to check the performance. 
 
![](/images/validation_script.png) 
 
 
 For visualization, it is better to have a obvious horizontal straight object and a vertical straight object to check the performance.
 
 ![](/images/two_stands_validation2.png) 
 
 
   
