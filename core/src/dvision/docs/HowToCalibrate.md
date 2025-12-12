How to calibrate camera.
===

1. calibrate camera intrinsic
2. calibrate camera extrinsic
3. update config


Before everything start
===
```
$ roslaunch dconfig dconfig.launch
```
So that we can have a roscore running and configs loaded

Intrinsic
===

1. Take images of chessboard
    
    ```
    $ rl dvision capture.launch
    ```
    Press `c` to capture pictures, all the image files will be located at current working directory(~/humanoid/devel/lib/dvision).

2. Copy the images from robot to your computer

3. use opencv to calibrate intrinsics
     
     ```
     $ rosrun dvision calib <image directory> [square size(mm)]
     ```
    Will use default size in calibration.cpp if no square size input.
    Just press space, wait for some moment when cv is calibrating.
    The result will be showed on the console. Copy them to dconfig/dvision/camera.yml


Extrinsic
===

1. take images
    
    ```
    $ roslaunch dbehavior default.launch skill:=GetImage
    ```

    Run this on the robot, and take many pictures with robot standing at the center of the field, facing to the goal.

2. mark points
    
    ```
    rl dcalibrator dcalibrator.launch

    ```
    Click point or use combobox to select point. Drag it to the right place, and press space in the text box to get the ouput. Press delete button to delete point that can't be seen.

    Generate a text file, make sure the intrinsic parameters we calibrated before are correctly loaded.
    

3. run matlab
    
    run `main.m`

Note
===

Remember to run `roslaunch dconfig dconfig.launch`
