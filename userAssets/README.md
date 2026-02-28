# 3-D Render for Advantage Scope

How to set up your Advantage Scope to include the 3D Render of the Robot

## Finding the Assets

In this folder (userAssests) there is another folder, Robot_2026. Hover over the folder and copy the path, it should look something like this if you paste it.

``` C:\REPO\Robot2026\userAssets\Robot_2026 ```
## Putting the Assets in Advantage Scope

Open Advantage Scope and in the top left click ```App```, then ```Show Assets Folder```. If at this point you already see a folder named ```Robot_2026```, you are already set up and you do not need to do this.

Now open up a new tab in your Windows File Exporer and paste your relative path in the top path bar (before doing this it should just say Home).

This should open up the ```Robot_2026 folder```. Now copy the folder and paste it back in the Advantage Scope User Assets folder.

Close it and go back into Advantage Scope.

## Bringing up the Model in 3D Sim

Now that you are back in Advantage Scope Simulate Robot Code in WPILIB and connect to the Sim.

On the Left side column find ```/AdvantageKit/RealOutputs/Odometry/Robot``` and drag the Robot Pose2D into the Poses tab at the bottom of your screen below the 3D Field. 

If you see KitBot 2026 or any other model that isn't 930 2026, right click the pose and select 930 2026.

Now scroll down and find ```/AdvantageKit/RealOutputs/RobotState``` open it up and open the Turret first, drag in the measured Pose3D as a Component of the Robot Pose(drag it into the robot Pose) and do the same for Extender.

It should now be set up correctly, you may want to test it out to make sure each moving part does what its supposed to at each state, when you want it to.