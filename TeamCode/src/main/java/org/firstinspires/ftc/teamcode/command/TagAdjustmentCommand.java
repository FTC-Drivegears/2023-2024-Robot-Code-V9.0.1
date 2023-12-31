package org.firstinspires.ftc.teamcode.command;

import org.firstinspires.ftc.teamcode.subsystems.AprilCamSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.util.GyroOdometry;

public class TagAdjustmentCommand {
    private AprilCamSubsystem aprilCamSubsystem;
    private GyroOdometry gyroOdometry;
    private MecanumCommand mecanumCommand;

    TagAdjustmentCommand(AprilCamSubsystem aprilCamSubsystem, GyroOdometry gyroOdometry, MecanumCommand mecanumCommand){
        this.aprilCamSubsystem = aprilCamSubsystem;
        this.gyroOdometry = gyroOdometry;
        this.mecanumCommand = mecanumCommand;
    }

    public double convertPixelsToCentimeters(double pixelSize, double imageSize, double distanceToObject) {
        return (pixelSize / imageSize) * distanceToObject * 100;
    }

    //method that calculates where the april tag is in comparison to where the robot is
    public Errors tagAdjustmentErrors(int tagID){
        double tagCenterX = aprilCamSubsystem.getTagCenterX(tagID);
        double tagCenterY = aprilCamSubsystem.getTagCenterY(tagID);

        // Get the distance to the tag
        double distanceToObject = aprilCamSubsystem.getDistanceToTag(tagID);

        double xError = convertPixelsToCentimeters(tagCenterX, AprilCamSubsystem.VIEW_WIDTH, distanceToObject);
        double yError = convertPixelsToCentimeters(tagCenterY, AprilCamSubsystem.VIEW_HEIGHT, distanceToObject);

        return new Errors(xError, yError);
    }

    //To use multiple AprilTags for state estimation, you can follow these steps:
    //
    //1. Detect all visible AprilTags: Use the `AprilCamSubsystem` to detect all visible AprilTags in the camera's field of view. The AprilTag detection will provide you with each tag's position in the camera's coordinate system.
    //
    //2. Convert to Robot Coordinates: Convert each tag's position from the camera's coordinate system to the robot's coordinate system. This will require knowledge of the transformation between the camera and the robot coordinate systems, which typically involves a rotation and a translation.
    //
    //3. Use Tag Positions: Once you have each tag's position in the robot's coordinate system, you can use this information to determine the robot's position. If you know each tag's position in a global coordinate system (like the field on which the robot is operating), you can infer the robot's position relative to that global coordinate system.
    //
    //4. State Estimation: Use a filter, such as a Kalman filter or a particle filter, to estimate the robot's state (position and orientation) based on the positions of the detected tags. The filter will help you combine the information from multiple tags and deal with uncertainties in the tag detections and the robot's motion.
    //
    //Here's a simple implementation of these steps in Java:
    //
    //```java
    //public class RobotLocalization {
    //    private AprilCamSubsystem aprilCamSubsystem;
    //    private GyroOdometry gyroOdometry;
    //    private KalmanFilter kalmanFilter;
    //
    //    RobotLocalization(AprilCamSubsystem aprilCamSubsystem, GyroOdometry gyroOdometry, KalmanFilter kalmanFilter){
    //        this.aprilCamSubsystem = aprilCamSubsystem;
    //        this.gyroOdometry = gyroOdometry;
    //        this.kalmanFilter = kalmanFilter;
    //    }
    //
    //    public void updateRobotPosition(int[] tagIDs, double[] tagGlobalXs, double[] tagGlobalYs) {
    //        for (int i = 0; i < tagIDs.length; i++) {
    //            // Step 1: Detect the AprilTag
    //            double tagCameraX = aprilCamSubsystem.getTagCenterX(tagIDs[i]);
    //            double tagCameraY = aprilCamSubsystem.getTagCenterY(tagIDs[i]);
    //
    //            // Step 2: Convert to Robot Coordinates
    //            // This step requires knowledge of the transformation between the camera and robot coordinate systems
    //            // For simplicity, let's assume the camera is mounted at the front of the robot and aligned with the robot's forward direction
    //            double tagRobotX = tagCameraX;
    //            double tagRobotY = tagCameraY;
    //
    //            // Step 3: Use Tag Position
    //            // If we know the tag's position in a global coordinate system (like the field), we can infer the robot's position
    //            // Let's assume we know the tag's global position is (tagGlobalXs[i], tagGlobalYs[i])
    //            double robotGlobalX = tagGlobalXs[i] - tagRobotX;
    //            double robotGlobalY = tagGlobalYs[i] - tagRobotY;
    //
    //            // Step 4: State Estimation
    //            // Use a filter, such as a Kalman filter, to estimate the robot's state (position and orientation)
    //            kalmanFilter.update(robotGlobalX, robotGlobalY);
    //        }
    //
    //        // Update the robot's position
    //        gyroOdometry.x = kalmanFilter.getX();
    //        gyroOdometry.y = kalmanFilter.getY();
    //    }
    //}
    //```
    //
    //This code assumes that the camera is mounted at the front of the robot and aligned with the robot's forward direction. If your camera is mounted differently, you'll need to adjust the transformation from camera coordinates to robot coordinates accordingly. Also, this code assumes that you have implemented a `KalmanFilter` class. If you're using a different filter for state estimation, you'll need to adjust the code accordingly.

    //take errors and translate it to new coordinates to move to

}

class Errors {
    private double xError;
    private double yError;

    Errors(double xError, double yError){
        this.xError = xError;
        this.yError = yError;
    }

    public double getXError(){
        return xError;
    }

    public double getYError(){
        return yError;
    }
}
