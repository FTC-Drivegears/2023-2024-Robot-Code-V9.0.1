package org.firstinspires.ftc.teamcode.util;

import org.apache.commons.math3.filter.KalmanFilter;
import org.firstinspires.ftc.teamcode.subsystems.AprilCamSubsystem;

public class RobotLocalization {
    private AprilCamSubsystem aprilCamSubsystem;
    private GyroOdometry gyroOdometry;
    private KalmanFilter kalmanFilter;

    RobotLocalization(AprilCamSubsystem aprilCamSubsystem, GyroOdometry gyroOdometry, KalmanFilter kalmanFilter){
        this.aprilCamSubsystem = aprilCamSubsystem;
        this.gyroOdometry = gyroOdometry;
        this.kalmanFilter = kalmanFilter;
    }

    public void updateRobotPosition(int[] tagIDs, double[] tagGlobalXs, double[] tagGlobalYs) {
        for (int i = 0; i < tagIDs.length; i++) {
            // Step 1: Detect the AprilTag
            double tagCameraX = aprilCamSubsystem.getTagCenterX(tagIDs[i]);
            double tagCameraY = aprilCamSubsystem.getTagCenterY(tagIDs[i]);

            // Step 2: Convert to Robot Coordinates
            // This step requires knowledge of the transformation between the camera and robot coordinate systems
            // For simplicity, let's assume the camera is mounted at the front of the robot and aligned with the robot's forward direction
            double tagRobotX = tagCameraX;
            double tagRobotY = tagCameraY;

            // Step 3: Use Tag Position
            // If we know the tag's position in a global coordinate system (like the field), we can infer the robot's position
            // Let's assume we know the tag's global position is (tagGlobalXs[i], tagGlobalYs[i])
            double robotGlobalX = tagGlobalXs[i] - tagRobotX;
            double robotGlobalY = tagGlobalYs[i] - tagRobotY;

            // Step 4: State Estimation
            // Use a filter, such as a Kalman filter, to estimate the robot's state (position and orientation)
            kalmanFilter.update(robotGlobalX, robotGlobalY);
        }

        // Update the robot's position
        gyroOdometry.x = kalmanFilter.getX();
        gyroOdometry.y = kalmanFilter.getY();
    }
}
