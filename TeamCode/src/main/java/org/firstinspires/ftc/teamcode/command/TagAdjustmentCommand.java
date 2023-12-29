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

    //method that calculates where the april tag is in comparison to where the robot is
    public Errors tagAdjustment(int tagID){
        double xPixel = aprilCamSubsystem.getTagCenterX(tagID);
        double yPixel = aprilCamSubsystem.getTagCenterY(tagID);

        // Camera properties
        double cameraFieldOfView = ...; // replace with actual value
        double cameraResolution = ...; // replace with actual value

        // Calculate field of view per pixel
        double fieldOfViewPerPixel = cameraFieldOfView / cameraResolution;

        // Calculate angle to the tag
        double xAngle = (xPixel - cameraResolution / 2) * fieldOfViewPerPixel;
        double yAngle = (yPixel - cameraResolution / 2) * fieldOfViewPerPixel;

        // Assuming you know the actual distance to the tag
        double distanceToTag = ...; // replace with actual value

        // Calculate displacement
        double xDisplacement = distanceToTag * Math.tan(xAngle);
        double yDisplacement = distanceToTag * Math.tan(yAngle);

        // Calculate error
        double xError = xDisplacement - gyroOdometry.x;
        double yError = yDisplacement - gyroOdometry.y;

        return new Errors(xError, yError);
    }

    //take errors and translate it to new coordinates to move to
    public void moveToTag(int tagID) {

        //error is currently in camera units
        Errors errors = tagAdjustment(tagID);
        double xError = errors.getXError();
        double yError = errors.getYError();

        //constant for scaling the error to match centimeters
        double scale = 0.1;


        //

        double theta = gyroOdometry.theta;

        mecanumCommand.setFinalPosition(true, 30, x, y, theta);
    }

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
