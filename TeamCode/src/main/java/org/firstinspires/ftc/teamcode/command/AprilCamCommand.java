package org.firstinspires.ftc.teamcode.command;

import org.firstinspires.ftc.teamcode.subsystems.AprilCamSubsystem;
import org.firstinspires.ftc.teamcode.util.GyroOdometry;

public class AprilCamCommand {
    private AprilCamSubsystem aprilCamSubsystem;
    private GyroOdometry gyroOdometry;
    private double xOffset = 10;
    private double yOffset = 10;
    public AprilCamCommand(AprilCamSubsystem aprilCamSubsystem, GyroOdometry gyroOdometry){
        this.aprilCamSubsystem = aprilCamSubsystem;
        this.gyroOdometry = gyroOdometry;
    }

    public void setOffset(double xOffset, double yOffset){
        this.xOffset = xOffset;
        this.yOffset = yOffset;
    }

    public double getxOffset(){
        return xOffset;
    }

    public double getyOffset(){
        return yOffset;
    }

    public double getTargetXRed(int aprilID){
        return gyroOdometry.x - aprilCamSubsystem.getAprilXDistance(aprilID, xOffset);
    }

    public double getTargetYRed(int aprilID){
        return gyroOdometry.y - aprilCamSubsystem.getAprilYDistance(aprilID, yOffset);
    }

    public double getTargetXBlue(int aprilID){
        return gyroOdometry.x + aprilCamSubsystem.getAprilXDistance(aprilID, xOffset);
    }

    public double getTargetYBlue(int aprilID){
        return gyroOdometry.y + aprilCamSubsystem.getAprilYDistance(aprilID, yOffset);
    }

}
