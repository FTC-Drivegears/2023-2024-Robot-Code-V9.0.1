package org.firstinspires.ftc.teamcode.drive.opmode.Tests;

import android.os.Build;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.command.MecanumCommand;
import org.firstinspires.ftc.teamcode.subsystems.AprilCamSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IMUSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;

import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.util.GyroOdometry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

@Autonomous(name="coordinate testing")
public class CoordinateTesting extends LinearOpMode {
    private MecanumSubsystem mecanumSubsystem;
    private MecanumCommand mecanumCommand;
    private IMUSubsystem imu;
    private OdometrySubsystem odometrySubsystem;
    private GyroOdometry gyroOdometry;
    private AprilCamSubsystem aprilCamSubsystem;
    private ArrayList<org.firstinspires.ftc.vision.apriltag.AprilTagDetection> detections;
    private HashMap<Integer, AprilTagDetection> detectionMap;
    FtcDashboard dashboard;
    TelemetryPacket packet;
    private double targetX = 0;
    private double targetY = 0;

    private boolean goToAprilTag = false;

    private Integer aprilID = 5;

    @Override
    public void runOpMode() throws InterruptedException {
        imu = new IMUSubsystem(hardwareMap);
        mecanumSubsystem = new MecanumSubsystem(hardwareMap);
        odometrySubsystem = new OdometrySubsystem(hardwareMap);
        gyroOdometry = new GyroOdometry(odometrySubsystem, imu);
        mecanumCommand = new MecanumCommand(mecanumSubsystem, odometrySubsystem, gyroOdometry, this);
        aprilCamSubsystem = new AprilCamSubsystem(hardwareMap);


//        mecanumCommand.turnOffInternalPID();
        imu.resetAngle();
        odometrySubsystem.reset();

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();

        waitForStart();
        odometrySubsystem.reset();

        Executor executor = Executors.newFixedThreadPool(6);
        CompletableFuture.runAsync(this::updateOdometry, executor);
        CompletableFuture.runAsync(this::updateTelemetry, executor);
        CompletableFuture.runAsync(this::pidProcess, executor);
        CompletableFuture.runAsync(this::motorProcess, executor);
        CompletableFuture.runAsync(this::tagDetectionProcess, executor);

//        sleep(8000);
//        mecanumCommand.moveRotation(Math.PI);


        mecanumCommand.setFinalPosition(true, 20, -70, -20, 0);
        while(!mecanumCommand.isPositionReached(false, false)){}


        goToAprilTag = true;

        sleep(5000);

        setTagTargets();
        sleep(20000);

       /* while(opModeIsActive() && !isStopRequested()) {
            if(aprilCamSubsystem.getHashmap().containsKey(aprilID)) {
                targetY = gyroOdometry.y + aprilCamSubsystem.getAprilYDistance(aprilID, 0);
                mecanumCommand.setFinalPosition(true, 30, 0, gyroOdometry.y + aprilCamSubsystem.getAprilYDistance(aprilID, 0), 0);
            }
            while(!mecanumCommand.isPositionReached(true, true)){}
            sleep(5000);
            if(aprilCamSubsystem.getHashmap().containsKey(aprilID)){
                targetX = gyroOdometry.x + aprilCamSubsystem.getAprilXDistance(aprilID, 0) - 20;
                mecanumCommand.setFinalPosition(true, 30, targetX, gyroOdometry.y + aprilCamSubsystem.getAprilYDistance(aprilID, 0), 0);
            }
            while(!mecanumCommand.isPositionReached(true, true)){}
            sleep(5000);
        }
        */

//        sleep(4000);
//        mecanumCommand.moveToGlobalPosition(100, 100, Math.PI);
//        sleep(4000);
//        mecanumCommand.moveToGlobalPosition(100, 100, 2*Math.PI);

    }

    public void updateOdometry() {
        while (opModeIsActive()) {
            imu.gyroProcess();
            gyroOdometry.process();
        }
    }
    public void pidProcess(){
        while (opModeIsActive()) {
            mecanumCommand.pidProcess();
        }
    }

    public void motorProcess(){
        while (opModeIsActive()) {
            mecanumSubsystem.motorProcess();
        }
    }

    public void updateTelemetry() {
        while (opModeIsActive()) {
            packet.put("x", gyroOdometry.x);
            packet.put("y", gyroOdometry.y);
            packet.put("output", mecanumCommand.globalXController.getOutputPositionalValue());
            packet.put("y output", mecanumCommand.globalYController.getOutputPositionalValue());
            packet.put("theta", gyroOdometry.theta);
            packet.put("theta output", mecanumCommand.globalThetaController.getOutputPositionalValue());
            packet.put("lfvel", mecanumSubsystem.lfvel);
            packet.put("lbvel", mecanumSubsystem.lbvel);
            packet.put("rfvel", mecanumSubsystem.rfvel);
            packet.put("rbvel", mecanumSubsystem.rbvel);
            telemetry.addData("x", gyroOdometry.x);
            telemetry.addData("y", gyroOdometry.y);
            telemetry.addData("xintegral", mecanumCommand.globalXController.getIntegralSum());
            telemetry.addData("output", mecanumCommand.globalXController.getOutputPositionalValue());
            telemetry.addData("apriltagYdistance", -aprilCamSubsystem.getAprilXDistance(aprilID, 0));
            telemetry.addData("apriltagXdistance", aprilCamSubsystem.getAprilYDistance(aprilID, 0));
            telemetry.addData("targetX", targetX);
            telemetry.addData("targetY", targetY);
            telemetry.update();
            dashboard.sendTelemetryPacket(packet);
        }
    }

    public void tagDetectionProcess(){
        while(opModeIsActive()) {
            if(goToAprilTag){
                aprilCamSubsystem.runDetections();
            }
        }
    }



    public void setTagTargets(){
        while(opModeIsActive() && !isStopRequested()) {
            if(goToAprilTag) {
                /*if(aprilCamSubsystem.getHashmap().containsKey(aprilID)) {
                    targetY = gyroOdometry.y + aprilCamSubsystem.getAprilXDistance(aprilID, 0);
                    mecanumCommand.setFinalPosition(true, 20, 0, gyroOdometry.y + aprilCamSubsystem.getAprilYDistance(aprilID, 0), 0);
                }
                */
                //while(!mecanumCommand.isPositionReached(false, false)){}
                sleep(1000);
                if(aprilCamSubsystem.getHashmap().containsKey(aprilID)){


                    targetY = gyroOdometry.y - aprilCamSubsystem.getAprilXDistance(aprilID, -5);
                    targetX = gyroOdometry.x + aprilCamSubsystem.getAprilYDistance(aprilID, -8);

                    mecanumCommand.setFinalPosition(true, 20, targetX, targetY, 0);
                    while(!mecanumCommand.isPositionReached(false, false)){}
                    goToAprilTag = false;
                }
            }
        }
    }


}