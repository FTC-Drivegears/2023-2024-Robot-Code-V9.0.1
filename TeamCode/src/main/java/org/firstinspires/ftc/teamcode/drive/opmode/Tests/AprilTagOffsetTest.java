package org.firstinspires.ftc.teamcode.drive.opmode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Objects;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.atomic.AtomicReference;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.command.MecanumCommand;
import org.firstinspires.ftc.teamcode.subsystems.IMUSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.util.GyroOdometry;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import org.firstinspires.ftc.teamcode.subsystems.AprilCamSubsystem;
import org.openftc.easyopencv.OpenCvCamera;

@TeleOp (name = "AprilTagOffsetTest", group = "Tests")
public class AprilTagOffsetTest extends LinearOpMode {

    AprilCamSubsystem aprilCamSubsystem;
    private FtcDashboard dashboard;
    private TelemetryPacket packet;
    private OpenCvCamera webcam;
    private MecanumSubsystem mecanumSubsystem;
    private MecanumCommand mecanumCommand;
    private IMUSubsystem imu;
    private OdometrySubsystem odometrySubsystem;
    private GyroOdometry gyroOdometry;
    private ArrayList<AprilTagDetection> detections;
    private HashMap<Integer, AprilTagDetection> detectionMap;

    @Override
    public void runOpMode() throws InterruptedException {
        aprilCamSubsystem = new AprilCamSubsystem(hardwareMap);

        packet = new TelemetryPacket();
        imu = new IMUSubsystem(hardwareMap);
        mecanumSubsystem = new MecanumSubsystem(hardwareMap);
        odometrySubsystem = new OdometrySubsystem(hardwareMap);
        gyroOdometry = new GyroOdometry(odometrySubsystem, imu);
        mecanumCommand = new MecanumCommand(mecanumSubsystem, odometrySubsystem, gyroOdometry, this);

        waitForStart();

        CompletableFuture.runAsync(this::tagDetectionProcess);
        CompletableFuture.runAsync(this::updateTelemetry);
        CompletableFuture.runAsync(this::updateOdometry);

        while(!isStopRequested() && opModeIsActive()) {
        }

    }
    public void tagDetectionProcess(){
        while(opModeIsActive()) {
            aprilCamSubsystem.runDetections();
            detections = aprilCamSubsystem.getDetections();
            detectionMap = aprilCamSubsystem.getHashmap();
        }
    }

    public void updateTelemetry(){
        while(opModeIsActive()) {
            telemetry.addData("x", gyroOdometry.x);
            telemetry.addData("y", gyroOdometry.y);
            telemetry.addData("theta", gyroOdometry.theta);
            telemetry.addData("detections", detections);
            telemetry.addData("detectionMap", detectionMap);
            for (int id : detectionMap.keySet()) {
                telemetry.addLine("ID: " + id);
                telemetry.addData("april x", Objects.requireNonNull(detectionMap.get(id)).ftcPose.x);
                telemetry.addData("april y", Objects.requireNonNull(detectionMap.get(id)).ftcPose.y);
                telemetry.addData("april z", Objects.requireNonNull(detectionMap.get(id)).ftcPose.z);
                telemetry.addData("transformed x", aprilCamSubsystem.getAprilXDistance(id, 0));
                telemetry.addData("transformed y", aprilCamSubsystem.getAprilYDistance(id, 0));
//                telemetry.addData("targetx", gyroOdometry.x + aprilCamSubsystem.getAprilXDistance())
            }
            telemetry.update();
        }
    }

    public void updateOdometry(){
        while(opModeIsActive()) {
            imu.gyroProcess();
            gyroOdometry.process();
        }
    }

}
