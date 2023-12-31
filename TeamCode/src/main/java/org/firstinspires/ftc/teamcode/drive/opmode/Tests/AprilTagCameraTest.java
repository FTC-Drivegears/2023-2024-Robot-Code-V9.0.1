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
import java.util.concurrent.atomic.AtomicReference;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import org.firstinspires.ftc.teamcode.subsystems.AprilCamSubsystem;
import org.openftc.easyopencv.OpenCvCamera;

@TeleOp (name = "AprilTagCameraTest", group = "Tests")
public class AprilTagCameraTest extends LinearOpMode {

    AprilCamSubsystem aprilCamSubsystem;
    private FtcDashboard dashboard;
    private TelemetryPacket packet;
    private OpenCvCamera webcam;

    @Override
    public void runOpMode() throws InterruptedException {
        webcam = hardwareMap.get(OpenCvCamera.class, "Webcam 1");
        aprilCamSubsystem = new AprilCamSubsystem(hardwareMap);

        packet = new TelemetryPacket();

        waitForStart();
        while(opModeIsActive()){
            aprilCamSubsystem.runDetections();
            telemetry.addData("Detections", aprilCamSubsystem.getDetections());
            telemetry.update();
        }

    }

}
