package org.firstinspires.ftc.teamcode.drive.opmode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.WebcamSubsystem;
import org.opencv.core.Point;

@TeleOp(name="camera test")
public class CameraTest extends LinearOpMode {
    //https://docs.wpilib.org/en/stable/docs/software/vision-processing/grip/introduction-to-grip.html
    private WebcamSubsystem webcamSubsystem;

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        webcamSubsystem = new WebcamSubsystem(
                hardwareMap,
                WebcamSubsystem.PipelineName.CONTOUR_RED,
                dashboard, gamepad1
        );
        dashboard.startCameraStream(webcamSubsystem.webcam, 24);

        waitForStart();
        while(opModeIsActive()){
            /*
            Go to http://192.168.43.1:8080/dash to see camera work
             */
            TelemetryPacket packet = new TelemetryPacket();
            Point largestContourCenter = webcamSubsystem
                    .getContourProcessor()
                    .largestContourCenter();
            packet.put("x", largestContourCenter.x);
            packet.put("y", largestContourCenter.y);
            packet.put("area", webcamSubsystem.getContourProcessor().largestContourArea());
            packet.put("Spike Position", findSpikePosition());
            packet.put("Camera Status", webcamSubsystem.getCameraState());
            packet.put("Button A Pressed?", gamepad1.a);
            dashboard.sendTelemetryPacket(packet);
            sleep(20);
        }
    }

    /**
     * Identifies what spike location the game piece is on.
     *
     * @return a String indication the location of the game piece
     */
    public String findSpikePosition() {
        double center = webcamSubsystem.getXProp();

        // TODO: Tune numbers so we can find the position of the spike
        // For reference, the camera is 864 pixels wide
        if(webcamSubsystem.getContourProcessor().largestContourArea() < 2500){
            return "right";
        }
        return center < 550 ? "left"
                : center < 800 ? "middle"
                : "right";
    }

}
