package org.firstinspires.ftc.teamcode.drive.opmode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.WebcamSubsystem;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

@TeleOp(name="camera test")
public class CameraTest extends LinearOpMode {
    //https://docs.wpilib.org/en/stable/docs/software/vision-processing/grip/introduction-to-grip.html
    private WebcamSubsystem camera;
    private FtcDashboard dashboard;
    private TelemetryPacket packet;
    @Override
    public void runOpMode(){
        camera = new WebcamSubsystem(hardwareMap, WebcamSubsystem.PipelineName.CONTOUR_BLUE);
        dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(camera.webcam, 24);
        packet = new TelemetryPacket();

        waitForStart();
        while(opModeIsActive()){
            dashboard.sendTelemetryPacket(packet);
            packet.put("x", camera.getContourProcessor().largestContourCenter().x);
            packet.put("y", camera.getContourProcessor().largestContourCenter().y);
            packet.put("area", camera.getContourProcessor().largestContourArea());
            packet.put("Spike Position", findSpikePosition());
            telemetry.addData("x", camera.getContourProcessor().largestContourCenter().x);
            telemetry.addData("y", camera.getContourProcessor().largestContourCenter().y);
            telemetry.addData("area", camera.getContourProcessor().largestContourArea());
            telemetry.addData("Spike Position", findSpikePosition());
            sleep(20);
        }
    }

    /**
     * Identifies what spike location the game piece is on.
     *
     * @return a String indication the location of the game piece
     */
    public String findSpikePosition() {
        double center = camera.getXProp();

        // TODO: Tune numbers so we can find the position of the spike
        // For reference, camera is 864 pixels wide
        return center < 288 ? "left"
                : center < 576 ? "middle"
                : "right";
    }

}
