package org.firstinspires.ftc.teamcode.drive.opmode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Pipeline Switch Test")
public class PipelineSwitchTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while(opModeIsActive()) {
            if (gamepad1.a) {
                telemetry.addData("Pipeline", "Stone");
            } else if (gamepad1.b) {
                telemetry.addData("Pipeline", "Skystone");
            }
            telemetry.update();
        }
    }
}
