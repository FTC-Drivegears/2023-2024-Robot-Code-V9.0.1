package org.firstinspires.ftc.teamcode.drive.opmode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Intake Test")
public class IntakeTest extends LinearOpMode {
    FtcDashboard dash;
    TelemetryPacket packet;
    @Override
    public void runOpMode() throws InterruptedException {
        dash = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
        DcMotor intake = hardwareMap.get(DcMotor.class, "intakeMotor");

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        while(opModeIsActive()) {
            intake.setPower(gamepad1.left_stick_y);
            dash.sendTelemetryPacket(packet);

//            packet.put("leftFront", leftFront.getCurrentPosition());
//            packet.put("rightFront", rightFront.getCurrentPosition());
//            packet.put("leftBack", leftBack.getCurrentPosition());
//            packet.put("rightBack", rightBack.getCurrentPosition());
//            //program for testing each dc motor individually
//            telemetry.addData("leftFront", leftFront.getCurrentPosition());
//            telemetry.addData("rightFront", rightFront.getCurrentPosition());
//            telemetry.addData("leftBack", leftBack.getCurrentPosition());
//            telemetry.addData("rightBack", rightBack.getCurrentPosition());
            telemetry.update();
        }


    }




}
