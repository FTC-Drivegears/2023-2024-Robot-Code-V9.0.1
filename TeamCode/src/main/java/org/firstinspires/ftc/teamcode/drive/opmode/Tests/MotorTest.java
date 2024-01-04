package org.firstinspires.ftc.teamcode.drive.opmode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Motor Test")
public class MotorTest extends LinearOpMode {
    FtcDashboard dash;
    TelemetryPacket packet;
    @Override
    public void runOpMode() throws InterruptedException {
        dash = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        while(opModeIsActive()) {
            dash.sendTelemetryPacket(packet);
            packet.put("leftFront", leftFront.getCurrentPosition());
            packet.put("rightFront", rightFront.getCurrentPosition());
            packet.put("leftBack", leftBack.getCurrentPosition());
            packet.put("rightBack", rightBack.getCurrentPosition());
            //program for testing each dc motor individually
            telemetry.addData("leftFront", leftFront.getCurrentPosition());
            telemetry.addData("rightFront", rightFront.getCurrentPosition());
            telemetry.addData("leftBack", leftBack.getCurrentPosition());
            telemetry.addData("rightBack", rightBack.getCurrentPosition());
            telemetry.update();
        }


    }




}
