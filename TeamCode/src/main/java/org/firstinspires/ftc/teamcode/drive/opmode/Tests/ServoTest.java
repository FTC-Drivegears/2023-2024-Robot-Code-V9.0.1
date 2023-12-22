package org.firstinspires.ftc.teamcode.drive.opmode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="singular servo test")
public class ServoTest extends LinearOpMode {
    com.qualcomm.robotcore.hardware.CRServo servo;

    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(CRServo.class, "intakeServo");
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.a){
                servo.setPower(1);
            }
            else if(gamepad1.b){
                servo.setPower(0);
            }
            telemetry.addData("current", servo.getPower());
        }
    }
}
