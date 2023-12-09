package org.firstinspires.ftc.teamcode.drive.opmode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="singular servo test")
public class ServoTest extends LinearOpMode {
    Servo servo = hardwareMap.get(Servo.class, "leftArm");
    double position = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.a){
                servo.setPosition(1);
            }
            else if(gamepad1.b){
                servo.setPosition(0);
            }
        }
    }
}
