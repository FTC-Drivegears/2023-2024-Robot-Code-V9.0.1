package org.firstinspires.ftc.teamcode.drive.opmode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Specifications;

@TeleOp(name="All Servo Test")
public class AllServoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //tests each servo individually
        Servo leftArm = hardwareMap.get(Servo.class, Specifications.LEFT_OUTPUT_ARM);
        Servo rightArm = hardwareMap.get(Servo.class, Specifications.RIGHT_OUTPUT_ARM);
        Servo leftTilt = hardwareMap.get(Servo.class, Specifications.LEFT_OUTPUT_TILT);
        Servo rightTilt = hardwareMap.get(Servo.class, Specifications.RIGHT_OUTPUT_TILT);
        Servo gate = hardwareMap.get(Servo.class, Specifications.PIXEL_GATE);

        waitForStart();





    }
}
