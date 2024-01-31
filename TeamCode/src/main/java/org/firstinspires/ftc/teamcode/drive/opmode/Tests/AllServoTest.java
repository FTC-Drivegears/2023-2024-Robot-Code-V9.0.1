package org.firstinspires.ftc.teamcode.drive.opmode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Specifications;

@TeleOp(name="All Servo Test")
public class AllServoTest extends LinearOpMode {
    private enum ServoSide {
        LEFT,
        RIGHT
    }

    private Servo armServo;
    private Servo tiltServo;
    private Servo gate;
    private Servo linkage;

    @Override
    public void runOpMode() throws InterruptedException {
        //tests each servo individually
        ServoSide side = ServoSide.LEFT;

        while(opModeInInit()) {
            if (gamepad1.dpad_left) {
                side = ServoSide.RIGHT;
            } else if (gamepad1.dpad_right) {
                side = ServoSide.LEFT;
            }
            telemetry.addData("side", side);
            telemetry.update();
        }
        waitForStart();
        if(side == ServoSide.LEFT){
            armServo = hardwareMap.get(Servo.class, Specifications.LEFT_OUTPUT_ARM);
            tiltServo = hardwareMap.get(Servo.class, Specifications.LEFT_OUTPUT_TILT);
        }
        else{
            armServo = hardwareMap.get(Servo.class, Specifications.RIGHT_OUTPUT_ARM);
            tiltServo = hardwareMap.get(Servo.class, Specifications.RIGHT_OUTPUT_TILT);
        }
        gate = hardwareMap.get(Servo.class, Specifications.PIXEL_GATE);
        linkage = hardwareMap.get(Servo.class, Specifications.INTAKE_SERVO);

        while(opModeIsActive()) {
            if (gamepad1.a) {
                armServo.setPosition(1);
            } else if (gamepad1.b) {
                armServo.setPosition(0);
            }
            else if (gamepad1.x) {
                tiltServo.setPosition(1);
            } else if (gamepad1.y) {
                tiltServo.setPosition(0);
            }
            else if (gamepad1.right_bumper) {
                gate.setPosition(1);
            } else if (gamepad1.left_bumper) {
                gate.setPosition(0);
            }
            else if (gamepad1.dpad_up) {
                linkage.setPosition(1);
            } else if (gamepad1.dpad_down) {
                linkage.setPosition(0);
            }
        }





    }
}
