package org.firstinspires.ftc.teamcode.drive.opmode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.command.OutputCommand;
import org.firstinspires.ftc.teamcode.util.Specifications;


@TeleOp(name="singular servo test")
public class ServoTest extends LinearOpMode {
    OutputCommand outputCommand;
    com.qualcomm.robotcore.hardware.Servo servo;
    CRServo roller;

    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, "leftTilt");
        roller = hardwareMap.get(CRServo.class, Specifications.INTAKE_ROLLER);
//        outputCommand = new OutputCommand(hardwareMap);
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.a){
                servo.setPosition(0.295);
            }
            else if(gamepad1.b){
                servo.setPosition(0.46);
            }
            roller.setPower(gamepad1.right_trigger);
//            telemetry.addData("pos",servo.getPosition());
            telemetry.update();
        }
    }
}
