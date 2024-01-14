package org.firstinspires.ftc.teamcode.drive.opmode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.command.OutputCommand;


@TeleOp(name="singular servo test")
public class ServoTest extends LinearOpMode {
    OutputCommand outputCommand;
    com.qualcomm.robotcore.hardware.Servo servo;

    @Override
    public void runOpMode() throws InterruptedException {
//        servo = hardwareMap.get(Servo.class, "leftTilt");
        outputCommand = new OutputCommand(hardwareMap);
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.a){
                outputCommand.tiltToBoard();
            }
            else if(gamepad1.b){
                outputCommand.tiltToIdle();
            }
//            telemetry.addData("pos",servo.getPosition());
            telemetry.update();
        }
    }
}
