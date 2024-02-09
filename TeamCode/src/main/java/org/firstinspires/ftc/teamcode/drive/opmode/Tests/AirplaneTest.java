package org.firstinspires.ftc.teamcode.drive.opmode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Airplane Test")
public class AirplaneTest extends LinearOpMode {
    private Servo shooterServo;
    @Override
    public void runOpMode(){
        shooterServo = hardwareMap.get(Servo.class, "droneShooter");
        shooterServo.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.a)
                shooterServo.setPosition(.75);
            else if(gamepad1.b)
                shooterServo.setPosition(1.25);
        }
    }
}
