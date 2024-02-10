package org.firstinspires.ftc.teamcode.drive.opmode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.command.OutputCommand;
import org.firstinspires.ftc.teamcode.util.Specifications;


@Config
@TeleOp(name="singular servo test")
public class ServoTest extends LinearOpMode {
    OutputCommand outputCommand;
    com.qualcomm.robotcore.hardware.Servo servo;
    Servo roller;
    public static double pos1 = 0.46;
    public static double pos2 = 0.2;
    private FtcDashboard dash;
    private TelemetryPacket packet;

    @Override
    public void runOpMode() throws InterruptedException {
//        servo = hardwareMap.get(Servo.class, "pixelGate");
        roller = hardwareMap.get(Servo.class, "droneShooter");
        dash = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
        outputCommand = new OutputCommand(hardwareMap);
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.a){
//                outputCommand.openGate();
                roller.setPosition(pos1);
            }
            else if(gamepad1.b){
                outputCommand.closeGate();
                roller.setPosition(pos2);
            }
//            roller.setPower(gamepad1.right_trigger);
//            telemetry.addData("pos",servo.getPosition());
            telemetry.update();
            packet.put("servoPos", servo.getPosition());
            dash.sendTelemetryPacket(packet);
        }
    }
}
