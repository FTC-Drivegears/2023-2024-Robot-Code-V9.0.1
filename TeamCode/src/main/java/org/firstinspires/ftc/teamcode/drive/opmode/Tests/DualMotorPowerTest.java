package org.firstinspires.ftc.teamcode.drive.opmode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.command.MecanumCommand;
import org.firstinspires.ftc.teamcode.command.MultiMotorCommand;
import org.firstinspires.ftc.teamcode.command.OutputCommand;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MultiMotorSubsystem;

import java.util.concurrent.CompletableFuture;

@Config
@TeleOp
public class DualMotorPowerTest extends LinearOpMode {
    private MultiMotorSubsystem multiMotorSubsystem;
    private MultiMotorCommand multiMotorCommand;
    private MecanumSubsystem mecanumSubsystem;
    private int level = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize your hardware components
        FtcDashboard dash = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        ElapsedTime timer = new ElapsedTime();
        double targetPosition = 0;

        multiMotorSubsystem = new MultiMotorSubsystem(hardwareMap, true, MultiMotorSubsystem.MultiMotorType.dualMotor);
        multiMotorCommand = new MultiMotorCommand(multiMotorSubsystem);
        mecanumSubsystem = new MecanumSubsystem(hardwareMap);
        OutputCommand outputCommand = new OutputCommand(hardwareMap);

        waitForStart();

        CompletableFuture.runAsync(this::liftProcess);

        while (opModeIsActive()) {
            outputCommand.tiltToIdle();
            outputCommand.armToIdle();

            if(gamepad1.left_bumper){
                multiMotorSubsystem.reset();
            }

            if(gamepad1.a){
                multiMotorSubsystem.getPidUp().integralReset();
                level = 4;
                targetPosition = 3100;
            }
            else if(gamepad1.b){
                multiMotorSubsystem.getPidUp().integralReset();
                level = 3;
                targetPosition = 0;
            }
            else if(gamepad1.y){
                multiMotorSubsystem.getPidUp().integralReset();
                level = 2;
                targetPosition = 3100;
            }
            else if(gamepad1.x){
                multiMotorSubsystem.getPidUp().integralReset();
                level = 1;
                targetPosition = 1300;
            }
            else if(gamepad1.dpad_down){
                multiMotorSubsystem.getPidUp().integralReset();
                level = 0;
                targetPosition = 0;
            }
            else if(gamepad1.right_bumper){
                multiMotorSubsystem.getPidUp().integralReset();
                level = 5;
                targetPosition = 900;
            }
//            else {
//                multiMotorSubsystem.moveLift(gamepad1.left_stick_y);
//            }
//            mecanumSubsystem.fieldOrientedMove(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, 0);

            packet.put("position", multiMotorSubsystem.getPosition());
            packet.put("power", multiMotorSubsystem.getMainPower());
            packet.put("auxpower", multiMotorSubsystem.getAux1Power());
            packet.put("derivativeValue", multiMotorSubsystem.getDerivativeValue());
            packet.put("errorValue", multiMotorSubsystem.getErrorValue());
            packet.put("intervalValue", multiMotorSubsystem.getIntervalValue());
            packet.put("lastErrorValue", multiMotorSubsystem.getLastErrorValue());
            packet.put("controlleroutput", multiMotorSubsystem.getCascadeOutput());
            packet.put("outputPositionalValue", multiMotorSubsystem.getPidUp().getOutputPositionalValue());
            packet.put("outputVelocityValue", multiMotorSubsystem.getCascadeVelocity());
            packet.put("level", level);
            packet.put("Target Position", targetPosition);
            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("position", multiMotorSubsystem.getPosition());
            telemetry.addData("power", multiMotorSubsystem.getMainPower());
            telemetry.addData("auxpower", multiMotorSubsystem.getAux1Power());
            telemetry.addData("auxpos", multiMotorSubsystem.getAuxPos());
            telemetry.addData("derivativeValue", multiMotorSubsystem.getDerivativeValue());
            telemetry.addData("controlleroutput", multiMotorSubsystem.getCascadeOutput());
            telemetry.addData("level", level);
            telemetry.addData("positionalValue", multiMotorSubsystem.getPidUp().getOutputPositionalValue());
            telemetry.addData("integralValue", multiMotorSubsystem.getPidUp().getIntegralSum()*0.000119);
            telemetry.update();
            dash.sendTelemetryPacket(packet);
        }
    }

    public void liftProcess(){
        while(opModeIsActive()){
            multiMotorCommand.LiftUpPositional(true, level);
            if(level == 0) {
                if ((multiMotorSubsystem.getDerivativeValue() == 0 && multiMotorSubsystem.getPosition() < 5) || (multiMotorSubsystem.getDerivativeValue() < 0 && multiMotorSubsystem.getPosition() < -5)) {
                    multiMotorSubsystem.reset();
                }
            }
        }
    }
}
