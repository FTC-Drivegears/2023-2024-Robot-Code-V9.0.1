package org.firstinspires.ftc.teamcode.drive.opmode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.command.MultiMotorCommand;
import org.firstinspires.ftc.teamcode.subsystems.MultiMotorSubsystem;

import java.util.concurrent.CompletableFuture;

@Autonomous(name = "liftauto test")
public class DualMotorAutoTest extends LinearOpMode {
    private MultiMotorSubsystem multiMotorSubsystem;
    private MultiMotorCommand multiMotorCommand;
    private int level = 0;
    private ElapsedTime elapsedTime;
    private FtcDashboard dashboard;
    private TelemetryPacket packet;
    private boolean running = true;
    @Override
    public void runOpMode() throws InterruptedException{
        ElapsedTime elapsedTime = new ElapsedTime();
        multiMotorSubsystem = new MultiMotorSubsystem(hardwareMap, true, MultiMotorSubsystem.MultiMotorType.dualMotor);
        multiMotorCommand = new MultiMotorCommand(multiMotorSubsystem);
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
        waitForStart();
        CompletableFuture.runAsync(this::liftProcess);
        CompletableFuture.runAsync(this::runTelemetry);

        while(opModeIsActive()) {
            elapsedTime.reset();
            while (elapsedTime.milliseconds() < 5000) {
                level = 1;
            }
            elapsedTime.reset();
            while (elapsedTime.milliseconds() < 5000) {
                level = 2;
            }
            elapsedTime.reset();
//            level = 0;
        }
    }



    public void liftProcess() {
        while(opModeIsActive() && running){
            multiMotorCommand.LiftUpPositional(true, level);
            if(level == 0 && running && (multiMotorSubsystem.getDerivativeValue() == 0 && multiMotorSubsystem.getPosition() < 5) || (multiMotorSubsystem.getDerivativeValue() < 0 && multiMotorSubsystem.getPosition() < -5)){
                multiMotorSubsystem.reset();
//                running = false;
            }
        }
    }

    public void runTelemetry(){
        while(opModeIsActive()){
            dashboard.sendTelemetryPacket(packet);
            telemetry.addData("level", level);
            telemetry.addData("targetPos", multiMotorCommand.getTargetPos());
            telemetry.addData("position", multiMotorSubsystem.getPosition());
            packet.put("level", level);
            packet.put("targetPos", multiMotorCommand.getTargetPos());
            packet.put("position", multiMotorSubsystem.getPosition());
            packet.put("integralSum", multiMotorSubsystem.getPidUp().getIntegralSum());
            packet.put("error", multiMotorSubsystem.getPidUp().getError());
            packet.put("integralsign", Math.signum(multiMotorSubsystem.getPidUp().getIntegralSum()));
            packet.put("errorsign", Math.signum(multiMotorSubsystem.getPidUp().getError()));
            packet.put("output", multiMotorSubsystem.getPidUp().getOutputPositionalValue());
            telemetry.update();
        }
    }


}
