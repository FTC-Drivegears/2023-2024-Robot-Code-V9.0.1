package org.firstinspires.ftc.teamcode.drive.opmode;

import android.os.Build;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.command.IntakeCommand;
import org.firstinspires.ftc.teamcode.command.MecanumCommand;
import org.firstinspires.ftc.teamcode.command.MultiMotorCommand;
import org.firstinspires.ftc.teamcode.command.OutputCommand;
import org.firstinspires.ftc.teamcode.subsystems.IMUSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;

import org.firstinspires.ftc.teamcode.subsystems.MultiMotorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WebcamSubsystem;
import org.firstinspires.ftc.teamcode.util.GyroOdometry;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

@Autonomous(name="Autonomous Back Blue Middle")
public class AutonomousBackBlueMiddle extends LinearOpMode {
    private MecanumSubsystem mecanumSubsystem;
    private MecanumCommand mecanumCommand;
    private IMUSubsystem imu;
    private OdometrySubsystem odometrySubsystem;
    private GyroOdometry gyroOdometry;
    private IntakeCommand intakeCommand;
    private WebcamSubsystem webcamSubsystem;
    private OutputCommand outputCommand;
    private MultiMotorSubsystem multiMotorSubsystem;
    private MultiMotorCommand multiMotorCommand;
    FtcDashboard dashboard;
    TelemetryPacket packet;
    private ElapsedTime timer;
    //67, -3, 0
    //54, 24, 0
    //57, -22, -0.832
    //38, 80, -1.58
    private int level = -1;
    private String position = "initalized";

    //TODO: make enum for diff teams

    @Override
    public void runOpMode() throws InterruptedException {
        imu = new IMUSubsystem(hardwareMap);
        mecanumSubsystem = new MecanumSubsystem(hardwareMap);
        odometrySubsystem = new OdometrySubsystem(hardwareMap);
        gyroOdometry = new GyroOdometry(odometrySubsystem, imu);
        mecanumCommand = new MecanumCommand(mecanumSubsystem, odometrySubsystem, gyroOdometry, this);
        //Note different for autonomous front red --> kpy
        mecanumCommand.setConstants(0.07, 0.01, 0.0075/2, 0.06, 0.005, 0.0075/2, 2, 0.05, 0.0);
        intakeCommand = new IntakeCommand(hardwareMap);
        outputCommand = new OutputCommand(hardwareMap);
        multiMotorSubsystem = new MultiMotorSubsystem(hardwareMap, true, MultiMotorSubsystem.MultiMotorType.dualMotor);
        multiMotorCommand = new MultiMotorCommand(multiMotorSubsystem);
        webcamSubsystem = new WebcamSubsystem(hardwareMap, WebcamSubsystem.PipelineName.CONTOUR_BLUE);
        timer = new ElapsedTime();

        odometrySubsystem.reset();
        imu.resetAngle();

        intakeCommand.raiseIntake();
        outputCommand.closeGate();

        outputCommand.armToIdle();
        outputCommand.tiltToIdle();
        double propPosition = 0;
        while(opModeInInit()){
            propPosition = webcamSubsystem.getXProp();
            telemetry.addData("proppos", propPosition);
            telemetry.update();
        }
        waitForStart();

        Executor executor = Executors.newFixedThreadPool(4);
        CompletableFuture.runAsync(this::updateOdometry, executor);
        CompletableFuture.runAsync(this::updateTelemetry, executor);
        CompletableFuture.runAsync(this::liftProcess, executor);
        timer.reset();
        intakeCommand.raiseIntake();
        mecanumCommand.moveToGlobalPosition(57, 0, 0);
        sleep(1500);

        timer.reset();
        while(timer.milliseconds() < 1500) {

            //TODO: tune
            if (propPosition > 75) {
                //pos RIGHT
                position = "right";
                mecanumCommand.moveToGlobalPosition(47, -28, 0);
                sleep(1500);
            } else if (propPosition <= 75 && propPosition > 0) {
                //pos middle
                position = "middle";
                mecanumCommand.moveToGlobalPosition(66, 3, 0);
                sleep(1000);
            } else {
                //pos left
                position = "left";
                mecanumCommand.moveToGlobalPositionAccurate(57, 18.5, 0.87);
                sleep(1000);
            }
        }
        timer.reset();
        while(timer.milliseconds() < 1000) {
            intakeCommand.intakeOut(0.3);
        }
        intakeCommand.stopIntake();
        mecanumCommand.moveToGlobalPosition(35, 0, 0);
        if(propPosition <= 100 && propPosition > 0) {
            mecanumCommand.moveToGlobalPosition(67, -40, 0);
            sleep(1000);
            mecanumCommand.moveToGlobalPosition(122, -40, 0);
            sleep(1000);
        }
        else if(propPosition > 100) {
            mecanumCommand.moveToGlobalPosition(40, 5, 0);
            sleep(1000);
            mecanumCommand.moveToGlobalPosition(130, 5, 0);
        }
        else{
            mecanumCommand.moveToGlobalPosition(122, -10, 0);
            sleep(3000);
        }
        mecanumCommand.moveToGlobalPosition(150, 140, 0);
        sleep(3000);
//        timer.reset();

        level = 1;
        outputCommand.armToBoard();
        outputCommand.tiltToBoard();
        sleep(2000);
        timer.reset();
        while(timer.milliseconds() < 3500) {
            //TODO: tune
            if (propPosition > 75 && opModeIsActive()) {
                //pos right
                mecanumCommand.moveToGlobalPositionAccurate(67, 219, -1.6);

            } else if (propPosition <= 75 && propPosition > 0 && opModeIsActive()) {
                //pos middle
                mecanumCommand.moveToGlobalPositionAccurate(59, 219, -1.6);
            } else {
                //pos left
                if(opModeIsActive()) {
                    mecanumCommand.moveToGlobalPositionAccurate(48, 217, -1.6);
                }
            }
        }
        //136, 0, `1.6
        //126, 140, 1.6
        //52, 220, 1.5
        //67, 222, 1.5
        //81, 222, 1.5
        //0, 222, 1.5
// 68, 6.3, -0.3

        timer.reset();
        while (timer.milliseconds() < 500){
            outputCommand.openGate();
        }
        outputCommand.closeGate();
        outputCommand.tiltToIdle();
        outputCommand.armToIdle();
        sleep(5000);
        level = 0;
        sleep(1000);
//        mecanumCommand.moveToGlobalPosition(0, 222, -1.6);
    }

    public void updateOdometry() {
        while (opModeIsActive()) {
            gyroOdometry.odometryProcess();
        }
    }

    public void updateTelemetry() {
        while (opModeIsActive()) {
//            packet.put("x", gyroOdometry.x);
//            packet.put("y", gyroOdometry.y);
            telemetry.addData("x", gyroOdometry.x);
            telemetry.addData("y", gyroOdometry.y);
            telemetry.addData("theta", gyroOdometry.theta);
            telemetry.addData("position", position);
//            packet.put("x", gyroOdometry.x);
//            packet.put("y", gyroOdometry.y);
//            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }
    }
    public void liftProcess() {
        while(opModeIsActive()) {
            multiMotorCommand.LiftUp(true, level);
        }
    }
}