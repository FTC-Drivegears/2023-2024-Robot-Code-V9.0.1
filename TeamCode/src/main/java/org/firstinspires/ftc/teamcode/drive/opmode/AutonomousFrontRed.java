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

@Autonomous(name="Autonomous Front Red")
public class AutonomousFrontRed extends LinearOpMode {
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


    @Override
    public void runOpMode() throws InterruptedException {
        imu = new IMUSubsystem(hardwareMap);
        mecanumSubsystem = new MecanumSubsystem(hardwareMap);
        odometrySubsystem = new OdometrySubsystem(hardwareMap);
        gyroOdometry = new GyroOdometry(odometrySubsystem, imu);
        mecanumCommand = new MecanumCommand(mecanumSubsystem, odometrySubsystem, gyroOdometry, this);
        //Note different for autonomous front red --> kpy
        mecanumCommand.setConstants(0.07, 0.01, 0.0075/2, 0.05, 0.005, 0.0075/2, 2, 0.05, 0.0);
        intakeCommand = new IntakeCommand(hardwareMap);
        outputCommand = new OutputCommand(hardwareMap);
        multiMotorSubsystem = new MultiMotorSubsystem(hardwareMap, true, MultiMotorSubsystem.MultiMotorType.dualMotor);
        multiMotorCommand = new MultiMotorCommand(multiMotorSubsystem);
        webcamSubsystem = new WebcamSubsystem(hardwareMap, WebcamSubsystem.PipelineName.CONTOUR_RED);
        timer = new ElapsedTime();

        odometrySubsystem.reset();
        imu.resetAngle();

        intakeCommand.raiseIntake();
        outputCommand.closeGate();

        outputCommand.armToIdle();
        outputCommand.tiltToIdle();

        String position = "middle";
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();

        double propPosition = 0;
//        while(opModeInInit()){
//            propPosition = webcamSubsystem.getXProp();
//        }

        waitForStart();

        Executor executor = Executors.newFixedThreadPool(5);
        CompletableFuture.runAsync(this::updateOdometry, executor);
        CompletableFuture.runAsync(this::updateTelemetry, executor);
        CompletableFuture.runAsync(this::motorProcess, executor);
        CompletableFuture.runAsync(this::pidProcess, executor);
//        CompletableFuture.runAsync(this::liftProcess, executor);


        /*
        intakeCommand.raiseIntake();
//        sleep(8000);
        mecanumCommand.setFinalPosition(true, 30, 57, 0, 0);
        sleep(1500);
        timer.reset();
        while(timer.milliseconds() < 1500) {

            //TODO: tune
            //left
            mecanumCommand.setFinalPosition(true, 30, 60, 17.5, 0.832);
            //right
            mecanumCommand.setFinalPosition(true, 30,47, -32, 0);
            //middle
            mecanumCommand.setFinalPosition(true, 30, 66, 3, 0);
//            if (propPosition > 100) {
//                //pos RIGHT
//                position = "right";
//                mecanumCommand.setFinalPosition(true, 30,47, -32, 0);
//            } else if (propPosition <= 100 && propPosition > 0) {
//                //pos middle
//                position = "middle";
//                mecanumCommand.setFinalPosition(true, 30, 66, 3, 0);
//                sleep(1000);
//            } else {
//                //pos left
//                position = "left";
//                mecanumCommand.setFinalPosition(true, 30, 60, 17.5, 0.832);
//            }
        }

         */

        sleep(2000);
        if(position.equals("left")) {
            mecanumCommand.setFinalPosition(true, 20, 73, 6.5, -1.5);
            while(!mecanumCommand.isPositionReached(true,false)) {}
        }
        else if(position.equals("middle")){
            mecanumCommand.setFinalPosition(true, 20, 122, 8, 0);
            while(!mecanumCommand.isPositionReached(false,true)) {}
        }
        else if(position.equals("right")) {
            mecanumCommand.setFinalPosition(true, 20, 103, -18, 0);
            while (!mecanumCommand.isPositionReached(true, false)) {
            }
        }

        intakeCommand.lowerIntake();

        timer.reset();

        while(timer.milliseconds() < 5000) {
            intakeCommand.intakeOut(0.5);
        }
        intakeCommand.stopIntake();


        sleep(2000);

        if(position.equals("middle")) {
            mecanumCommand.setFinalPosition(true, 5, 126, 0, 0);
            while(!mecanumCommand.isCoordinatePassed()) {}
            mecanumCommand.setFinalPosition(true, 30, 126, -64, 0);
            while(!mecanumCommand.isCoordinatePassed()) {}
            mecanumCommand.setFinalPosition(true, 30, 60, -92, -1.58);
            while(!mecanumCommand.isPositionReached(true,true)) {}
            //mecanumCommand.setFinalPosition(true, 30, 64, -77, -1.58);
            //while(!mecanumCommand.isPositionReached(false,false)) {}
        }
        else if(position.equals("right")){
            mecanumCommand.setFinalPosition(true, 20, 68, 83.5, -1.58);
            while(!mecanumCommand.isCoordinatePassed()) {}
            mecanumCommand.setFinalPosition(true, 20, 59, -76, 0);
            while(!mecanumCommand.isCoordinatePassed()) {}
            mecanumCommand.setFinalPosition(true, 20, 68, 83.5, -1.58);
            while(!mecanumCommand.isPositionReached(false,false)) {}
        }
        else if(position.equals("left")){
            mecanumCommand.setFinalPosition(true, 20, 34, 81.69, 1.53);
            while(!mecanumCommand.isPositionReached(false,false)) {}
        }

        level = 5;
        outputCommand.armToBoard();
        outputCommand.tiltToBoard();
        level = 1;

        /*
        timer.reset();
        while(timer.milliseconds() < 3500) {
            //TODO: tune
            if (propPosition > 100) {
                //pos right
                mecanumCommand.setFinalPosition(true, 30, 36, -78.5, 1.65);

            } else if (propPosition <= 100 && propPosition > 0) {
                //pos middle
                mecanumCommand.setFinalPosition(true, 30, 52, -80, 1.65);
            } else {
                //pos left
                mecanumCommand.setFinalPosition(true, 30, 68, -81.5, 1.65);
            }
        }
         */
        /*
        timer.reset();
//        while (timer.milliseconds() < 500){
//            outputCommand.openGate();
//        }
//        outputCommand.closeGate();
//        outputCommand.tiltToIdle();
//        outputCommand.armToIdle();
        sleep(6000);
        level = 0;

        */

    }
    public void pidProcess(){
        while (opModeIsActive()) {
            mecanumCommand.pidProcess();
        }
    }

    public void motorProcess(){
        while (opModeIsActive()) {
            mecanumSubsystem.motorProcess();
        }
    }

    public void updateOdometry() {
        while (opModeIsActive()) {
            imu.gyroProcess();
            gyroOdometry.process();
        }
    }

    public void updateTelemetry() {
        while (opModeIsActive()) {
            packet.put("x", gyroOdometry.x);
            packet.put("y", gyroOdometry.y);
            telemetry.addData("x", gyroOdometry.x);
            telemetry.addData("y", gyroOdometry.y);
            telemetry.addData("theta", gyroOdometry.theta);
            telemetry.addData("position", position);
            packet.put("x", gyroOdometry.x);
            packet.put("y", gyroOdometry.y);
            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }
    }
    public void liftProcess() {
        while(opModeIsActive()) {
            multiMotorCommand.LiftUp(true, level);
        }
    }
}