package org.firstinspires.ftc.teamcode.drive.opmode;

import android.os.Build;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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

@Autonomous(name="Autonomous Front Blue")
public class AutonomousFrontBlue extends LinearOpMode {
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


    @Override
    public void runOpMode() throws InterruptedException {
        //contour location before 60
        imu = new IMUSubsystem(hardwareMap);
        mecanumSubsystem = new MecanumSubsystem(hardwareMap);
        odometrySubsystem = new OdometrySubsystem(hardwareMap);
        gyroOdometry = new GyroOdometry(odometrySubsystem, imu);
        mecanumCommand = new MecanumCommand(mecanumSubsystem, odometrySubsystem, gyroOdometry, this);
        intakeCommand = new IntakeCommand(hardwareMap);
        outputCommand = new OutputCommand(hardwareMap);
        multiMotorSubsystem = new MultiMotorSubsystem(hardwareMap, true, MultiMotorSubsystem.MultiMotorType.dualMotor);
        multiMotorCommand = new MultiMotorCommand(multiMotorSubsystem);
        webcamSubsystem = new WebcamSubsystem(hardwareMap, WebcamSubsystem.PipelineName.CONTOUR_BLUE);
        timer = new ElapsedTime();
        LinearOpMode opMode = this;

        odometrySubsystem.reset();
//        imu.resetAngle();
//
//        intakeCommand.raiseIntake();
//        outputCommand.closeGate();
//
//        outputCommand.armToIdle();
//        outputCommand.tiltToIdle();
        String position = "middle";
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
        double propPosition = 0;
        while(opModeInInit()){
            propPosition = webcamSubsystem.getXProp();
        }

        waitForStart();

        Executor executor = Executors.newFixedThreadPool(4);
        CompletableFuture.runAsync(this::updateOdometry, executor);
        CompletableFuture.runAsync(this::updateTelemetry, executor);
        CompletableFuture.runAsync(this::pidProcess, executor);
        CompletableFuture.runAsync(this::motorProcess, executor);
//        CompletableFuture.runAsync(this::liftProcess, executor);




        sleep(2000);
        if(position.equals("left")) {
            mecanumCommand.setFinalPosition(true, 20, 103, 15, 0);
            while(!mecanumCommand.isPositionReached(true,false)) {}
        }
        else if(position.equals("middle")){
            mecanumCommand.setFinalPosition(true, 20, 122, -8, 0);
            while(!mecanumCommand.isPositionReached(false,true)) {}
        }
        else if(position.equals("right")){
            mecanumCommand.setFinalPosition(true, 20, 70, -7, 1.5);
            while(!mecanumCommand.isPositionReached(true,false)) {
            }
        }

        intakeCommand.lowerIntake();

        timer.reset();

        while(timer.milliseconds() < 5000) {
            intakeCommand.intakeOut(0.5);
        }
        intakeCommand.stopIntake();

        sleep(2000);
        /*
        if(position.equals("right")){
            while(mecanumCommand.isPositionPassed()) {
                mecanumCommand.setFinalPosition(true, 30, 79.74, 6.13, 0.84);
            }
        }
        */

//        timer.reset();
//        while(timer.milliseconds() < 1500) {
//            if (propPosition < 100 && propPosition > 0) {
//                //pos 2
//                mecanumCommand.moveToGlobalPositionAccurate(67, -3, 0);
//            } else if (propPosition > 100) {
//                mecanumCommand.moveToGlobalPositionAccurate(55, -17, -0.832);
//                sleep(1000);
//            } else {
//                mecanumCommand.moveToGlobalPositionAccurate(54, 20, 0);
//            }
//        }
        /*
        timer.reset();

        while(timer.milliseconds() < 1000) {
            intakeCommand.intakeOut(0.3);
        }
        intakeCommand.stopIntake();
        */


        level = 1;
        outputCommand.armToBoard();
        outputCommand.tiltToBoard();
        timer.reset();

//        while(timer.milliseconds() < 3500) {
//            if (propPosition < 100 && propPosition > 0 && opModeIsActive()) {
//                //pos 2
//                mecanumCommand.moveToGlobalPosition(53, 83.5, -1.58);
//            } else if (propPosition >= 100 && opModeIsActive()){
//                mecanumCommand.moveToGlobalPosition(68, 83.5, -1.58);
//            } else {
//                mecanumCommand.moveToGlobalPosition(34, 83.5, -1.58);
//            }
//        }

        if(position.equals("middle")) {
            mecanumCommand.setFinalPosition(true, 30, 146, -77, 0);
            while(!mecanumCommand.isCoordinatePassed()) {}
            mecanumCommand.setFinalPosition(true, 30, 59, -76, 0);
            while(!mecanumCommand.isCoordinatePassed()) {}
            mecanumCommand.setFinalPosition(true, 30, 53, 83.5, -1.58);
            while(!mecanumCommand.isPositionReached(false,false)) {}
        }
        else if(position.equals("right")){
            mecanumCommand.setFinalPosition(true, 30, 68, 83.5, -1.58);
            while(!mecanumCommand.isCoordinatePassed()) {}
            mecanumCommand.setFinalPosition(true, 30, 68, 83.5, -1.58);
            while(!mecanumCommand.isCoordinatePassed()) {}
            mecanumCommand.setFinalPosition(true, 30, 68, 83.5, -1.58);
            while(!mecanumCommand.isPositionReached(false,false)) {}
        }
        else if(position.equals("left")){
            mecanumCommand.setFinalPosition(true, 30, 34, 81.69, 1.53);
            while(!mecanumCommand.isCoordinatePassed()) {}
            mecanumCommand.setFinalPosition(true, 30, 34, 81.69, 1.53);
            while(!mecanumCommand.isCoordinatePassed()) {}
            mecanumCommand.setFinalPosition(true, 30, 34, 81.69, 1.53);
            while(!mecanumCommand.isPositionReached(false,false)) {}
        }

        timer.reset();
        while (timer.milliseconds() < 500){
            outputCommand.openGate();
        }
        outputCommand.closeGate();
        outputCommand.tiltToIdle();
        outputCommand.armToIdle();
        sleep(3000);
        level = 0;
//        mecanumCommand.moveToGlobalPosition(30, 65, -1.58);
//        sleep(1000);
//        mecanumCommand.moveToGlobalPosition(0, 84, -1.58);


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
            packet.put("theta", gyroOdometry.theta);
            packet.put("position reached", mecanumCommand.isPositionReached(false,false));
            telemetry.addData("x", gyroOdometry.x);
            telemetry.addData("y", gyroOdometry.y);
            telemetry.addData("theta", gyroOdometry.theta);
//            packet.put("x", gyroOdometry.x);
//            packet.put("y", gyroOdometry.y);
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
