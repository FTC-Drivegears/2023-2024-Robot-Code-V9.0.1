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
import org.firstinspires.ftc.teamcode.subsystems.AprilCamSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IMUSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;

import org.firstinspires.ftc.teamcode.subsystems.MultiMotorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.WebcamSubsystem;
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
   //private WebcamSubsystem webcamSubsystem;
    private OutputCommand outputCommand;
    private MultiMotorSubsystem multiMotorSubsystem;
    private MultiMotorCommand multiMotorCommand;
    private AprilCamSubsystem aprilCamSubsystem;
    FtcDashboard dashboard;
    TelemetryPacket packet;
    private ElapsedTime timer;
    //67, -3, 0
    //54, 24, 0
    //57, -22, -0.832
    //38, 80, -1.58
    private int level = -1;
    private String position = "initalized";
    private double targetX = 0;
    private double targetY = 0;

    private Integer aprilID = 2;

    private boolean goToAprilTag = false;

    private String autoColor = "blue"; // or "red"

    private String parkPlace = "left";
    private boolean running = false;
    private boolean raising = false;


    @Override
    public void runOpMode() throws InterruptedException {
        imu = new IMUSubsystem(hardwareMap);
        mecanumSubsystem = new MecanumSubsystem(hardwareMap);
        odometrySubsystem = new OdometrySubsystem(hardwareMap);
        gyroOdometry = new GyroOdometry(odometrySubsystem, imu);
        mecanumCommand = new MecanumCommand(mecanumSubsystem, odometrySubsystem, gyroOdometry, this);
        intakeCommand = new IntakeCommand(hardwareMap);
        outputCommand = new OutputCommand(hardwareMap);
        multiMotorSubsystem = new MultiMotorSubsystem(hardwareMap, true, MultiMotorSubsystem.MultiMotorType.dualMotor);
        multiMotorCommand = new MultiMotorCommand(multiMotorSubsystem);
        //webcamSubsystem = new WebcamSubsystem(hardwareMap, WebcamSubsystem.PipelineName.CONTOUR_BLUE);
        aprilCamSubsystem = new AprilCamSubsystem(hardwareMap);
        timer = new ElapsedTime();
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();

        imu.resetAngle();
        odometrySubsystem.reset();

//        intakeCommand.raiseIntake();
//        outputCommand.closeGate();
//
//        outputCommand.armToIdle();
//        outputCommand.tiltToIdle();
        double propPosition = 0;
        while(opModeInInit()){
            //TODO: determine which Xprop positions make left, middle, right
//            propPosition = webcamSubsystem.getXProp();
        }
        waitForStart();

        Executor executor = Executors.newFixedThreadPool(7);
        CompletableFuture.runAsync(this::updateOdometry, executor);
        CompletableFuture.runAsync(this::updateTelemetry, executor);
        CompletableFuture.runAsync(this::pidProcess, executor);
        CompletableFuture.runAsync(this::motorProcess, executor);
        CompletableFuture.runAsync(this::liftProcess, executor);
        //CompletableFuture.runAsync(this::tagDetectionProcess);


        sleep(1000);
        timer.reset();
        intakeCommand.raiseIntake();
        String position = "left";
        timer.reset();


        //PIXEL DROPOFF POSITION
        mecanumCommand.setFinalPosition(true, 30, 71.5, 0, 0);
        while(!mecanumCommand.isPositionPassed() && !isStopRequested()){}
        if(position.equals("left")) {
            mecanumCommand.setFinalPosition(true, 30, 107.76, 19.99, 0);
            while(!mecanumCommand.isPositionReached(true,true) && !isStopRequested()) {
            }
        }
        else if(position.equals("middle")){
            mecanumCommand.setFinalPosition(true, 30, 120.36, -12.48, 0);
            while(!mecanumCommand.isPositionReached(true,true)) {
            }
        }
        else if(position.equals("right")){
            mecanumCommand.setFinalPosition(true, 30,  69.48, -22, 2.11);
            while(!mecanumCommand.isPositionReached(true,true)) {
            }
        }
        sleep(1000);
        timer.reset();
        //release pixel

        intakeCommand.lowerIntake();
        while(timer.milliseconds() < 3000) {
            intakeCommand.intakeOut(0.5);
        }
        intakeCommand.stopIntake();
        intakeCommand.raiseIntake();
        timer.reset();

        //
//        level = 1;
//        outputCommand.armToBoard();
//        outputCommand.tiltToBoard();

        //move to board
        mecanumCommand.setFinalPosition(true, 30, 133, 31, Math.PI / 2);
        while(!mecanumCommand.isPositionPassed()) {
        }
        mecanumCommand.setFinalPosition(true, 30, 80.5, 49, Math.PI / 2);
        while(!mecanumCommand.isPositionPassed()){
        }
        /*
        goToAprilTag = true;
        sleep(1000);

        while(goToAprilTag && !isStopRequested()) {
            if(aprilCamSubsystem.getHashmap().containsKey(aprilID)){
                mecanumCommand.setFinalPosition(true, 30, getTargetX(-8.0), getTargetY(-5.0), getTargetTheta());
            }
            while(!mecanumCommand.isPositionReached(true, true)){}
        }

         */

        if(position.equals("left")) {
            mecanumCommand.setFinalPosition(true, 30, 66, 85, Math.PI/2);
        }
        else if(position.equals("middle")){
            mecanumCommand.setFinalPosition(true, 30, 76, 85, Math.PI/2);
        }
        else if(position.equals("right")){
            mecanumCommand.setFinalPosition(true, 30, 90, 85, Math.PI/2);
        }

        while(!mecanumCommand.isPositionReached(true,true)) {
        }
        //activate lift mode
        running = true;

        //activate raising (go to level 5, raising level)
        timer.reset();
        raising = true;

        //wait 1500 ms for the lift to raise
        while(timer.milliseconds() < 825){}

        //swing out arm and tilt
        timer.reset();
        outputCommand.armToBoard();
        outputCommand.tiltToBoard();

        while(timer.milliseconds() < 1600){}
        raising = false;

        //go to drop off level (1)
        level = 1;

        //TODO: put pixel dropoff here (open/close gate + timing)

        timer.reset();
        outputCommand.openGate();
        while(timer.milliseconds() < 250){}
        outputCommand.closeGate();
        outputCommand.outputWheelIn();
        while(timer.milliseconds() < 750){}

        //retract lift
        timer.reset();
        outputCommand.tiltToIdle();
        outputCommand.armToIdle();
        while(timer.milliseconds() < 1000){}
        multiMotorSubsystem.getPidUp().integralReset();
        level = 0;
        //lift mode gets stopped in the thread afterwards


        if(parkPlace.equalsIgnoreCase("left")){
            // Checkpoint
            mecanumCommand.setFinalPosition(true, 30, 9, 80, Math.PI / 2);
            while(!mecanumCommand.isPositionReached(true,true)) {}
            // Park
            mecanumCommand.setFinalPosition(true, 30, 9, 111, Math.PI / 2);
            while(!mecanumCommand.isPositionReached(true,true)) {}
        }
        else if(parkPlace.equalsIgnoreCase("right")){
            // Checkpoint
            mecanumCommand.setFinalPosition(true, 30, 133, 80, Math.PI / 2);
            while(!mecanumCommand.isPositionReached(true,true)) {}
            // Park
            mecanumCommand.setFinalPosition(true, 30, 133, 111, Math.PI / 2);
            while(!mecanumCommand.isPositionReached(true,true)) {}
        }

        stop();

        //LIFT DROPOFF
//        while(timer.milliseconds() < 3500) {
        //TODO: tune
        //heading: -1.5833333730697632
        //imu heading: 4.699851934109823
        //leftEncoder: 33559
        //rightEncoder: 0
        //x: 65.16827461821727
        //y: -187.29583391841874
//
//            if(position.equals("left")) {
//                while(mecanumCommand.isPositionReached(false,false)) {
//                    mecanumCommand.setFinalPosition(true, 30, 36.27, 81.69, 1.53);
//                }
//            }
//            else if(position.equals("middle")){
//                while(mecanumCommand.isPositionReached(false,false)) {
//                    mecanumCommand.setFinalPosition(true, 30, 36.27, 81.69, 1.53);
//                }
//            }
//            else if(position.equals("right")){
//                while(mecanumCommand.isPositionReached(false,false)) {
//                    mecanumCommand.setFinalPosition(true, 30, 36.27, 81.69, 1.53);
//                }
//            }
//        }
        //136, 0, `1.6
        //126, 140, 1.6
        //52, 220, 1.5
        //67, 222, 1.5
        //81, 222, 1.5
        //0, 222, 1.5
// 68, 6.3, -0.3

//        timer.reset();
//        if(opModeIsActive()) {
//            while (timer.milliseconds() < 500) {
//                outputCommand.openGate();
//            }
//            outputCommand.closeGate();
//            outputCommand.tiltToIdle();
//            outputCommand.armToIdle();
//            sleep(2000);
//            level = 0;
//            sleep(1000);
//        }
//        mecanumCommand.moveToGlobalPosition(10, -222, 1.6);
    }

    public void pidProcess(){
        while (opModeIsActive()) {
            mecanumCommand.pidProcess();
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
            packet.put("xReached", mecanumCommand.isXReached());
            packet.put("yReached", mecanumCommand.isYReached());
            packet.put("errorX", mecanumCommand.globalXController.getError());
            packet.put("integralSumX", mecanumCommand.globalXController.getIntegralSum());
            packet.put("errorY", mecanumCommand.globalYController.getError());
            packet.put("integralSumY", mecanumCommand.globalYController.getIntegralSum());

            telemetry.addData("x", gyroOdometry.x);
            telemetry.addData("y", gyroOdometry.y);
            telemetry.addData("theta", gyroOdometry.theta);
            telemetry.addData("position", position);
            telemetry.addData("errorOutput", mecanumCommand.globalXController.getError()*0.04);

            telemetry.addData("apriltagYdistance", aprilCamSubsystem.getAprilYDistance(1, 0));
            telemetry.addData("apriltagXdistance", aprilCamSubsystem.getAprilXDistance(1, 0));
            telemetry.addData("targetX", targetX);
            telemetry.addData("targetY", targetY);

//            packet.put("x", gyroOdometry.x);
//            packet.put("y", gyroOdometry.y);
            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }
    }
    public void liftProcess() {
        while(opModeIsActive() && running){
            if(raising){
                multiMotorCommand.LiftUpPositional(true, 5);
            }
            else {
                multiMotorCommand.LiftUpPositional(true, level);
            }
            if(running && (multiMotorSubsystem.getDerivativeValue() == 0 && multiMotorSubsystem.getPosition() < 5) || (multiMotorSubsystem.getDerivativeValue() < 0 && multiMotorSubsystem.getPosition() < -5)){
                level = 0;
                multiMotorSubsystem.reset();
                running = false;
            }
        }
    }

    public void motorProcess(){
        while (opModeIsActive()) {
            mecanumSubsystem.motorProcess();
        }
    }

    public void tagDetectionProcess(){
        while(opModeIsActive()) {
            aprilCamSubsystem.runDetections();
        }
    }


}