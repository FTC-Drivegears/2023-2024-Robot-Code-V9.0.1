package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
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
    private int level = 5;
    private String position = "initalized";
    private double targetX = 0;
    private double targetY = 0;

    private Integer aprilID = 2;

    private boolean goToAprilTag = false;

    private String autoColor = "blue"; // or "red"

    private String parkPlace = "left";
    private boolean running = false;
    private boolean raising = false;
    private String currentState = "";


    @Override
    public void runOpMode() throws InterruptedException {
        instantiateSubsystems();
        readyRobot();

        double propPosition = 0;
        while(opModeInInit() && !isStopRequested()){
            //TODO: determine which Xprop positions make left, middle, right
//            propPosition = webcamSubsystem.getXProp();
        }

        waitForStart();
        startThreads();

        String position = "left";

        //Spike Drop-off
        moveToCheckpoint(71.5, 0, 0);
        switch (position) {
            case "left":
                moveTo(107.76, 19.99, 0);
                break;
            case "middle":
                moveTo(120.36, -12.48, 0);
                break;
            case "right":
                moveTo(69.48, -22, 2.11);
                break;
        }
        releaseIntakePixel();

        //Middle Back
        moveToCheckpoint(133, 31, Math.PI / 2);

        //Middle Front
        moveToCheckpoint(80.5, 49, Math.PI / 2);

        // Detecting April Tag Code
        //goToAprilTag = true;
        //sleep(1000);
        //
        //while(goToAprilTag && !isStopRequested()) {
        //    if(aprilCamSubsystem.getHashmap().containsKey(aprilID)){
        //        mecanumCommand.setFinalPosition(true, 30, getTargetX(-8.0), getTargetY(-5.0), getTargetTheta());
        //    }
        //    while(!mecanumCommand.isPositionReached(true, true)){}
        //}

        // Pixel Board Drop-off
        switch (position) {
            case "left":
                moveTo(66, 80, Math.PI / 2);
                break;
            case "middle":
                moveTo(76, 80, Math.PI / 2);
                break;
            case "right":
                moveTo(90, 80, Math.PI / 2);
                break;
        }
        dropPixel();

//         Parking
        if (parkPlace.equalsIgnoreCase("left")) {
            // Checkpoint
            moveToCheckpoint(9, 80, Math.PI / 2);
            // Park
            moveTo(9, 111, Math.PI / 2);
        } else {
            // Checkpoint
            moveToCheckpoint(133, 80, Math.PI / 2);
            // Park
            moveTo(133, 111, Math.PI / 2);
        }

        stop();
    }

    // Side Processes
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
            packet.put("currentState", currentState);
            packet.put("level", level);

            telemetry.addData("x", gyroOdometry.x);
            telemetry.addData("y", gyroOdometry.y);
            telemetry.addData("theta", gyroOdometry.theta);
            telemetry.addData("position", position);
            telemetry.addData("errorOutput", mecanumCommand.globalXController.getError()*0.04);
            telemetry.addData("apriltagYdistance", aprilCamSubsystem.getAprilYDistance(1, 0));
            telemetry.addData("apriltagXdistance", aprilCamSubsystem.getAprilXDistance(1, 0));
            telemetry.addData("targetX", targetX);
            telemetry.addData("targetY", targetY);
            telemetry.addData("timer", timer.milliseconds());
            telemetry.addData("raising", raising);
            telemetry.addData("running", running);
            telemetry.addData("liftPos", multiMotorSubsystem.getPosition());
            telemetry.addData("pidoutput", multiMotorSubsystem.getPidUp().getOutputPositionalValue());

            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }
    }
    public void liftProcess() {
        while(opModeIsActive()){
            if(running) {
                multiMotorCommand.LiftUpPositional(true, level);
                if (level == 0 && running && (multiMotorSubsystem.getDerivativeValue() == 0 && multiMotorSubsystem.getPosition() < 5) || (multiMotorSubsystem.getDerivativeValue() < 0 && multiMotorSubsystem.getPosition() < -5)) {
                    multiMotorSubsystem.reset();
                    multiMotorSubsystem.getPidUp().integralReset();
                    running = false;
                }
            }
            else{
                multiMotorSubsystem.reset();
                multiMotorSubsystem.getPidUp().integralReset();
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

    // Helper/Command Functions

    private void instantiateSubsystems() {
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
    }

    private void readyRobot() {
        imu.resetAngle();
        odometrySubsystem.reset();
        intakeCommand.raiseIntake();
        outputCommand.closeGate();
        outputCommand.armToIdle();
        outputCommand.tiltToIdle();
        multiMotorSubsystem.reset();
    }

    private void startThreads() {
        Executor executor = Executors.newFixedThreadPool(6);
        CompletableFuture.runAsync(this::updateOdometry, executor);
        CompletableFuture.runAsync(this::updateTelemetry, executor);
        CompletableFuture.runAsync(this::liftProcess, executor);
        CompletableFuture.runAsync(this::pidProcess, executor);
        CompletableFuture.runAsync(this::motorProcess, executor);
        //CompletableFuture.runAsync(this::tagDetectionProcess);
    }
    private void dropPixel() {
        currentState = "dropping";
        //set dropoff level
        level = 5;

        //activate lift mode in raising
        running = true;

        //activate raising (go to level 5, raising level)
        timer.reset();


        while(!multiMotorSubsystem.isPositionReached(950));
        outputCommand.armToBoard();
        outputCommand.tiltToBoard();
        waitTime(1000);
        level = 1;
        while(!multiMotorSubsystem.isPositionReached(450));
        waitTime(250);

        //drop off
        outputCommand.openGate();
        waitTime(250);
        outputCommand.closeGate();
        outputCommand.outputWheelIn();
        waitTime(500);
        outputCommand.outputWheelStop();
        waitTime(1500);

        level = 5;
        while(!multiMotorSubsystem.isPositionReached(950));
        outputCommand.tiltToIdle();
        outputCommand.armToIdle();
        waitTime(1700);
        //retract lift
        level = 0;
        while (running && !isStopRequested());
        multiMotorSubsystem.reset();

        //lift mode gets stopped in the thread afterwards

    }
    private void waitTime(double milliseconds){
        timer.reset();
        while(timer.milliseconds() < milliseconds && !isStopRequested()){}
    }

    private void moveTo(double x, double y, double theta) {
        mecanumCommand.setFinalPosition(true, 30, x, y, theta);
        while (!mecanumCommand.isPositionReached(true, true) && !isStopRequested()) ;
    }

    private void moveToCheckpoint(double x, double y, double theta) {
        mecanumCommand.setFinalPosition(true, 30, x, y, theta);
        while (!mecanumCommand.isPositionPassed() && !isStopRequested()) ;
    }

    private void releaseIntakePixel() {
        //release pixel
        intakeCommand.lowerIntake();
        intakeCommand.intakeOut(0.5);
        timer.reset();
        while(timer.milliseconds() < 1000);
        intakeCommand.raiseIntake();
        intakeCommand.stopIntake();
    }

}