package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
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

import java.util.List;
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
    private List<LynxModule> allHubs; //GET ALL LYNX MODULES
    FtcDashboard dashboard;
    TelemetryPacket packet;
    int[] liftPositions = {0, 450, 1200, 2200, 4500, 1000};
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
        allHubs = hardwareMap.getAll(LynxModule.class);
        for(LynxModule hub : allHubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        } //BULK READS

        instantiateSubsystems();
        readyRobot();

        double propPosition = 0;
        while(opModeInInit() && !isStopRequested()){
            //TODO: determine which Xprop positions make left, middle, right
//            propPosition = webcamSubsystem.getXProp();
        }

        waitForStart();
        startThreads();

        String position = "right";

      //  Spike Drop-off
        moveToCheckpoint(71.5, 0, 0);
        switch (position) {
            case "left":
                moveTo(107.76, 19.99, 0);
                break;
            case "middle":
                moveTo(124.36, -12.48, 0);
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
//        dropPixel();
        moveToCheckpoint(71.5, 0, Math.PI / 2);
        moveToCheckpoint(71.5, -135, Math.PI / 2);
        moveToCheckpoint(95.5, -185, Math.PI / 2);
 //       pickupPixels();

        moveToCheckpoint(71.5, -135, Math.PI / 2);
        moveToCheckpoint(71.5, 0, Math.PI / 2);
        moveTo(90, 80, Math.PI / 2);
  //      dropPixel();



         //Parking
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
    }

    // Side Processes
    public void pidProcess(){
        while (opModeIsActive() && !isStopRequested()) {
            mecanumCommand.pidProcess();
        }
    }

    public void updateOdometry() {
        while (opModeIsActive() && !isStopRequested()) {
            imu.gyroProcess();
            gyroOdometry.process();
        }
    }

    public void updateTelemetry() {
        while (opModeIsActive() && !isStopRequested()) {
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
            packet.put("raising", raising);
            packet.put("running", running);
            packet.put("liftPos", multiMotorSubsystem.getPosition());
            packet.put("pidoutput", multiMotorSubsystem.getPidUp().getOutputPositionalValue());

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
        while(opModeIsActive() && !isStopRequested()){
//            if(running) {
                multiMotorCommand.LiftUp(running, level);
                if (level == 0 && (multiMotorSubsystem.getDerivativeValue() == 0 && multiMotorSubsystem.getPosition() < 40) || (multiMotorSubsystem.getDerivativeValue() < 0 && multiMotorSubsystem.getPosition() < -5)) {
                    level = /*-1*/0;
                    multiMotorSubsystem.reset();
                    running = false;
                }
//            }
        }
    }

    public void motorProcess(){
        while (opModeIsActive() && !isStopRequested()) {
            mecanumSubsystem.motorProcess();
        }
    }

    public void tagDetectionProcess(){
        while(opModeIsActive() && !isStopRequested()) {
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
        Executor executor = Executors.newFixedThreadPool(5);
        CompletableFuture.runAsync(this::updateOdometry, executor);
        CompletableFuture.runAsync(this::updateTelemetry, executor);
//        CompletableFuture.runAsync(this::liftProcess, executor);
        CompletableFuture.runAsync(this::pidProcess, executor);
        CompletableFuture.runAsync(this::motorProcess, executor);
        //CompletableFuture.runAsync(this::tagDetectionProcess);
    }
    private void pickupStack(){
        //move to stack
        moveToCheckpoint(133, 31, Math.PI / 2);
        //lower intake
        intakeCommand.halfIntake();
        //intake
        intakeCommand.intakeIn(0.5);
        //move to front
        moveToCheckpoint(80.5, 49, Math.PI / 2);
        sleep(400);

        // Couch out pixels that are stuck
        intakeCommand.intakeRollerOut();
        sleep(100);
        intakeCommand.intakeIn(0.5);
        sleep(400);

        //stop intake
        intakeCommand.stopIntake();
        //raise intake
        intakeCommand.raiseIntake();
        //move back and release excess pixels
        intakeCommand.intakeOut(0.5);
        moveToCheckpoint(133, 31, Math.PI / 2);
        waitTime(300);
    }
    private void raisingLift() {
        currentState = "raising lift";
        multiMotorSubsystem.reset();
        //set dropoff level
        level = 5;
        while(!multiMotorSubsystem.isPositionReached(liftPositions[level]) && !isStopRequested()){
            multiMotorCommand.LiftUpPositional(true, level);
        }

        outputCommand.armToBoard();
        outputCommand.tiltToBoard();
        timer.reset();
        while(timer.milliseconds() < 1000 && !isStopRequested()){
            multiMotorCommand.LiftUpPositional(true, level);
        }
        level = 1;
        while(!multiMotorSubsystem.isPositionReached(liftPositions[level]) && !isStopRequested()){
            multiMotorCommand.LiftUpPositional(true, level);
        }
        timer.reset();
        while(timer.milliseconds() < 1000 && !isStopRequested()){
            multiMotorCommand.LiftUpPositional(true, level);
        }

    }
    private void dropPixel() {
        currentState = "dropping pixel";
        //drop off
        outputCommand.openGate();

        timer.reset();
        level = 1;
        while(timer.milliseconds() < 250 && !isStopRequested()){
            multiMotorCommand.LiftUpPositional(true, level);
        }
        outputCommand.closeGate();
        outputCommand.outputWheelIn();
        timer.reset();
        while(timer.milliseconds() < 500 && !isStopRequested()){
            multiMotorCommand.LiftUpPositional(true, level);
        }
        outputCommand.outputWheelStop();
        timer.reset();
        while(timer.milliseconds() < 1500 && !isStopRequested()){
            multiMotorCommand.LiftUpPositional(true, level);
        }
    }
    public void lowerLift() {
        currentState = "lowering lift";

        level = 5;
        while(!multiMotorSubsystem.isPositionReached(liftPositions[level]) && !isStopRequested()){
            multiMotorCommand.LiftUpPositional(true, level);
        }
        outputCommand.tiltToIdle();
        outputCommand.armToIdle();
        timer.reset();
        while(timer.milliseconds() < 1000 && !isStopRequested()){
            multiMotorCommand.LiftUpPositional(true, level);
        }
        //retract lift
        level = 0;
        while(!((multiMotorSubsystem.getDerivativeValue() == 0 && multiMotorSubsystem.getPosition() < 40) || (multiMotorSubsystem.getDerivativeValue() < 0 && multiMotorSubsystem.getPosition() < -5))) {
            multiMotorCommand.LiftUpPositional(true, level);
        }
        multiMotorSubsystem.moveLift(0);

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
    public void pickupPixels(){
        intakeCommand.lowerIntake();
        intakeCommand.intakeIn(1);
        timer.reset();
        while(timer.milliseconds() < 1000);
        intakeCommand.raiseIntake();
        intakeCommand.stopIntake();
    }
}