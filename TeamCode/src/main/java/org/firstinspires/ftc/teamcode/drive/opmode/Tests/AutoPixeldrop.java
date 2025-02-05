package org.firstinspires.ftc.teamcode.drive.opmode.Tests;

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

@Autonomous(name="pixel drop test", group="tests")
public class AutoPixeldrop extends LinearOpMode {
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
    int[] liftPositions = {0, 450, 1200, 2200, 4500, 1000, 250};
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
    private String status = "Uninitialized";

    @Override
    public void runOpMode() throws InterruptedException {

        instantiateSubsystems();
        readyRobot();
//
//        double propPosition = 0;

        waitForStart();
        releaseIntakePixel();
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

    public void motorProcess(){
        while (opModeIsActive() && !isStopRequested()) {
            mecanumSubsystem.motorProcess();
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
//        webcamSubsystem = new WebcamSubsystem(hardwareMap, WebcamSubsystem.PipelineName.CONTOUR_BLUE);
//        aprilCamSubsystem = new AprilCamSubsystem(hardwareMap);
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
        Executor executor = Executors.newFixedThreadPool(4);
        CompletableFuture.runAsync(this::updateOdometry, executor);
        CompletableFuture.runAsync(this::updateTelemetry, executor);
        CompletableFuture.runAsync(this::pidProcess, executor);
        CompletableFuture.runAsync(this::motorProcess, executor);
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
        while(timer.milliseconds() < 600 && !isStopRequested()){
            multiMotorCommand.LiftUpPositional(true, level);
        }
        level = 6;
        timer.reset();
        while((!multiMotorSubsystem.isPositionReached(liftPositions[level]) || timer.milliseconds() < 1500) && !isStopRequested()){
            multiMotorCommand.LiftUpPositional(true, level);
        }
//        timer.reset();
//        while(timer.milliseconds() < 1000 && !isStopRequested()){
//            multiMotorCommand.LiftUpPositional(true, level);
//        }

    }
    private void dropPixel() {
        currentState = "dropping pixel";
        timer.reset();
        while(timer.milliseconds() < 250 && !isStopRequested()){
            outputCommand.outputWheelIn();
            multiMotorCommand.LiftUpPositional(true, level);
        }
        outputCommand.outputWheelStop();
        //drop off
        outputCommand.openGate();

        timer.reset();
        level = 6;
        while(timer.milliseconds() < 1000 && !isStopRequested()){
            multiMotorCommand.LiftUpPositional(true, level);
        }
        outputCommand.closeGate();
        outputCommand.outputWheelIn();
        timer.reset();
        while(timer.milliseconds() < 500 && !isStopRequested()){
            multiMotorCommand.LiftUpPositional(true, level);
        }
        outputCommand.outputWheelStop();
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
        while(!((multiMotorSubsystem.getDerivativeValue() == 0 && multiMotorSubsystem.getPosition() < 40) || (multiMotorSubsystem.getDerivativeValue() < 0 && multiMotorSubsystem.getPosition() < -5)) && !isStopRequested()) {
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
        intakeCommand.intakeOut(0.65);
        timer.reset();
        while(timer.milliseconds() < 1500);
        intakeCommand.stopIntake();
        intakeCommand.raiseIntake();
    }
    public void pickupPixels(){
        intakeCommand.lowerIntake();
        intakeCommand.intakeIn(0.7);
        intakeCommand.intakeRollerIn();
        mecanumCommand.setFinalPosition(true, 30,40, -170, 2);
        timer.reset();
        while(timer.milliseconds() < 2500);
        intakeCommand.raiseIntake();
        intakeCommand.intakeOut(1);
        mecanumCommand.setFinalPosition(true, 30, 55, -187, 2);
        timer.reset();
        while(timer.milliseconds() < 1000);
        intakeCommand.stopIntake();
    }
}