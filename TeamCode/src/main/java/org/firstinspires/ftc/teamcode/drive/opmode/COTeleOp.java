package org.firstinspires.ftc.teamcode.drive.opmode;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.DashboardCore;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.command.IntakeCommand;
import org.firstinspires.ftc.teamcode.command.MecanumCommand;
import org.firstinspires.ftc.teamcode.command.MultiMotorCommand;
import org.firstinspires.ftc.teamcode.command.OutputCommand;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IMUSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MultiMotorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.util.GridAutoCentering;
import org.firstinspires.ftc.teamcode.util.GyroOdometry;
import org.firstinspires.ftc.teamcode.util.TimerList;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

@TeleOp
public class COTeleOp extends LinearOpMode {
    private FtcDashboard dashboard;
    private TelemetryPacket packet;
    private MultiMotorSubsystem multiMotorSubsystem;
    private MultiMotorCommand multiMotorCommand;
    private OdometrySubsystem odometrySubsystem;
    private MecanumSubsystem mecanumSubsystem;
    private MecanumCommand mecanumCommand;
//    private OutputCommand outputCommand;
    private IntakeCommand intakeCommand;
    private IMUSubsystem imuSubsystem;
    private GyroOdometry gyroOdometry;
    private GridAutoCentering gridAutoCentering;
    private OutputCommand outputCommand;
    private ColorSensorSubsystem colorSensorSubsystem;
    private ElapsedTime pixelTimer, liftTimer;
    private int raiseLevel = 5;
    private int level = -1;
    private int pixelCounter = 0;
    private boolean running = true;
    private boolean raising = false;
    private String color1 = "none";
    private String color2 = "none";
    private int colorCounter = 0;

    private enum RUNNING_STATE {
        LIFT_STOP,
        RETRACT_LIFT,
        RAISE_LIFT,
        DROP
    }
    private RUNNING_STATE state = RUNNING_STATE.LIFT_STOP;

    private final TimerList timerList = new TimerList();
//    private Servo shooterServo;
//    private Servo rightHang;

    @SuppressLint("SuspiciousIndentation")
    @Override
    public void runOpMode() throws InterruptedException {
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
        colorSensorSubsystem = new ColorSensorSubsystem(hardwareMap);

        imuSubsystem = new IMUSubsystem(hardwareMap);

        mecanumSubsystem = new MecanumSubsystem(hardwareMap); //MAKE SURE THIS IS INSTANTIATED BEFORE ODOMETRY SUBSYSTEM

        odometrySubsystem = new OdometrySubsystem(hardwareMap);

        gyroOdometry = new GyroOdometry(odometrySubsystem,imuSubsystem);

        mecanumCommand = new MecanumCommand(mecanumSubsystem, odometrySubsystem,  gyroOdometry, this);

        intakeCommand = new IntakeCommand(hardwareMap);
        outputCommand = new OutputCommand(hardwareMap);

        gridAutoCentering = new GridAutoCentering(mecanumSubsystem, gyroOdometry);

//        colorSensorSubsystem = new ColorSensorSubsystem(hardwareMap);
        multiMotorSubsystem = new MultiMotorSubsystem(hardwareMap, true, MultiMotorSubsystem.MultiMotorType.dualMotor);
        multiMotorCommand = new MultiMotorCommand(multiMotorSubsystem);

//        shooterServo = hardwareMap.get(Servo.class, "leftHang");
//        rightHang = hardwareMap.get(Servo.class,"rightHang");
//
//        rightHang.setDirection(Servo.Direction.REVERSE);
//        shooterServo.setDirection(Servo.Direction.REVERSE);

        pixelTimer = new ElapsedTime();
        liftTimer = new ElapsedTime();

        while(opModeInInit()) {
            odometrySubsystem.reset();
            imuSubsystem.resetAngle();

            intakeCommand.lowerIntake();
            outputCommand.closeGate();

            outputCommand.armToIdle();
            outputCommand.tiltToIdle();

            pixelCounter = 0;
            timerList.resetTimer("colorLoop");
        }

//        disableAutoLift = false;

        waitForStart();

        Executor executor = Executors.newFixedThreadPool(4);
//        CompletableFuture.runAsync(this::updateTelemetry, executor);
        CompletableFuture.runAsync(this::odometryProcess, executor);
        CompletableFuture.runAsync(this::LiftProcess, executor);
//        CompletableFuture.runAsync(this::sensorUpdate, executor);
        CompletableFuture.runAsync(this::motorProcess);

        while(opModeIsActive()) {

            //setting levels for running lift
            if (state == RUNNING_STATE.LIFT_STOP) {
                //set lift level
                if (gamepad1.a) {
                    running = true;
                    timerList.resetTimer("raiseLift");
                    raising = true;
                    level = 1;
                    state = RUNNING_STATE.RAISE_LIFT;
                    timerList.resetTimer("armTilt");
                    pixelCounter = 0;
                } else if (gamepad1.b) {
                    running = true;
                    timerList.resetTimer("raiseLift");
                    raising = true;
                    level = 2;
                    state = RUNNING_STATE.RAISE_LIFT;
                    timerList.resetTimer("armTilt");
                    pixelCounter = 0;
                } else if (gamepad1.y) {
                    running = true;
                    timerList.resetTimer("raiseLift");
                    raising = true;
                    level = 3;
                    state = RUNNING_STATE.RAISE_LIFT;
                    timerList.resetTimer("armTilt");
                    pixelCounter = 0;
                } else if (gamepad1.x) {
                    running = true;
                    timerList.resetTimer("raiseLift");
                    raising = true;
                    level = 4;
                    state = RUNNING_STATE.RAISE_LIFT;
                    timerList.resetTimer("armTilt");
                    pixelCounter = 0;
                }
            }
            //when lift is raised
            if (state == RUNNING_STATE.RAISE_LIFT) {
                if(timerList.checkTimePassed("raiseLift", 825)) {
                    outputCommand.armToBoard();
                    outputCommand.tiltToBoard();
                    if(timerList.checkTimePassed("raiseLift", 1600)){
                        raising = false;
                    }
                }
                //change state
                if(gamepad2.right_bumper){
                    if(pixelCounter != 0 || timerList.checkTimePassed("armTilt", 1000)) {
                        //drop pixel (one)
                        pixelCounter += 1;
                        timerList.resetTimer("pixelDrop");
                        outputCommand.outputWheelStop();
                        state = RUNNING_STATE.DROP;
                    }
                }
                if(gamepad2.b && timerList.checkTimePassed("armTilt", 1000)){
                    timerList.resetTimer("liftTimer");
                    state = RUNNING_STATE.RETRACT_LIFT;
                }
            }

            if(gamepad1.dpad_right){
                imuSubsystem.resetAngle();

            }

            if(state == RUNNING_STATE.DROP){
                raising = false;
                if(!timerList.checkTimePassed("pixelDrop", 750)) {
                    if (timerList.checkTimePassed("pixelDrop", 250)) {
                        outputCommand.closeGate();
                        outputCommand.outputWheelIn();
                    } else {
                        outputCommand.openGate();
                    }
                } else { //if pixel is done dropping, reset the state
                    state = RUNNING_STATE.RAISE_LIFT;
                }
            }
            if(state == RUNNING_STATE.RETRACT_LIFT){
                raising = true;
                outputCommand.tiltToIdle();
                outputCommand.armToIdle();
                if(timerList.checkTimePassed("liftTimer", 1700)){
                    raising = false;
                    level = 0;
                }
                //TODO: potentially implement
                if(multiMotorSubsystem.getDerivativeValue() == 0 && multiMotorSubsystem.getPosition() < 5){
                    pixelCounter = 0;
                    level = /*-1*/0;
                    state = RUNNING_STATE.LIFT_STOP;
                }
            }
            //emergency lift controls
            if(Math.abs(gamepad2.left_stick_y) > 0.8){
                raising = false;
                running = false;
                state = RUNNING_STATE.DROP;
                level = 0;
                multiMotorSubsystem.moveLift(-gamepad2.right_stick_y);
                if(gamepad1.right_bumper){
                    timerList.resetTimer("pixelDrop");
                }
            }
            else{
                running = true;
            }


            if(gamepad1.dpad_left) {
                imuSubsystem.resetAngle();
            }
            else if(gamepad2.x){
                raising = false;
                level = 2;
                timerList.resetTimer("liftTimer");
                state = RUNNING_STATE.RETRACT_LIFT;
            }
            else if(gamepad2.y){
                raising = false;
                //reset the lift condition after manually reaching the bottom
                state = RUNNING_STATE.LIFT_STOP;
                multiMotorSubsystem.reset();
                level = -1;
            }
            if(gamepad2.dpad_up){
                intakeCommand.raiseIntake();
            }
            else if(gamepad2.dpad_down){
                intakeCommand.lowerIntake();
            }

            if(gamepad2.dpad_right){
                outputCommand.droneToShoot();
            }
            else if(gamepad2.dpad_left){
                outputCommand.droneToNotShoot();
            }
            else {
                outputCommand.droneToIdle();
            }

            //intake
            if(gamepad2.right_trigger > 0.5){
                intakeCommand.intakeIn(0.8);
            }
            else if(gamepad2.left_trigger > 0.5){
                intakeCommand.intakeOut(0.5);
            }
            else{
                intakeCommand.stopIntake();
            }

//            if(gamepad1.left_bumper){
//                shooterServo.setPosition(0.5);
//                rightHang.setPosition(0);
//
//            }
//            else{
//                shooterServo.setPosition(0);
//                rightHang.setPosition(0.28);
//            }

            //TODO: auto center/change zero
            updateTelemetry();
//            lightProcess();
            mecanumCommand.moveGlobalPartial(true, gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x);
        }

    }
    public void motorProcess(){
        while(opModeIsActive()){
            mecanumSubsystem.motorProcessTeleOp();
        }
    }
    public void LiftProcess(){
        while(opModeIsActive()){
            if(raising){
                multiMotorCommand.LiftUpPositional(running, raiseLevel);
            }
            else {
                multiMotorCommand.LiftUpPositional(running, level);
            }
        }
    }
    public void odometryProcess(){
        while(opModeIsActive()){
            imuSubsystem.gyroProcess();
            gyroOdometry.process();
        }
    }

//    public void lightProcess(){
//        //light pattern sequence (ORDER OF THE IF-ELSE STATEMENTS MATTER)
//        if(timerList.checkTimePassed("colorLoop", 1425) || (color2.equals("none") && timerList.checkTimePassed("colorLoop", 100))){
//            colorSensorSubsystem.setColor(color1);
//            timerList.resetTimer("colorLoop");
//            //color 1 on at end of blink sequence, resets timer (so start from the else statement again and come back up)
//            //or stay on if color 2 is missing
//        }
//        else if(timerList.checkTimePassed("colorLoop", 1350)){
//            colorSensorSubsystem.setColor("none");
//            //off for 75ms
//        } else if(timerList.checkTimePassed("colorLoop", 1250)){
//            colorSensorSubsystem.setColor(color2);
//            //color 2 on for 100ms
//        } else if(timerList.checkTimePassed("colorLoop", 1175)){
//            colorSensorSubsystem.setColor("none");
//            //off for 75ms
//        } else if(timerList.checkTimePassed("colorLoop", 1075)){
//            colorSensorSubsystem.setColor(color2);
//            //color 2 on for 100 ms
//        } else if(timerList.checkTimePassed("colorLoop", 1000)) {
//            colorSensorSubsystem.setColor("none");
//            //off for 75 ms
//        } else {
//            //color 1 on for 1 second
//            colorSensorSubsystem.setColor(color1);
//        }
//    }
//
//    //i moved sensor update since color sensor read times are so fucking long itll screw up the light color timings
//    public void sensorUpdate(){
//        timerList.resetTimer("sensorLoop");
//        while(opModeIsActive()) {
//            if(timerList.checkTimePassed("sensorLoop", 1500)) {
//                color1 = colorSensorSubsystem.findColor1();
//                color2 = colorSensorSubsystem.findColor2();
//                timerList.resetTimer("sensorLoop");
//            }
//        }
//    }
    public void updateTelemetry(){
        packet.put("lift output", multiMotorSubsystem.getPower());
        telemetry.addData("x", gyroOdometry.x);
        telemetry.addData("y", gyroOdometry.y);
        telemetry.addData("theta", gyroOdometry.theta);
        telemetry.addData("lift level", level);
        telemetry.addData("lift state", state);
        telemetry.addData("pixelnumber", pixelCounter);
        telemetry.addData("raiseLiftTimer", timerList.getTimer("raiseLift"));
        telemetry.addData("liftTimer", timerList.getTimer("liftTimer"));
        telemetry.addData("armTilt", timerList.getTimer("armTilt"));
        telemetry.addData("lift position", multiMotorSubsystem.getPosition());
        telemetry.addData("color1", color1);
        telemetry.addData("color2", color2);
        telemetry.addData("lift velocity", multiMotorSubsystem.getDerivativeValue());
        telemetry.addData("cascadeOutput",multiMotorSubsystem.getCascadeOutput());
        dashboard.sendTelemetryPacket(packet);
        telemetry.update();
    }

}
