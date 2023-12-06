package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

@TeleOp
public class COTeleOp extends LinearOpMode {
    private MultiMotorSubsystem multiMotorSubsystem;
    private MultiMotorCommand multiMotorCommand;
    private OdometrySubsystem odometrySubsystem;
    private MecanumSubsystem mecanumSubsystem;
    //    private MecanumCommand mecanumCommand;
//    private OutputCommand outputCommand;
    private IntakeCommand intakeCommand;
    private IMUSubsystem imuSubsystem;
    private GyroOdometry gyroOdometry;
    private GridAutoCentering gridAutoCentering;
    private OutputCommand outputCommand;
    private ColorSensorSubsystem colorSensorSubsystem;
    private ElapsedTime pixelTimer, liftTimer;
    private int level = -1;
    private int pixelCounter;

    private enum RUNNING_STATE {
        LIFT_STOP,
        RETRACT_LIFT,
        RAISE_LIFT,
        DROP
    }
    private RUNNING_STATE state = RUNNING_STATE.LIFT_STOP;

    @Override
    public void runOpMode() throws InterruptedException {
        multiMotorSubsystem = new MultiMotorSubsystem(hardwareMap, true, MultiMotorSubsystem.MultiMotorType.dualMotor);
        multiMotorCommand = new MultiMotorCommand(multiMotorSubsystem);

        imuSubsystem = new IMUSubsystem(hardwareMap);

        mecanumSubsystem = new MecanumSubsystem(hardwareMap); //MAKE SURE THIS IS INSTANTIATED BEFORE ODOMETRY SUBSYSTEM

        gyroOdometry = new GyroOdometry(odometrySubsystem, imuSubsystem);
        odometrySubsystem = new OdometrySubsystem(hardwareMap);

//        mecanumCommand = new MecanumCommand(mecanumSubsystem, odometrySubsystem,  gyroOdometry, this);

        intakeCommand = new IntakeCommand(hardwareMap);
        outputCommand = new OutputCommand(hardwareMap);

        gridAutoCentering = new GridAutoCentering(mecanumSubsystem, gyroOdometry);

        colorSensorSubsystem = new ColorSensorSubsystem(hardwareMap);

        pixelTimer = new ElapsedTime();
        liftTimer = new ElapsedTime();

        odometrySubsystem.reset();
        imuSubsystem.resetAngle();

        intakeCommand.lowerIntake();
        outputCommand.closeGate();

        outputCommand.armToIdle();
        outputCommand.tiltToIdle();
        pixelCounter = 0;

//        disableAutoLift = false;

        waitForStart();

        Executor executor = Executors.newFixedThreadPool(7);
        CompletableFuture.runAsync(this::LiftProcess, executor);
        CompletableFuture.runAsync(this::odometryProcess, executor);

        //emergency lift change for owen

        while(opModeIsActive()) {
            mecanumSubsystem.fieldOrientedMove(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x, imuSubsystem.getTheta());

            //setting levels for running lift
            if (state == RUNNING_STATE.LIFT_STOP) {
                //set lift level
                if (gamepad1.a) {
                    level = 1;
                    state = RUNNING_STATE.RAISE_LIFT;
                } else if (gamepad1.b) {
                    level = 2;
                    state = RUNNING_STATE.RAISE_LIFT;
                } else if (gamepad1.y) {
                    level = 3;
                    state = RUNNING_STATE.RAISE_LIFT;
                } else if (gamepad1.x) {
                    level = 4;
                    state = RUNNING_STATE.RAISE_LIFT;
                }
            }

            //when lift is raised
            if (state == RUNNING_STATE.RAISE_LIFT) {
                outputCommand.armToBoard();
                outputCommand.tiltToBoard();
                //change state
                if(gamepad2.right_bumper){
                    //drop pixel (one)
                    dropPixel();
                    pixelCounter +=1;
                    state = RUNNING_STATE.DROP;
                }
            }

            if(state == RUNNING_STATE.DROP){
                if(gamepad2.right_bumper){
                    //drop second pixel
                    dropPixel();
                    pixelCounter += 1;

                }
                else if(gamepad2.b && pixelCounter >= 1){
                    //retract
                    liftTimer.reset();
                    state = RUNNING_STATE.RETRACT_LIFT;
                }
            }

            if(state == RUNNING_STATE.RETRACT_LIFT){
                outputCommand.tiltToIdle();
                outputCommand.armToIdle();
                if(liftTimer.milliseconds() > 2000){
                    level = 0;
                }
                if(multiMotorSubsystem.getPosition() < 18){
                    pixelCounter = 0;
                    state = RUNNING_STATE.LIFT_STOP;
                }
            }


            //emergency lift controls
            if(Math.abs(gamepad2.left_stick_y) > 0.8){
                state = RUNNING_STATE.LIFT_STOP;
                multiMotorSubsystem.moveLift(-gamepad2.right_stick_y);
            }
            if(gamepad2.x){
                level = 2;
                liftTimer.reset();
                state = RUNNING_STATE.RETRACT_LIFT;
            }
            else if(gamepad2.y){
                //reset the lift condition after manually reaching the bottom
                state = RUNNING_STATE.LIFT_STOP;
                multiMotorSubsystem.reset();
                level = 0;
            }
            if(gamepad2.dpad_up){
                intakeCommand.raiseIntake();
            }
            else if(gamepad2.dpad_down){
                intakeCommand.lowerIntake();
            }

            //intake
            if(gamepad2.right_trigger > 0.5){
                intakeCommand.intakeIn(0.9);
            }
            else if(gamepad2.left_trigger > 0.5){
                intakeCommand.intakeOut(0.7);
            }
            else{
                intakeCommand.stopIntake();
            }

            //TODO: auto center/change zero
            updateTelemetry();
        }

    }

    public void LiftProcess(){
        while(opModeIsActive()){
            if(state != RUNNING_STATE.LIFT_STOP) {
                multiMotorCommand.LiftUp(true,level);
            }
        }
    }

    public void odometryProcess(){
        while(opModeIsActive()){
            gyroOdometry.odometryProcess();
        }
    }
    public void updateTelemetry(){
        telemetry.addData("x", gyroOdometry.x);
        telemetry.addData("y", gyroOdometry.y);
        telemetry.addData("theta", gyroOdometry.theta);
        telemetry.addData("lift level", level);
        telemetry.addData("lift state", state);
        telemetry.addData("pixelnumber", pixelCounter);
        telemetry.addData("pixeltimer time", pixelTimer.milliseconds());
        telemetry.addData("liftTimer time", liftTimer.milliseconds());
        telemetry.update();
    }

    public void dropPixel(){

        outputCommand.outputWheelStop();
        pixelTimer.reset();
        while(pixelTimer.milliseconds() < 700) {
            if (pixelTimer.milliseconds() >= 250) {
                outputCommand.closeGate();
                outputCommand.outputWheelIn();
            } else {
                outputCommand.openGate();
            }
        }

    }
}
