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
import org.firstinspires.ftc.teamcode.util.GyroOdometry;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

@Autonomous(name="Thompson's Tests")
public class ThompsonsTests extends LinearOpMode {
    private MecanumSubsystem mecanumSubsystem;
    private MecanumCommand mecanumCommand;
    private IMUSubsystem imu;
    private OdometrySubsystem odometrySubsystem;
    private GyroOdometry gyroOdometry;
    private IntakeCommand intakeCommand;
//    private WebcamSubsystem webcamSubsystem;
    private OutputCommand outputCommand;
    private MultiMotorSubsystem multiMotorSubsystem;
    private MultiMotorCommand multiMotorCommand;
    FtcDashboard dashboard;
    TelemetryPacket packet;
    private ElapsedTime timer;
    private int level = -1;
    private String position = "initalized";

    private boolean running = false;
    private boolean raising = false;
    private String status = "Uninitialized";

    @Override
    public void runOpMode() throws InterruptedException {
        instantiateSubsystems();
        readyRobot();

        double propPosition = 0;
        while(opModeInInit()){
            //TODO: determine which Xprop positions make left, middle, right
//            propPosition = webcamSubsystem.getXProp();
        }

        waitForStart();
        startThreads();
        status = "Countdown: 3";
        sleep(1000);
        status = "Countdown: 2";
        sleep(1000);
        status = "Countdown: 1";
        sleep(1000);
        status = "Running set. level = 2";
        running = true;
        level = 2;
        sleep(3000);
        status = "level = 4";
        level = 4;
        sleep(3000);
        status = "level = 1";
        level = 1;
        sleep(3000);
        status = "level = 3";
        level = 3;
        sleep(3000);
        status = "raising";
        raising = true;
        sleep(10000);



//        Run this once the lift is actually working
//        status = "Moving to dropoff level 1";
//        //set dropoff level
//        level = 1;
//
//        while (!multiMotorSubsystem.isPositionReached() && !isStopRequested());
//
//        status = "Activating lift mode in raising";
//
//        //activate lift mode in raising
//        raising = true;
//        running = true;
//
//        //activate raising (go to level 5, raising level)
//        timer.reset();
//
//        //wait 1500 ms for the lift to raise
//        while(timer.milliseconds() < 5000){}
//
//        status = "Extending arm/tilt";
//        //swing out arm and tilt
//        timer.reset();
//        outputCommand.armToBoard();
//        outputCommand.tiltToBoard();
//
//        while(timer.milliseconds() < 5000){}
//
//
//        status = "Dropping Pixel";
//        raising = false;
//
//        //drop off
//        timer.reset();
//        outputCommand.openGate();
//        while(timer.milliseconds() < 5000){}
//
//        timer.reset();
//        outputCommand.closeGate();
//        outputCommand.outputWheelIn();
//        while(timer.milliseconds() < 5000){}
//
//        status = "Retracting Lift";
//        //retract lift
//        timer.reset();
//        outputCommand.tiltToIdle();
//        outputCommand.armToIdle();
//        while(timer.milliseconds() < 5000){}
//        multiMotorSubsystem.getPidUp().integralReset();
//        level = 0;
//        //lift mode gets stopped in the thread afterwards

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
            packet.put("errorX", mecanumCommand.globalXController.getError()*0.04);
            packet.put("integralSumX", mecanumCommand.globalXController.getIntegralSum());
            packet.put("errorY", mecanumCommand.globalYController.getError()*0.04);
            packet.put("integralSumY", mecanumCommand.globalYController.getIntegralSum());

            telemetry.addData("x", gyroOdometry.x);
            telemetry.addData("y", gyroOdometry.y);
            telemetry.addData("theta", gyroOdometry.theta);
            telemetry.addData("lift power",
                    multiMotorSubsystem.getPidUp().outputPositional(1000,
                            multiMotorSubsystem.getPosition()
                    )
            );
            telemetry.addData("status", status);

            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }
    }
    public void liftProcess() {
        // This function may be workth testing if the lift breaks again
        // multiMotorSubsystem.LiftPositionalProcess(200);

        while(opModeIsActive()){
            if (running) {
                multiMotorCommand.LiftUpPositional(true, level);
                if ((level == 0 &&
                        (multiMotorSubsystem.getDerivativeValue() == 0
                                && multiMotorSubsystem.getPosition() < 5))
                        || (multiMotorSubsystem.getDerivativeValue() < 0
                        && multiMotorSubsystem.getPosition() < -5)) {
                    multiMotorSubsystem.reset();
                    running = false;
                }
            }
        }
    }

    public void motorProcess(){
        while (opModeIsActive()) {
            mecanumSubsystem.motorProcess();
        }
    }

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
        //aprilCamSubsystem = new AprilCamSubsystem(hardwareMap);
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

        //set dropoff level
        while(opModeIsActive() && !isStopRequested()) {
            level = 5;
        }

        //activate lift mode in raising
        running = true;

        //activate raising (go to level 5, raising level)
        timer.reset();

        //wait 1500 ms for the lift to raise
        while(timer.milliseconds() < 1500){}

        //swing out arm and tilt
        timer.reset();
        outputCommand.armToBoard();
        outputCommand.tiltToBoard();

        while(timer.milliseconds() < 1600){}
        level = 1;

        //drop off
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