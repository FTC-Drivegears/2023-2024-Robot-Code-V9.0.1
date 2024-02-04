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
import org.firstinspires.ftc.teamcode.subsystems.WebcamSubsystem;
import org.firstinspires.ftc.teamcode.util.GyroOdometry;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

@Autonomous(name="Thompson's Tests")
public class ThompsonsTests extends LinearOpMode {

    // Systems, Commands, and Utils
    private GyroOdometry gyroOdometry;
    private IMUSubsystem imu;
    private IntakeCommand intakeCommand;
    private MecanumCommand mecanumCommand;
    private MecanumSubsystem mecanumSubsystem;
    private MultiMotorCommand multiMotorCommand;
    private MultiMotorSubsystem multiMotorSubsystem;
    private OdometrySubsystem odometrySubsystem;
    private OutputCommand outputCommand;
    private WebcamSubsystem webcamSubsystem;

    FtcDashboard dashboard;
    TelemetryPacket packet;
    private ElapsedTime timer;
    private int level = -1;
    private boolean running = false;
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

    }

    public void updateTelemetry() {
        while (opModeIsActive()) {
            telemetry.addData("spike location", webcamSubsystem.findSpikePosition());
            telemetry.addData("april tags", webcamSubsystem.getDetections());
            telemetry.update();
        }
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

    /**
     * Create the required subsystems.
     */
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
        webcamSubsystem = new WebcamSubsystem(hardwareMap, WebcamSubsystem.PipelineName.CONTOUR_BLUE);
        timer = new ElapsedTime();
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
    }

    /**
     * Put robot in a ready position.
     */
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

        status = "Moving to dropoff level 1";
        //set dropoff level
        level = 1;

        while (!multiMotorSubsystem.isPositionReached() && !isStopRequested());

        status = "Activating lift mode in raising";

        //activate lift mode in raising
        running = true;

        //activate raising (go to level 5, raising level)
        timer.reset();

        //wait 1500 ms for the lift to raise
        while(timer.milliseconds() < 5000){}

        status = "Extending arm/tilt";
        //swing out arm and tilt
        timer.reset();
        outputCommand.armToBoard();
        outputCommand.tiltToBoard();

        while(timer.milliseconds() < 5000){}


        status = "Dropping Pixel";
//        raising = false;

        //drop off
        timer.reset();
        outputCommand.openGate();
        while(timer.milliseconds() < 5000){}

        timer.reset();
        outputCommand.closeGate();
        outputCommand.outputWheelIn();
        while(timer.milliseconds() < 5000){}

        status = "Retracting Lift";
        //retract lift
        timer.reset();
        outputCommand.tiltToIdle();
        outputCommand.armToIdle();
        while(timer.milliseconds() < 5000){}
        multiMotorSubsystem.getPidUp().integralReset();
        level = 0;
        //lift mode gets stopped in the thread afterwards


        //set dropoff level
        while(opModeIsActive() && !isStopRequested()) {
            level = 5;
        }

        //activate lift mode in raising
        running = true;
        level = 5;

        /*
        Design idea: in subsystems, write functions that return Threads.
        We can run the join() methods to wait for them
         */
        while (opModeIsActive() && !isStopRequested()
                && !multiMotorSubsystem.isPositionReached()
        );

        //swing out arm and tilt
        timer.reset();
        outputCommand.armToBoard();
        outputCommand.tiltToBoard();
        while(timer.milliseconds() < 500);
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

    private void intakePixel() {
        intakeCommand.lowerIntake();
        intakeCommand.intakeIn(1);
        intakeCommand.intakeRollerIn();
        sleep(700);
        intakeCommand.raiseIntake();
        intakeCommand.stopIntake();
        intakeCommand.intakeRollerStop();
    }

    private void detectAprilTag() {
//        //goToAprilTag = true;
//        //sleep(1000);
//        //
//        //while(goToAprilTag && !isStopRequested()) {
//        //    if(aprilCamSubsystem.getHashmap().containsKey(aprilID)){
//        //        mecanumCommand.setFinalPosition(true, 30, getTargetX(-8.0), getTargetY(-5.0), getTargetTheta());
//        //    }
//        //    while(!mecanumCommand.isPositionReached(true, true)){}

    }
}