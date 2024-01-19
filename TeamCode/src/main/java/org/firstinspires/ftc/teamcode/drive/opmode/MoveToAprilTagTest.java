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
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.ArrayList;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;


@Autonomous(name="MoveToAprilTagTest")
public class MoveToAprilTagTest extends LinearOpMode {
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

    private boolean goToAprilTag = true;
    AprilCamSubsystem aprilCamSubsystem;
    private OpenCvCamera webcam;

    private ArrayList<AprilTagDetection> detections;


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
        imu.resetAngle();

        intakeCommand.raiseIntake();
        outputCommand.closeGate();

        outputCommand.armToIdle();
        outputCommand.tiltToIdle();
        String position = "right";
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
        double propPosition = 0;
        while (opModeInInit()) {
            propPosition = webcamSubsystem.getXProp();
        }

        waitForStart();

        Executor executor = Executors.newFixedThreadPool(7);
        CompletableFuture.runAsync(this::updateOdometry, executor);
        CompletableFuture.runAsync(this::updateTelemetry, executor);
        CompletableFuture.runAsync(this::pidProcess, executor);
        CompletableFuture.runAsync(this::detectAprilTags, executor);
        CompletableFuture.runAsync(this::motorProcess, executor);
        CompletableFuture.runAsync(this::liftProcess, executor);

        goToAprilTag = true;
        sleep(500);
        timer.reset();
        while(timer.milliseconds() < 30000){
        }
        //while (!mecanumCommand.isPositionReached(false, false)) {
        //}

    }

    public void pidProcess() {
        while (opModeIsActive()) {
            mecanumCommand.pidProcess();
        }
    }

    public void motorProcess() {
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
            packet.put("position reached", mecanumCommand.isPositionReached(false, false));
            telemetry.addData("x", gyroOdometry.x);
            telemetry.addData("y", gyroOdometry.y);
            telemetry.addData("theta", gyroOdometry.theta);



//            packet.put("x", gyroOdometry.x);
//            packet.put("y", gyroOdometry.y);

            telemetry.addData("goToAprilTag", goToAprilTag);

            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }
    }

    public void liftProcess() {
        while (opModeIsActive()) {
            multiMotorCommand.LiftUp(true, level);
        }
    }

    public void detectAprilTags() {
        /*
        while (opModeIsActive()) {
            telemetry.addData("goToAprilTag", goToAprilTag);
            if (goToAprilTag) {
                aprilCamSubsystem.runDetections();
                if (aprilCamSubsystem.getDetections().size() > 0) {
                    ArrayList<AprilTagDetection> detections = aprilCamSubsystem.getDetections();
                    telemetry.addData("Detections", detections);

                    //int closestnum = detections.get(0).id;
                /*
                for (int i = 0; i < detections.size(); i++) {

                }
                if(){
                    double targetX = gyroOdometry.x + detections.get(i).ftcPose.x;
                    double targetY =  gyroOdometry.y + detections.get(i).ftcPose.y;
                    double targetTheta = math.pi/2;
                    mecanumCommand.setFinalPosition(true,30 , targetX, targetY, targetTheta);
                    while(!mecanumCommand.isCoordinatePassed()) {};
                }
                else{

                }


                    int target = 4;

                    for (int i = 0; i < detections.size(); i++) {
                        telemetry.addData("x" + i, detections.get(i).ftcPose.x);
                        telemetry.addData("y" + i, detections.get(i).ftcPose.y);
                        telemetry.addData("z" + i, detections.get(i).ftcPose.z);
                        telemetry.addData("yaw" + i, detections.get(i).ftcPose.yaw);
                        telemetry.addData("pitch" + i, detections.get(i).ftcPose.pitch);
                        telemetry.addData("roll" + i, detections.get(i).ftcPose.roll);


                        //Note: probably in the wrong spot
                        if (detections.get(i).id == target) {
                            double targetX = gyroOdometry.x + detections.get(i).ftcPose.x;
                            double targetY = gyroOdometry.y + detections.get(i).ftcPose.y;
                            //double targetTheta = Math.PI / 2;
                            double targetTheta = 0;
                            //mecanumCommand.setFinalPosition(true, 30, targetX, targetY, targetTheta);
                        }


                    }
                }

            }

        }

    */


        while (opModeIsActive()) {

            if (goToAprilTag) {

                aprilCamSubsystem.runDetections();

                telemetry.addData("Hello", "hello");

                if (aprilCamSubsystem.getDetections().size() > 0) {
                    detections = aprilCamSubsystem.getDetections();

                    int target = 4;

                    double targetX = gyroOdometry.x + aprilCamSubsystem.getIdValues(target).ftcPose.x;
                    double targetY = gyroOdometry.y + aprilCamSubsystem.getIdValues(target).ftcPose.y;
                    //double targetTheta = Math.PI / 2;
                    double targetTheta = 0;
                    //mecanumCommand.setFinalPosition(true, 30, targetX, targetY, targetTheta);
                }
                telemetry.addData("Detections", detections);

                for (int i = 0; i < detections.size(); i++) {
                    telemetry.addData("x" + i, detections.get(i).ftcPose.x);
                    telemetry.addData("y" + i, detections.get(i).ftcPose.y);
                    telemetry.addData("z" + i, detections.get(i).ftcPose.z);
                    telemetry.addData("yaw" + i, detections.get(i).ftcPose.yaw);
                    telemetry.addData("pitch" + i, detections.get(i).ftcPose.pitch);
                    telemetry.addData("roll" + i, detections.get(i).ftcPose.roll);
                }

            }

        }
    }

}


