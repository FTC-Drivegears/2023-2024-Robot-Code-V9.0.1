package org.firstinspires.ftc.teamcode.drive.opmode.Tests;

import android.os.Build;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.IMUSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.util.GyroOdometry;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;

@TeleOp(name="Reading")
public class Reading extends LinearOpMode {

//    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private GyroOdometry odo;
    private IMUSubsystem imu;
    private FtcDashboard dash;
    private TelemetryPacket packet;
    private OdometrySubsystem odometrySubsystem;
    private MecanumSubsystem mecanumSubsystem;

    @Override
    public void runOpMode() throws InterruptedException {
        imu = new IMUSubsystem(hardwareMap);
        odometrySubsystem = new OdometrySubsystem(hardwareMap);
        odo = new GyroOdometry(odometrySubsystem, imu);
        mecanumSubsystem = new MecanumSubsystem(hardwareMap);

//        frontLeft = hardwareMap.get(DcMotor.class, "leftForward");
//        frontRight = hardwareMap.get(DcMotor.class, "rightForward");
//        backLeft = hardwareMap.get(DcMotor.class, "leftBack");
//        backRight = hardwareMap.get(DcMotor.class, "rightBack");
//
//        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
//        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
//        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu.resetAngle();
        dash = FtcDashboard.getInstance();
        packet = new TelemetryPacket();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry()); // allows telemetry to output to phone and dashboard

//        odometrySubsystem.reset();

        waitForStart();

        CompletableFuture.runAsync(this::runOdometry);

        while (opModeIsActive()) {
            if(gamepad1.a){
                mecanumSubsystem.getLeftForward().setPower(1);
                mecanumSubsystem.getLeftBack().setPower(0);
                mecanumSubsystem.getRightForward().setPower(0);
                mecanumSubsystem.getRightBack().setPower(0);
            }
            else if(gamepad1.b){
                mecanumSubsystem.getLeftForward().setPower(0);
                mecanumSubsystem.getLeftBack().setPower(1);
                mecanumSubsystem.getRightForward().setPower(0);
                mecanumSubsystem.getRightBack().setPower(0);
            }
            else if(gamepad1.y){
                mecanumSubsystem.getLeftForward().setPower(0);
                mecanumSubsystem.getLeftBack().setPower(0);
                mecanumSubsystem.getRightForward().setPower(1);
                mecanumSubsystem.getRightBack().setPower(0);
            }
            else if(gamepad1.x){
                mecanumSubsystem.getLeftForward().setPower(0);
                mecanumSubsystem.getLeftBack().setPower(0);
                mecanumSubsystem.getRightForward().setPower(0);
                mecanumSubsystem.getRightBack().setPower(1);
            }
            else{
                mecanumSubsystem.getLeftForward().setPower(0);
                mecanumSubsystem.getLeftBack().setPower(0);
                mecanumSubsystem.getRightForward().setPower(0);
                mecanumSubsystem.getRightBack().setPower(0);
            }
            telemetry.addData("x", odo.x);
            telemetry.addData("y", odo.y);
            telemetry.addData("heading", odo.theta);
            telemetry.addData("imu heading", imu.getTheta());
            telemetry.addData("leftEncoder", odometrySubsystem.leftEncoder());
            telemetry.addData("rightEncoder", odometrySubsystem.rightEncoder());
            telemetry.addData("aux", odometrySubsystem.backEncoder());
            telemetry.update();
        }
    }
    public void runOdometry(){
        while(opModeIsActive()){
            odo.odometryProcess();
        }
    }
}