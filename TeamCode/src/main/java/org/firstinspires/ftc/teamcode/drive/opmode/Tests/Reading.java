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

    private DcMotor frontLeft, frontRight, backLeft, backRight;
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
        frontLeft = mecanumSubsystem.getLeftForward();
        frontRight = mecanumSubsystem.getRightForward();
        backLeft = mecanumSubsystem.getLeftBack();
        backRight = mecanumSubsystem.getRightBack();

        waitForStart();

        CompletableFuture.runAsync(this::runOdometry);

        while (opModeIsActive()) {
            if(gamepad1.a){
                frontLeft.setPower(1);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
            }
            else if(gamepad1.b){
                frontRight.setPower(1);
                frontLeft.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
            }
            else if(gamepad1.y){
                backLeft.setPower(1);
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backRight.setPower(0);
            }
            else if(gamepad1.x){
                backRight.setPower(1);
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
            }
            else{
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
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
            odo.process();
        }
    }
}