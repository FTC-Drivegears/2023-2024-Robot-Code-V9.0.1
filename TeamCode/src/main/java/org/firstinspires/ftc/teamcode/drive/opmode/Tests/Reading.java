package org.firstinspires.ftc.teamcode.drive.opmode.Tests;

import android.os.Build;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private GyroOdometry odo;
    private IMUSubsystem imu;
    private FtcDashboard dash;
    private TelemetryPacket packet;
    private OdometrySubsystem odometrySubsystem;
    private MecanumSubsystem mecanumSubsystem;

    @Override
    public void runOpMode() throws InterruptedException {
        dash = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
        imu = new IMUSubsystem(hardwareMap);
        mecanumSubsystem = new MecanumSubsystem(hardwareMap);
        odometrySubsystem = new OdometrySubsystem(hardwareMap);
        odo = new GyroOdometry(odometrySubsystem, imu);
        frontLeft = mecanumSubsystem.getLeftForward();
        frontRight = mecanumSubsystem.getRightForward();
        backLeft = mecanumSubsystem.getLeftBack();
        backRight = mecanumSubsystem.getRightBack();

        mecanumSubsystem.reset();

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
                backRight.setPower(1
                );
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
            }
            else if(gamepad1.dpad_up){
                backRight.setVelocity(1, AngleUnit.RADIANS);
                frontLeft.setVelocity(0, AngleUnit.RADIANS);
                frontRight.setVelocity(0, AngleUnit.RADIANS);
                backLeft.setVelocity(0, AngleUnit.RADIANS);
                telemetry.update();
            }
            else if(gamepad1.dpad_down){
                backRight.setVelocity(0, AngleUnit.RADIANS);
                frontLeft.setVelocity(1, AngleUnit.RADIANS);
                frontRight.setVelocity(0, AngleUnit.RADIANS);
                backLeft.setVelocity(0, AngleUnit.RADIANS);
            }
            else if(gamepad1.dpad_right){
                backRight.setVelocity(0, AngleUnit.RADIANS);
                frontLeft.setVelocity(0, AngleUnit.RADIANS);
                frontRight.setVelocity(1, AngleUnit.RADIANS);
                backLeft.setVelocity(0, AngleUnit.RADIANS);
            }
            else if(gamepad1.dpad_left){
                backRight.setVelocity(0, AngleUnit.RADIANS);
                frontLeft.setVelocity(0, AngleUnit.RADIANS);
                frontRight.setVelocity(0, AngleUnit.RADIANS);
                backLeft.setVelocity(1, AngleUnit.RADIANS);
            }
            else{
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                backRight.setVelocity(0, AngleUnit.RADIANS);
                frontLeft.setVelocity(0, AngleUnit.RADIANS);
                frontRight.setVelocity(0, AngleUnit.RADIANS);
                backLeft.setVelocity(0, AngleUnit.RADIANS);
            }
            telemetry.addData("leftFront", mecanumSubsystem.getLeftForward().getCurrentPosition());
            telemetry.addData("rightFront", mecanumSubsystem.getRightForward().getCurrentPosition());
            telemetry.addData("leftBack", mecanumSubsystem.getLeftBack().getCurrentPosition());
            telemetry.addData("rightBack", mecanumSubsystem.getRightBack().getCurrentPosition());
            telemetry.addData("x", odo.x);
            telemetry.addData("y", odo.y);
            telemetry.addData("heading", odo.theta);
            telemetry.addData("dtheta", odo.dTheta);
            telemetry.addData("dtheta2", imu.dTheta);
            telemetry.addData("imu heading", imu.getTheta());
            telemetry.addData("leftEncoder", odometrySubsystem.leftEncoder());
            telemetry.addData("rightEncoder", odometrySubsystem.rightEncoder());
            telemetry.addData("aux", odometrySubsystem.backEncoder());
            telemetry.addData("velocity test", mecanumSubsystem.getLeftForward().getVelocity(AngleUnit.RADIANS));
            packet.put("x", odo.x);
            packet.put("y", odo.y);
            packet.put("heading", odo.theta);
            packet.put("imu heading", imu.getTheta());
            telemetry.update();
            dash.sendTelemetryPacket(packet);
        }
    }
    public void runOdometry(){
        while(opModeIsActive()){
            imu.gyroProcess();
            odo.process();
        }
    }
}