package org.firstinspires.ftc.teamcode.drive.opmode.Tests;

import android.os.Build;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.command.MecanumCommand;
import org.firstinspires.ftc.teamcode.subsystems.IMUSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;

import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.util.GyroOdometry;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

@Autonomous(name="coordinate testing")
public class CoordinateTesting extends LinearOpMode {
    private MecanumSubsystem mecanumSubsystem;
    private MecanumCommand mecanumCommand;
    private IMUSubsystem imu;
    private OdometrySubsystem odometrySubsystem;
    private GyroOdometry gyroOdometry;
    FtcDashboard dashboard;
    TelemetryPacket packet;

    @Override
    public void runOpMode() throws InterruptedException {
        imu = new IMUSubsystem(hardwareMap);
        mecanumSubsystem = new MecanumSubsystem(hardwareMap);
        odometrySubsystem = new OdometrySubsystem(hardwareMap);
        gyroOdometry = new GyroOdometry(odometrySubsystem, imu);
        mecanumCommand = new MecanumCommand(mecanumSubsystem, odometrySubsystem, gyroOdometry, this);

        mecanumCommand.turnOffInternalPID();
        imu.resetAngle();
        odometrySubsystem.reset();

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();

        waitForStart();
        odometrySubsystem.reset();

        Executor executor = Executors.newFixedThreadPool(5);
        CompletableFuture.runAsync(this::updateOdometry, executor);
        CompletableFuture.runAsync(this::updateTelemetry, executor);
        CompletableFuture.runAsync(this::pidProcess, executor);
        CompletableFuture.runAsync(this::motorProcess, executor);

//        sleep(8000);
//        mecanumCommand.moveRotation(Math.PI);
        while(opModeIsActive()) {
            mecanumCommand.setFinalPosition(true, 30, 40, 0, 0);
            sleep(4000);

        }
//        sleep(4000);
//        mecanumCommand.moveToGlobalPosition(100, 100, Math.PI);
//        sleep(4000);
//        mecanumCommand.moveToGlobalPosition(100, 100, 2*Math.PI);

    }

    public void updateOdometry() {
        while (opModeIsActive()) {
            imu.gyroProcess();
            gyroOdometry.process();
        }
    }
    public void pidProcess(){
        while (opModeIsActive()) {
            mecanumCommand.pidProcess();
        }
    }

    public void motorProcess(){
        while (opModeIsActive()) {
            mecanumSubsystem.motorProcess();
        }
    }

    public void updateTelemetry() {
        while (opModeIsActive()) {
            packet.put("x", gyroOdometry.x);
            packet.put("y", gyroOdometry.y);
            packet.put("output", mecanumCommand.globalXController.getOutputPositionalValue());
            packet.put("y output", mecanumCommand.globalYController.getOutputPositionalValue());
            packet.put("theta", gyroOdometry.theta);
            packet.put("theta output", mecanumCommand.globalThetaController.getOutputPositionalValue());
            packet.put("lfvel", mecanumSubsystem.lfvel);
            packet.put("lbvel", mecanumSubsystem.lbvel);
            packet.put("rfvel", mecanumSubsystem.rfvel);
            packet.put("rbvel", mecanumSubsystem.rbvel);
            telemetry.addData("x", gyroOdometry.x);
            telemetry.addData("y", gyroOdometry.y);
            telemetry.addData("xintegral", mecanumCommand.globalXController.getIntegralSum());
            telemetry.addData("output", mecanumCommand.globalXController.getOutputPositionalValue());
            telemetry.update();
            dashboard.sendTelemetryPacket(packet);
        }
    }
}