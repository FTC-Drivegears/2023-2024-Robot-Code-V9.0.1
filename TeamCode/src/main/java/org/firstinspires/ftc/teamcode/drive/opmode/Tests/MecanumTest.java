package org.firstinspires.ftc.teamcode.drive.opmode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.command.MecanumCommand;
import org.firstinspires.ftc.teamcode.subsystems.IMUSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.util.GyroOdometry;

import java.util.concurrent.CompletableFuture;

@TeleOp(name="drive test")
public class MecanumTest extends LinearOpMode{
    private MecanumSubsystem mecanumSubsystem;
    private IMUSubsystem imu;
    private OdometrySubsystem odo;
    private GyroOdometry gyroOdometry;
    private MecanumCommand mecanumCommand;
    @Override
    public void runOpMode() throws InterruptedException {

        imu = new IMUSubsystem(hardwareMap);
        odo = new OdometrySubsystem(hardwareMap);
        gyroOdometry = new GyroOdometry(odo, imu);
        mecanumSubsystem = new MecanumSubsystem(hardwareMap);
        mecanumCommand = new MecanumCommand(mecanumSubsystem, odo, gyroOdometry, this);

        imu.resetAngle();
        mecanumSubsystem.reset();

        waitForStart();

        CompletableFuture.runAsync(this::runOdometry);
        CompletableFuture.runAsync(this::runBase);

        while (opModeIsActive()) {
            //drive.move(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
            mecanumCommand.moveGlobalPartial(true, gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

//            telemetry.addData("Heading in DEG", imu.getHeadingDEG());
            telemetry.addData("leftFront", mecanumSubsystem.getLeftForward().getCurrentPosition());
            telemetry.addData("rightFront", mecanumSubsystem.getRightForward().getCurrentPosition());
            telemetry.addData("leftBack", mecanumSubsystem.getLeftBack().getCurrentPosition());
            telemetry.addData("rightBack", mecanumSubsystem.getRightBack().getCurrentPosition());
//            telemetry.addData("leftFrontCurrent", mecanumSubsystem.getLeftForward().getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("rightFrontCurrent", mecanumSubsystem.getRightForward().getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("leftBackCurrent", mecanumSubsystem.getLeftBack().getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("rightBackCurrent", mecanumSubsystem.getRightBack().getCurrent(CurrentUnit.AMPS));
            telemetry.addData("gamepad1.left_stick_y", gamepad1.left_stick_y);
            telemetry.addData("gamepad1.left_stick_x", gamepad1.left_stick_x);
            telemetry.addData("gamepad1.right_stick_x", gamepad1.right_stick_x);
            telemetry.addData("leftFrontPower", mecanumSubsystem.getLeftForward().getPower());
            telemetry.addData("rightFrontPower", mecanumSubsystem.getRightForward().getPower());
            telemetry.addData("leftBackPower", mecanumSubsystem.getLeftBack().getPower());
            telemetry.addData("rightBackPower", mecanumSubsystem.getRightBack().getPower());
            telemetry.addData("lfVel", mecanumSubsystem.lfvel);
            telemetry.addData("rfVel", mecanumSubsystem.rfvel);
            telemetry.addData("lbvel", mecanumSubsystem.lbvel);
            telemetry.addData("rbvel", mecanumSubsystem.rbvel);
            telemetry.addData("x", gyroOdometry.x);
            telemetry.addData("y", gyroOdometry.y);
//            telemetry.addData("theta", imu.Theta);
//            telemetry.addData("theta2", imu.getTheta());
            telemetry.addData("gyrotheta", gyroOdometry.theta);
            telemetry.update();
        }
    }
    public void runOdometry(){
        while(opModeIsActive()){
            imu.gyroProcess();
            gyroOdometry.process();
        }
    }

    public void runBase(){
        while(opModeIsActive()){
            mecanumSubsystem.motorProcessTeleOp();
        }
    }
}
