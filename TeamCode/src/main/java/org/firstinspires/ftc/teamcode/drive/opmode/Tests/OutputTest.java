package org.firstinspires.ftc.teamcode.drive.opmode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.command.IntakeCommand;
import org.firstinspires.ftc.teamcode.command.OutputCommand;

@Config
@TeleOp(name="Output Test")
public class  OutputTest extends LinearOpMode {
    private OutputCommand outputCommand;
    private IntakeCommand intakeCommand;

    public static double position = 0.5;
    @Override
    public void runOpMode() throws InterruptedException {
        outputCommand = new OutputCommand(hardwareMap);
        intakeCommand = new IntakeCommand(hardwareMap);
        outputCommand.initialize();
        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.a){
                outputCommand.openGate();
            }
            else if(gamepad1.b){
                outputCommand.closeGate();
            }
            else if(gamepad1.x){
                outputCommand.closeGate();

            }
            else if(gamepad1.y){
                outputCommand.openGate();
            }
            else if(gamepad1.dpad_up){
                outputCommand.tiltToIdle();
            }
            else if(gamepad1.dpad_down){
                outputCommand.tiltToBoard();
            }
            else if(gamepad1.right_trigger > 0.5){
                outputCommand.outputWheelOut();

            }
            else if(gamepad1.dpad_right){
                outputCommand.armToIdle();
            }
            else if(gamepad1.dpad_left){
                outputCommand.armToBoard();
            }
            else{
                outputCommand.outputWheelStop();

            }

            telemetry.addData("intake", intakeCommand.intakeValue());
//            telemetry.addData("gate position", outputCommand.getGatePosition());
            telemetry.addData("left arm position", outputCommand.getLeftArmPosition());
            telemetry.addData("right arm position", outputCommand.getRightArmPosition());
            telemetry.addData("gate position", outputCommand.getGatePosition());
            telemetry.addData("left tilt position", outputCommand.getLeftTiltPosition());
//            telemetry.addData("right tilt position", outputCommand.getRightTiltPosition());
            telemetry.update();
        }
    }
}
