package org.firstinspires.ftc.teamcode.drive.opmode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ColorSensorSubsystem;

@TeleOp(name = "colour sensor reading")
public class ColourSubsystemTest extends LinearOpMode{
    private ColorSensorSubsystem colorSensorSubsystem;
    private String color1 ="";
    private String color2 = "";

    @Override
    public void runOpMode() {
        colorSensorSubsystem = new ColorSensorSubsystem(hardwareMap);

        waitForStart();
        while(opModeIsActive()) {
            color1 = colorSensorSubsystem.findColor1();
            color2 = colorSensorSubsystem.findColor2();

            //RBG
            //black
                //746, 319
                //1122, 410
                //1408, 534
            //purple (front)
                //1567, 2639, 2532
            //purple (back)
                //1340, 2207, 1900
            //yellow (back)
                //1705, 768, 2367
            //yellow (front)
                //1871, 1441, 2923
            //green (front)
                //894, 1286, 1980
            //green (back)
                //647, 745, 1650
            //white (front)
                //2120, 3011, 3772
            //white (back)
                //3300, 4260, 5480
            //

            telemetry.addData("color1", color1);
            telemetry.addData("color2", color2);
            telemetry.addData("red1", colorSensorSubsystem.getRed1());
            telemetry.addData("red2", colorSensorSubsystem.getRed2());
            telemetry.addData("blue1", colorSensorSubsystem.getBlue1());
            telemetry.addData("blue2", colorSensorSubsystem.getBlue2());
            telemetry.addData("green1", colorSensorSubsystem.getGreen1());
            telemetry.addData("green2", colorSensorSubsystem.getGreen2());
            telemetry.update();
        }
    }
}
