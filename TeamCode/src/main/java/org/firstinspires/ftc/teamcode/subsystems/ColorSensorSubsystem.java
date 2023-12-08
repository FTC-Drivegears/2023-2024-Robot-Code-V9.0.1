package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Specifications;

public class ColorSensorSubsystem {

    private RevBlinkinLedDriver Light;
    private RevColorSensorV3 colorSensor1;
    private RevColorSensorV3 colorSensor2;
    public ColorSensorSubsystem(HardwareMap hardwareMap) {
        Light = hardwareMap.get(RevBlinkinLedDriver.class, "led");
        colorSensor1 = hardwareMap.get(RevColorSensorV3.class, Specifications.FIRST_COLOR_SENSOR);
        colorSensor2 = hardwareMap.get(RevColorSensorV3.class, Specifications.SECOND_COLOR_SENSOR);
    }

    public void setColor(String color){
        switch (color) {
            case "Lilac Purple":
                setPatternLilacPurple();
                break;
            case "Yellow":
                setPatternYellow();
                break;
            case "Green":
                setPatternGreen();
                break;
            case "White":
                setPatternWhite();
                break;
            case "none":
                setPatternNothing();
                break;
        }
    }


    // Set pattern methods
    public void setPatternGreen() {
        Light.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
    }

    public void setPatternLilacPurple() {
        Light.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
    }

    public int getRed1(){ return colorSensor1.red();}
    public int getGreen1(){ return colorSensor1.green();}
    public int getBlue1(){ return colorSensor1.blue();}

    public int getRed2(){ return colorSensor2.red();}
    public int getGreen2(){ return colorSensor2.green();}
    public int getBlue2(){ return colorSensor2.blue();}
    public void setPatternRed() {
        Light.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
    }

    public void setPatternYellow() {
        Light.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED_ORANGE);
    }

    public void setPatternNothing() {
        Light.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }
    public void setPatternWhite(){
        Light.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
    }

    //ranges front --> purple, yellow, green, white, black
    //1567, 1871, 894, 2120, 746 // R
    //2639, 1441, 1286, 3011, 1122 // B
    //2532, 2923, 1980, 3772, 1408 // G

    public String findColor1(){
        double red = getRed1();
        double blue = getBlue1();
        double green = getGreen1();

        if((red + blue + green) > 8000){
            return "White";
        }
        else if((red + blue + green) < 3500){
            return "none";
        }
        else if(red < blue && red < green && (red + blue + green) < 5000){
            return "Green";
        }
        else if(green > blue && green > red && green - blue > 700){
            return "Yellow";
        }
        else{
            return "Lilac Purple";
        }
    }
    public String findColor2(){
        double red = getRed2();
        double blue = getBlue2();
        double green = getGreen2();

        if((red + blue + green) > 8000){
            return "White";
        }
        else if((red + blue + green) < 2000){
            return "none";
        }
        else if(red < blue && red < green && (red + blue + green) < 4000){
            return "Green";
        }
        else if(green > blue && green > red && green - blue > 1000){
            return "Yellow";
        }
        else{
            return "Lilac Purple";
        }
    }

    //ranges back --> purple, yellow, green, white, black
    //1340, 1705, 647, 3300, 319
    //2207, 768, 745, 4260, 410
    //1900, 2367, 1650, 5480, 534
}
