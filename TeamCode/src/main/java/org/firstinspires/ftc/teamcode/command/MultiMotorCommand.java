package org.firstinspires.ftc.teamcode.command;

import org.firstinspires.ftc.teamcode.subsystems.MultiMotorSubsystem;
import org.firstinspires.ftc.teamcode.util.Interval;

public class MultiMotorCommand {
    private MultiMotorSubsystem multiMotorSubsystem;
    private int level = -1;
    private int targetPosition = 0;

    public MultiMotorCommand(MultiMotorSubsystem multiMotorSubsystem){
        this.multiMotorSubsystem = multiMotorSubsystem;
    }

    public void LiftUpPositional(boolean run, int level){
        switch(level){
            case 0:
                if(run) {
                    multiMotorSubsystem.LiftPositionalProcess(0);
                    targetPosition = 0;
                }
                break;
            case 1:
                //TODO: deceleration intervals
                if(run) {
                    multiMotorSubsystem.LiftPositionalProcess(450);
                    targetPosition = 450;
                }
                break;
            case 2:
                if(run) {
                    multiMotorSubsystem.LiftPositionalProcess(1200);
                }
                break;
            case 3:
                if(run) {
                    multiMotorSubsystem.LiftPositionalProcess(1800);
                    targetPosition = 1800;
                }
                break;
            case 4:
                if(run) {
                    multiMotorSubsystem.LiftPositionalProcess(2200);
                    targetPosition = 2200;
                }
                break;
            case 5:
                multiMotorSubsystem.LiftPositionalProcess(950);
                targetPosition = 900;
                break;
            default:
//                multiMotorSubsystem.moveLift(0);
                break;
        }
    }

    public void LiftUp(boolean run, int level){
        this.level = level;
        Interval interval1;
        Interval interval2;
        Interval interval3;
        Interval interval4;
        Interval interval5;

        switch(level){
            case 0:
                interval1 = new Interval(1000, 3400, -2000);
                interval2 = new Interval(500, 1000, -700);
                interval3 = new Interval(90, 500, -100);
                interval4 = new Interval(5, 90, -50);
                interval5 = new Interval(-400, 5, 0);
                if(run) {
                    multiMotorSubsystem.LiftCascadeProcess(0, interval1, interval2, interval3, interval4, interval5);
                }
                break;
            case 1:
                interval1 = new Interval(-400, 300, 1000);
                interval2 = new Interval(300, 440, 400);
                interval3 = new Interval(440, 5000, 0);
                //TODO: deceleration intervals
                if(run) {
                    multiMotorSubsystem.LiftCascadeProcess(450, interval1, interval2, interval3);
                }
                break;
            case 2:
                interval1 = new Interval(-400, 850, 2000);
                interval2 = new Interval(850, 1075, 1700);
                interval3 = new Interval(1075, 1190, 900);
                interval4 = new Interval(1190, 5000, 0);
                if(run) {
                    multiMotorSubsystem.LiftCascadeProcess(1200, interval1, interval2, interval3, interval4);
                }
                break;
            case 3:
                interval1 = new Interval(-400, 1500, 2000);
                interval2 = new Interval(1500, 1700, 1700);
                interval3 = new Interval(1700, 1780, 1000);
                interval4 = new Interval(1780, 5000, 0);
                if(run) {
                    multiMotorSubsystem.LiftCascadeProcess(1800, interval1, interval2, interval3, interval4);
                }
                break;
            case 4:
                interval1 = new Interval(-400, 1750, 2000);
                interval2 = new Interval(1750, 2000, 1700);
                interval3 = new Interval(2000, 2150, 1000);
                interval4 = new Interval(2150, 5000, 0);
                if(run) {
                    multiMotorSubsystem.LiftCascadeProcess(2200, interval1, interval2, interval3, interval4);
                }
                break;

//                Intervals for Position 4100
//                Interval interval1 = new Interval(0, 3700, -2500);
//                Interval interval2 = new Interval(3400, 3700, -2000);
//                Interval interval3 = new Interval(3700, 4000, -1000);
//                Interval interval4 = new Interval(4000, 4500, 0);
//                if(run) {
//                    multiMotorSubsystem.LiftCascadeProcess(4100, interval1, interval2, interval3, interval4);
//                }
            case 5:
                // Thompson did this :p
                //good job thompson :thumbsup:
                multiMotorSubsystem.LiftCascadeProcess(
                        820,
                        new Interval(-400, 700, 2000),
                        new Interval(700, 750, 1700),
                        new Interval(750, 800, 1000),
                        new Interval(800, 1350+2000, 0)
                );
                break;
            default:
//                multiMotorSubsystem.moveLift(0);
                break;
        }
    }

    public int getLevel() {
        return level;
    }
    public double getTargetPos(){
        return targetPosition;
    }
}
