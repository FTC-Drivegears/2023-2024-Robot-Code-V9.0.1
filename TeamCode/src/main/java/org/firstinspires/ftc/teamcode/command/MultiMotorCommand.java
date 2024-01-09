package org.firstinspires.ftc.teamcode.command;

import org.firstinspires.ftc.teamcode.subsystems.MultiMotorSubsystem;
import org.firstinspires.ftc.teamcode.util.Interval;

public class MultiMotorCommand {
    private MultiMotorSubsystem multiMotorSubsystem;
    private int level = -1;

    public MultiMotorCommand(MultiMotorSubsystem multiMotorSubsystem){
        this.multiMotorSubsystem = multiMotorSubsystem;
    }

    public void LiftUp(boolean run, int level){
        this.level = level;
        Interval interval1;
        Interval interval2;
        Interval interval3;
        Interval interval4;


        switch(level){
            case 0:
                interval1 = new Interval(1000, 3400, -2000);
                interval2 = new Interval(500, 1000, -1500);
                interval3 = new Interval(18, 500, -500);
                interval4 = new Interval(-400, 18, 0);
                if(run) {
                    multiMotorSubsystem.LiftCascadeProcess(0, interval1, interval2, interval3, interval4);
                }
                break;
            case 1:
                interval1 = new Interval(-400, 300, 1000);
                interval2 = new Interval(300, 400, 400);
                interval3 = new Interval(250, 5000, 0);
                //TODO: deceleration intervals
                if(run) {
                    multiMotorSubsystem.LiftCascadeProcess(450, interval1, interval2, interval3);
                }
                break;
            case 2:
                interval1 = new Interval(-400, 900, 2000);
                interval2 = new Interval(900, 1075, 1700);
                interval3 = new Interval(1075, 1150, 900);
                interval4 = new Interval(1150, 5000, 0);
                if(run) {
                    multiMotorSubsystem.LiftCascadeProcess(1200, interval1, interval2, interval3, interval4);
                }
                break;
            case 3:
                interval1 = new Interval(-400, 1950, 2000);
                interval2 = new Interval(1950, 2100, 1700);
                interval3 = new Interval(2100, 2200, 1000);
                interval4 = new Interval(2200, 5000, 0);
                if(run) {
                    multiMotorSubsystem.LiftCascadeProcess(2317, interval1, interval2, interval3, interval4);
                }
                break;
            case 4:
                interval1 = new Interval(-400, 2300, 2000);
                interval2 = new Interval(2300, 2500, 1700);
                interval3 = new Interval(2500, 2650, 1000);
                interval4 = new Interval(2650, 5000, 0);
                if(run) {
                    multiMotorSubsystem.LiftCascadeProcess(2700, interval1, interval2, interval3, interval4);
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
                interval1 = new Interval(-400, 600, 2000);
                interval2 = new Interval(600, 700, 1700);
                interval3 = new Interval(700, 800, 800);
                interval4 = new Interval(800, 1350+2000, 0);
                multiMotorSubsystem.LiftCascadeProcess(820, interval1, interval2, interval3, interval4);
                break;
            default:
//                multiMotorSubsystem.moveLift(0);
                break;
        }
    }

    public int getLevel() {
        return level;
    }
}
