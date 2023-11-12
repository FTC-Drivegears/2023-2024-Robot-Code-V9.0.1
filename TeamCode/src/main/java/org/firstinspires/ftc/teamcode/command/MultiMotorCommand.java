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
                interval1 = new Interval(1000, 3400, 2700);
                interval2 = new Interval(500, 1000, 1500);
                interval3 = new Interval(50, 500, 800);
                interval4 = new Interval(-400, 50, 0);
                if(run) {
                    multiMotorSubsystem.LiftCascadeProcess(0, interval1, interval2, interval3, interval4);
                }
                break;
            case 1:
                interval1 = new Interval(-400, 200, -600);
                interval2 = new Interval(200, 2800, 0);
                if(run) {
                    multiMotorSubsystem.LiftCascadeProcess(247, interval1, interval2);
                }
                break;
            case 2:
                interval1 = new Interval(-400, 700, -2500);
                interval2 = new Interval(700, 1000, -2000);
                interval3 = new Interval(1000, 1250, -1000);
                interval4 = new Interval(1250, 1350+2000, 0);
                if(run) {
                    multiMotorSubsystem.LiftCascadeProcess(1324, interval1, interval2, interval3, interval4);
                }
                break;
            case 3:
                interval1 = new Interval(-400, 1950, -2500);
                interval2 = new Interval(1950, 2100, -2000);
                interval3 = new Interval(2100, 2200, -1000);
                interval4 = new Interval(2200, 3000, 0);
                if(run) {
                    multiMotorSubsystem.LiftCascadeProcess(2317, interval1, interval2, interval3, interval4);
                }
                break;
            case 4:
                interval1 = new Interval(-400, 2700, -2500);
                interval2 = new Interval(2700, 2800, -2000);
                interval3 = new Interval(2800, 3000, -1000);
                interval4 = new Interval(3000, 3400, 0);
                if(run) {
                    multiMotorSubsystem.LiftCascadeProcess(3100, interval1, interval2, interval3, interval4);
                }

//                Intervals for Position 4100
//                Interval interval1 = new Interval(0, 3700, -2500);
//                Interval interval2 = new Interval(3400, 3700, -2000);
//                Interval interval3 = new Interval(3700, 4000, -1000);
//                Interval interval4 = new Interval(4000, 4500, 0);
//                if(run) {
//                    multiMotorSubsystem.LiftCascadeProcess(4100, interval1, interval2, interval3, interval4);
//                }
                break;
        }
    }

    public int getLevel() {
        return level;
    }
}
