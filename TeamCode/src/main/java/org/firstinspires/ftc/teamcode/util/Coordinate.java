package org.firstinspires.ftc.teamcode.util;

public class Coordinate {
    private double x;
    private double y;
    private double theta;

    public Coordinate(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    public double getX() {
        return x;
    }
    public double getY() {
        return y;
    }
    public double getTheta() {
        return theta;
    }
}
