package org.firstinspires.ftc.teamcode.util;

public class LogisticFunction {
    private double L;
    private double k;
    private double x0;
    private double b;

    public LogisticFunction(double L, double k, double x0, double b){
        this.L = L;
        this.k = k;
        this.x0 = x0;
        this.b = b;
    }

    public double getOutput(double x){
        return L/(1+Math.pow(Math.E, -k*(x-x0)))+b;
    }

    public double getDerivativeOutput(double x){
        return (L*k*Math.pow(Math.E, -k*(x-x0)))/Math.pow(1+Math.pow(Math.E, -k*(x-x0)), 2);
    }

    public double getL() {
        return L;
    }

    public double getK() {
        return k;
    }

    public double getX0() {
        return x0;
    }

    public double getB() {
        return b;
    }

    public void setL(double l) {
        L = l;
    }
}
