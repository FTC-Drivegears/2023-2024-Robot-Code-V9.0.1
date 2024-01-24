package org.firstinspires.ftc.teamcode.command;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.AprilCamSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.util.GyroOdometry;
import org.firstinspires.ftc.teamcode.util.LogisticFunction;
import org.firstinspires.ftc.teamcode.util.PIDCore;
import org.firstinspires.ftc.teamcode.util.Specifications;

@Config
public class MecanumCommand {
    private MecanumSubsystem mecanumSubsystem;
    private OdometrySubsystem odometrySubsystem;
    private GyroOdometry gyroOdometry;
    private AprilCamSubsystem aprilCamSubsystem;
    private boolean run;
    private LinearOpMode opMode;
    private ElapsedTime elapsedTime;
    public PIDCore globalXController;
    public PIDCore globalYController;
    public PIDCore globalThetaController;
    public PIDCore globalCascadeXController;
    public PIDCore globalCascadeYController;
    public PIDCore globalCascadeThetaController;
    private static double cascadekpx = 0.045;
    private static double cascadekdx = 0.0005;
    private static double cascadekix = 0.0075/2;
    private static double cascadekpy = 0.045;
    private static double cascadekdy = 0.0005;
    private static double cascadekiy = 0.004/2;
    private static double cascadekptheta = 2;
    private static double cascadekdtheta = 0.05;
    private static double cascadekitheta = 0.0075/2;
    //TODO: tune these
    private static double cascadekpxVel = 0.07;
    private static double cascadekdxVel = 0.01;
    private static double cascadekixVel = 0.004/2;
    private static double cascadekpyVel = 0.055;
    private static double cascadekdyVel = 0.0005;
    private static double cascadekiyVel = 0.0075/2;
    private static double cascadekpthetaVel = 2;
    private static double cascadekdthetaVel = 0.05;
    private static double cascadekithetaVel = 0.0;


    //TODO: can still tune teeny tiny bit
    private static double kpx = 0.045;
    private static double kdx = 0.00;
    private static double kix = 0.06;
    private static double kpy = 0.045;
    private static double kdy = 0.00;
    private static double kiy = 0.06;
    private static double kptheta = 2.5;
    private static double kdtheta = 0;
    private static double kitheta = 0.4;
    private double ex = 0;
    private double ey = 0;
    private double etheta = 0;
    private double xFinal;
    private double yFinal;
    private double thetaFinal;
    private double velocity;

    public void setthetaConstants(double kptheta, double kdtheta, double kitheta){
        MecanumCommand.kptheta = kptheta;
        MecanumCommand.kdtheta = kdtheta;
        MecanumCommand.kitheta = kitheta;
        globalThetaController.setConstant(kptheta,kdtheta,kitheta);
    }
    public void setConstants(double kpx, double kdx, double kix, double kpy, double kdy, double kiy, double kptheta, double kdtheta, double kitheta){
        MecanumCommand.kpx = kpx;
        MecanumCommand.kdx = kdx;
        MecanumCommand.kix = kix;
        MecanumCommand.kpy = kpy;
        MecanumCommand.kdy = kdy;
        MecanumCommand.kiy = kiy;
        MecanumCommand.kptheta = kptheta;
        MecanumCommand.kdtheta = kdtheta;
        MecanumCommand.kitheta = kitheta;
        globalXController.setConstant(kpx, kdx, kix);
        globalYController.setConstant(kpy, kdy, kiy);
        globalThetaController.setConstant(kptheta, kdtheta, kitheta);
    }

    public void addToConstants(double kpx, double kdx, double kix, double kpy, double kdy, double kiy, double kptheta, double kdtheta, double kitheta){
        MecanumCommand.kpx += kpx;
        MecanumCommand.kdx += kdx;
        MecanumCommand.kix += kix;
        MecanumCommand.kpy += kpy;
        MecanumCommand.kdy += kdy;
        MecanumCommand.kiy += kiy;
        MecanumCommand.kptheta += kptheta;
        MecanumCommand.kdtheta += kdtheta;
        MecanumCommand.kitheta += kitheta;
        globalXController.setConstant(kpx, kdx, kix);
        globalYController.setConstant(kpy, kdy, kiy);
        globalThetaController.setConstant(kptheta, kdtheta, kitheta);
    }

    public MecanumCommand(MecanumSubsystem mecanumSubsystem, OdometrySubsystem odometrySubsystem, GyroOdometry gyroOdometry, LinearOpMode opmode) {
        this.mecanumSubsystem = mecanumSubsystem;
        this.odometrySubsystem = odometrySubsystem;;
        this.gyroOdometry = gyroOdometry;
        this.opMode = opmode;
        globalXController = new PIDCore(kpx, kdx, kix);
        globalYController = new PIDCore(kpy, kdy, kiy);
        globalThetaController = new PIDCore(kptheta, kdtheta, kitheta);
        globalCascadeXController = new PIDCore(cascadekpx, cascadekdx, cascadekix);
        globalCascadeYController = new PIDCore(cascadekpy, cascadekdy, cascadekiy);
        globalCascadeThetaController = new PIDCore(cascadekptheta, cascadekdtheta, cascadekitheta);
        elapsedTime = new ElapsedTime();
        xFinal = gyroOdometry.x;
        yFinal = gyroOdometry.y;
        thetaFinal = gyroOdometry.theta;
        velocity = 0;
    }

    public MecanumCommand(MecanumSubsystem mecanumSubsystem, OdometrySubsystem odometrySubsystem, GyroOdometry gyroOdometry, AprilCamSubsystem aprilCamSubsystem, LinearOpMode opmode) {
        this.mecanumSubsystem = mecanumSubsystem;
        this.odometrySubsystem = odometrySubsystem;;
        this.gyroOdometry = gyroOdometry;
        this.aprilCamSubsystem = aprilCamSubsystem;
        this.opMode = opmode;
        globalXController = new PIDCore(kpx, kdx, kix);
        globalYController = new PIDCore(kpy, kdy, kiy);
        globalThetaController = new PIDCore(kptheta, kdtheta, kitheta);
        globalCascadeXController = new PIDCore(cascadekpx, cascadekdx, cascadekix);
        globalCascadeYController = new PIDCore(cascadekpy, cascadekdy, cascadekiy);
        globalCascadeThetaController = new PIDCore(cascadekptheta, cascadekdtheta, cascadekitheta);
        elapsedTime = new ElapsedTime();
        xFinal = gyroOdometry.x;
        yFinal = gyroOdometry.y;
        thetaFinal = gyroOdometry.theta;
        velocity = 0;
    }
    public void turnOffInternalPID(){
        mecanumSubsystem.turnOffInternalPID();
    }

    public void pidProcess(){
        ex = globalXController.outputPositionalIntegralControl(xFinal, gyroOdometry.x);
        ey = -globalYController.outputPositionalIntegralControl(yFinal, gyroOdometry.y);
        etheta = -globalThetaController.outputPositionalIntegralControl(thetaFinal, gyroOdometry.theta);

        if(isXReached()){
            globalXController.integralReset();
        }
        if(isYReached()){
            globalYController.integralReset();
        }
        if(isThetaReached()){
            globalThetaController.integralReset();
        }

        if(isXPassed()){
            globalXController.activateIntegral();
        }
        else{
            globalXController.deactivateIntegral();
        }
        if(isYPassed()){
            globalYController.activateIntegral();
        }
        else{
            globalYController.deactivateIntegral();
        }
        if(isThetaPassed()){
            globalThetaController.activateIntegral();
        }
        else{
            globalThetaController.deactivateIntegral();
        }
        if (Math.abs(ex) > velocity || Math.abs(ey) > velocity){
            double max = Math.max(Math.abs(ex), Math.abs(ey));
            ex = ex / max * velocity;
            ey = ey / max * velocity;
            etheta = etheta / max * velocity;
        }
        moveGlobalPartial(true, ex, ey, etheta);
    }



    public void pidProcessLogistic(){
        //TODO: make sure xFinal works as a valid L, might need to overshoot a bit because of nature of PID Controllers

        LogisticFunction logisticFunctionX = new LogisticFunction(xFinal, 0.1, gyroOdometry.x, 0);
        LogisticFunction logisticFunctionY = new LogisticFunction(yFinal, 0.1, gyroOdometry.y, 0);
        LogisticFunction logisticFunctionTheta = new LogisticFunction(thetaFinal, 0.1, gyroOdometry.theta, 0);

        ex = globalCascadeXController.cascadeOutput(xFinal, gyroOdometry.x, logisticFunctionX.getDerivativeOutput(gyroOdometry.x), globalCascadeXController.getDerivative());
        ey = globalCascadeYController.cascadeOutput(yFinal, gyroOdometry.y, logisticFunctionY.getDerivativeOutput(gyroOdometry.y), globalCascadeYController.getDerivative());
        etheta = globalCascadeThetaController.cascadeOutput(thetaFinal, gyroOdometry.theta, logisticFunctionTheta.getDerivativeOutput(gyroOdometry.theta), globalCascadeThetaController.getDerivative());
        if (Math.abs(ex) > velocity || Math.abs(ey) > velocity){
            double max = Math.max(Math.abs(ex), Math.abs(ey));
            ex = ex / max * velocity;
            ey = ey / max * velocity;
            etheta = etheta / max * velocity;
        }
        moveGlobalPartial(true, ex, ey, etheta);
    }

    public void move(boolean run, double lv, double lh, double rv, double rh) {
        mecanumSubsystem.move(run, lv, lh, rv, rh);
    }

    public void moveGlobalPartial(boolean run, double vertical, double horizontal, double rotational){
        if (run){
            double angle = Math.PI/2 - gyroOdometry.theta;
            double localVertical = vertical*Math.cos(gyroOdometry.theta) - horizontal*Math.cos(angle);
            double localHorizontal = vertical*Math.sin(gyroOdometry.theta) + horizontal*Math.sin(angle);
            mecanumSubsystem.partialMove(true, localVertical, localHorizontal, rotational);
        }
    }

    // this method is designed to be used in an autonomous (linear) opmode

    public void movePartial(boolean run, double vertical, double horizontal, double rotational){
        if (run){
            vertical *= Specifications.MAX_ANGULAR_VEL;
            horizontal *= Specifications.MAX_ANGULAR_VEL;
            rotational *= Specifications.MAX_ANGULAR_VEL;
            mecanumSubsystem.partialMove(true, vertical, horizontal, rotational);
        }
    }

    public void localToCoordMixed(boolean run, double xf, double yf, double power){
        double xi = odometrySubsystem.rightEncoder();
        double yi = odometrySubsystem.backEncoder();
        double x = 0;
        double y = 0;
        while (run && (Math.abs(xf - x) > 0.1 || Math.abs(yf - y) > 0.1)){
            x = (odometrySubsystem.rightEncoder() - xi) / odometrySubsystem.odometryTick * odometrySubsystem.odometryCir;
            y = (odometrySubsystem.backEncoder() - yi) / odometrySubsystem.odometryTick * odometrySubsystem.odometryCir;
            mecanumSubsystem.partialMove(true, Math.copySign(power, xf - x), Math.copySign(power, (yf - y)), 0);
        }
        mecanumSubsystem.partialMove(true, 0, 0, 0);
    }

    public void turnToAngle(double pow, double angle) {

//        double xTemp = mecanumSubsystem.x;
//        double yTemp = mecanumSubsystem.y;

        if (angle < 0) {
            angle += 360;
        }

        double angleTemp = angle - odometrySubsystem.convertBearing(odometrySubsystem.theta);
        if (angleTemp < 0) {
            angleTemp += 360;
        }

        if (angleTemp < 180) {
            while (Math.abs(odometrySubsystem.convertBearing(odometrySubsystem.theta) - angle) > 3 && opMode.opModeIsActive()) {
                mecanumSubsystem.move(true, 0 , 0, pow);
            }
        } else {
            while (Math.abs(odometrySubsystem.convertBearing(odometrySubsystem.theta) - angle) > 3 && opMode.opModeIsActive()) {
                mecanumSubsystem.move(true, 0, 0, -pow);
            }
        }
//        while ( Math.abs(mecanumSubsystem.Theta - angle) > 3 ) {
//            mecanumSubsystem.move(true, 0, 0, directionalPow);
//        }
        mecanumSubsystem.stop(true);
//        mecanumSubsystem.x = xTemp;
//        mecanumSubsystem.y = yTemp;

    }

    public void turnToAngleMixed(double pow, double angle) {

//        double xTemp = mecanumSubsystem.x;
//        double yTemp = mecanumSubsystem.y;

        if (angle < 0) {
            angle += 360;
        }

        double angleTemp = angle - odometrySubsystem.convertBearing(gyroOdometry.theta);
        if (angleTemp < 0) {
            angleTemp += 360;
        }

        if (angleTemp < 180) {
            while (Math.abs(odometrySubsystem.convertBearing(gyroOdometry.theta) - angle) > 3 && opMode.opModeIsActive()) {
                mecanumSubsystem.move(true, 0 , 0, pow);
            }
        } else {
            while (Math.abs(odometrySubsystem.convertBearing(gyroOdometry.theta) - angle) > 3 && opMode.opModeIsActive()) {
                mecanumSubsystem.move(true, 0, 0, -pow);
            }
        }
//        while ( Math.abs(mecanumSubsystem.Theta - angle) > 3 ) {
//            mecanumSubsystem.move(true, 0, 0, directionalPow);
//        }
        mecanumSubsystem.stop(true);
//        mecanumSubsystem.x = xTemp;
//        mecanumSubsystem.y = yTemp;

    }

    public void turnToZeroMixed(double pow) {

//        double xTemp = odometrySubsystem.x;
//        double yTemp = odometrySubsystem.y;

        if(odometrySubsystem.convertBearing(gyroOdometry.theta) < 0) {
            while ((Math.abs(odometrySubsystem.convertBearing(gyroOdometry.theta)) > 5) && opMode.opModeIsActive()) {
                mecanumSubsystem.move(true, 0, 0, -pow);
            }
        } else {
            while ((Math.abs(odometrySubsystem.convertBearing(gyroOdometry.theta)) > 5) && opMode.opModeIsActive()) {
                mecanumSubsystem.move(true, 0, 0, pow);
            }
        }
        mecanumSubsystem.stop(true);
//        mecanumSubsystem.x = xTemp;
//        mecanumSubsystem.y = yTemp;
    }

    public void turnToZero(double pow) {

//        double xTemp = mecanumSubsystem.x;
//        double yTemp = mecanumSubsystem.y;

        if (odometrySubsystem.convertBearing(odometrySubsystem.theta) < 0) {
            while ((Math.abs(odometrySubsystem.convertBearing(odometrySubsystem.theta)) > 5) && opMode.opModeIsActive()) {
                mecanumSubsystem.move(true, 0, 0, -pow);
            }
        } else {
            while ((Math.abs(odometrySubsystem.convertBearing(odometrySubsystem.theta)) > 5) && opMode.opModeIsActive()) {
                mecanumSubsystem.move(true, 0, 0, pow);
            }
        }
        mecanumSubsystem.stop(true);
//        mecanumSubsystem.x = xTemp;
//        mecanumSubsystem.y = yTemp;
    }

    public void moveToGlobalPosition(double targetX, double targetY, double targetTheta) {
        // stop moving if within 5 ticks or 0.2 radians from the position
        while (Math.abs(targetX - gyroOdometry.x) > 6
                || Math.abs(targetY - gyroOdometry.y) > 6
                || Math.abs(targetTheta - gyroOdometry.theta) > 0.2) {

            mecanumSubsystem.fieldOrientedMove(
                    globalYController.outputPositionalCapped(targetY, gyroOdometry.y, 0),
                   -globalXController.outputPositionalCapped(targetX, gyroOdometry.x, 0),
                    globalThetaController.outputPositionalCapped(targetTheta, gyroOdometry.theta, 0),
                    gyroOdometry.theta);
        }
        mecanumSubsystem.stop(true);
    }

    public void moveToGlobalPositionAccurate(double targetX, double targetY, double targetTheta) {
        // stop moving if within 5 ticks or 0.2 radians from the position
        while (Math.abs(targetX - gyroOdometry.x) > 2.5
                || Math.abs(targetY - gyroOdometry.y) > 2.5
                || Math.abs(targetTheta - gyroOdometry.theta) > 0.1) {

            mecanumSubsystem.fieldOrientedMove(
                    globalYController.outputPositionalCapped(targetY, gyroOdometry.y, 100),
                    -globalXController.outputPositionalCapped(targetX, gyroOdometry.x, 100),
                    globalThetaController.outputPositionalCapped(targetTheta, gyroOdometry.theta, 100),
                    gyroOdometry.theta);
        }
        mecanumSubsystem.stop(true);
    }

    public void moveToGlobalPositionLogistic(double targetX, double targetY, double targetTheta){
        //use a logistic function with a cascade controller
        //TODO: finish
        LogisticFunction logisticFunctionX = new LogisticFunction(targetX - gyroOdometry.x, 0.1, 0, 0);
        while (Math.abs(targetX - gyroOdometry.x) > 2.5
                || Math.abs(targetY - gyroOdometry.y) > 2.5
                || Math.abs(targetTheta - gyroOdometry.theta) > 0.1) {

            mecanumSubsystem.fieldOrientedMove(
                    globalCascadeYController.cascadeOutput(targetY, gyroOdometry.y, logisticFunctionX.getOutput(gyroOdometry.x), globalCascadeYController.getDerivative()),
                    -globalXController.outputPositionalCapped(targetX, gyroOdometry.x, 100),
                    globalThetaController.outputPositionalCapped(targetTheta, gyroOdometry.theta, 100),
                    gyroOdometry.theta);
        }
    }

    public void moveRotation(double targetTheta) {
        // stop moving if within 5 ticks or 0.2 radians from the position
        while (Math.abs(targetTheta - odometrySubsystem.theta) > 0.1) {

            mecanumSubsystem.fieldOrientedMove(
                    0, 0,
                    -globalThetaController.outputPositional(targetTheta, odometrySubsystem.theta),
                    0);
        }
        mecanumSubsystem.stop(true);
    }

    public void setFinalPosition(boolean run, double velocity, double x, double y, double theta){
        if (run){
            xFinal = x;
            yFinal = y;
            thetaFinal = theta;
            this.velocity = velocity;
        }
    }

    public void setFinalAprilPosition(boolean run, double velocity, double dx, double dy, double dTheta){
        if (run){
            //get x, y, theta values from the april tag
            xFinal = xFinal - dx;
            yFinal = yFinal - dy;
            thetaFinal = thetaFinal - dTheta;
            this.velocity = velocity;
        }
    }
    public double getXFinal() {
        return xFinal;
    }

    public double getYFinal() {
        return yFinal;
    }

    public double getThetaFinal() {
        return thetaFinal;
    }

    public double getVelocity() {
        return velocity;
    }

    public double getXDifference(){
        return Math.abs(xFinal - gyroOdometry.x);
    }

    public double getYDifference(){
        return Math.abs(yFinal - gyroOdometry.y);
    }

    public double getThetaDifference(){
        return Math.abs(thetaFinal - gyroOdometry.theta);
    }

    public boolean isPositionPassed(){
        return getXDifference() < 5 && getYDifference() < 5 && getThetaDifference() < 0.05;
    }

    public boolean isPositionReached(boolean xtol, boolean ytol){
        boolean withinXRange = (getXDifference() < 1.5);
        if (xtol) {
            withinXRange = (getXDifference() < 3);
        }
        boolean withinYRange = (getYDifference() < 1.5);
        if (ytol) {
            withinYRange = (getYDifference() < 3);
        }
        return withinXRange && withinYRange && getThetaDifference() < 0.1;
    }

    public boolean isCoordinateReached(){
        return getXDifference() < 2.5 && getYDifference() < 2.5;
    }

    public boolean isCoordinatePassed(){
        return getXDifference() < 5 && getYDifference() < 5;
    }

    public boolean isYReached(){
        return getYDifference() < 1.5;
    }

    public boolean isYPassed(){
        return getYDifference() < 10;
    }

    public boolean isXReached(){
        return getXDifference() < 1.5;
    }

    public boolean isXPassed(){
        return getXDifference() < 10;
    }

    public boolean isThetaReached(){
        return getThetaDifference() < 0.02;
    }

    public boolean isThetaPassed(){
        return getThetaDifference() < 0.2;
    }

    public boolean isInGeneralVicinity() {
        return getXDifference() < 10 && getYDifference() < 10 && getThetaDifference() < 0.2;
    }
}