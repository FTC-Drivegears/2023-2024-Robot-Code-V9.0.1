package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.util.Pipelines.AprilTagPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.ArrayList;

public class AprilCamSubsystem {

    public OpenCvCamera webcam;
    int cameraMonitorViewId;
    public AprilTagPipeline aprilTagPipeline;
    boolean initial;
    public static final int VIEW_WIDTH = 320;
    public static final int VIEW_HEIGHT = 176;
    public static final int CENTER_X = VIEW_WIDTH / 2;
    public static final int CENTER_Y = VIEW_HEIGHT / 2;
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    public AprilCamSubsystem(){
        aprilTagPipeline = new AprilTagPipeline(0.166, fy, fx, cy, cx);
        webcam.setPipeline(aprilTagPipeline);
    }

    public ArrayList<AprilTagDetection> getAprilTag(){
        if(!aprilTagPipeline.getLatestDetections().isEmpty()){
            //this returns an arraylist of all detected april tags
            //it contains: public int id;
            //    public int hamming;
            //    public float decisionMargin;
            //    public Point center;
            //    public Point[] corners;
            //    public AprilTagPose pose;
            return aprilTagPipeline.getLatestDetections();
        }
        else{
            return null;
        }
    }
}
