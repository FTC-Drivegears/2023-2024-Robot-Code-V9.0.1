package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.Pipelines.AprilTagPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class AprilCamSubsystem {

    public OpenCvCamera webcam;
    public AprilTagPipeline aprilTagPipeline;
    boolean initial;
    public static final int VIEW_WIDTH = 320;
    public static final int VIEW_HEIGHT = 176;
    private int cameraMonitorViewId; // ID of the viewport which camera feed will be displayed
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    public AprilCamSubsystem(HardwareMap hardwareMap){
        aprilTagPipeline = new AprilTagPipeline(0.166, fy, fx, cy, cx);
        webcam.setPipeline(aprilTagPipeline);
        // initiate the needed parameters
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId","id",hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        // initiate the camera object with created parameters and pipeline
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        webcam.setPipeline(aprilTagPipeline);

        // runs camera on a separate thread so it can run simultaneously with everything else
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){
            @Override
            public void onOpened() {
                // starts the camera stream when init is pressed
                webcam.startStreaming(VIEW_WIDTH,VIEW_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    public double getTagCenterX(int tagID){
        ArrayList<AprilTagDetection> detections = aprilTagPipeline.getLatestDetections();
        for(AprilTagDetection detection : detections){
            if(detection.id == tagID){
                return detection.center.x;
            }
        }
        return -1;
    }
    public double getTagCenterY(int tagID){
        ArrayList<AprilTagDetection> detections = aprilTagPipeline.getLatestDetections();
        for(AprilTagDetection detection : detections){
            if(detection.id == tagID){
                return detection.center.y;
            }
        }
        return -1;
    }
}
