package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Pipelines.AprilTagPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class AprilCamSubsystem {
    AprilTagProcessor aprilTagProcessor;
    VisionPortal visionPortal;

    ArrayList<AprilTagDetection> detections;
    OpenCvCamera webcam;

    //get aprilTagProcessor
    public AprilTagProcessor getAprilTagProcessor(){
        return aprilTagProcessor;
    }

    //get visionPortal
    public VisionPortal getVisionPortal(){
        return visionPortal;
    }

    public AprilCamSubsystem(HardwareMap hardwareMap){


        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTagProcessor)
                .setCamera(hardwareMap.get(CameraName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();
        detections = new ArrayList<>();
    }

    //starts gathering detections (process)
    public void runDetections(){
        if(visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING){
            detections = aprilTagProcessor.getDetections();
        }
    }

    //returns list of all april tag detections
    public ArrayList<AprilTagDetection> getDetections(){
        return detections;
    }

    //reutrns list of all fieldpositions from april tag metadata
    public ArrayList<VectorF> getFieldPositions(){
        ArrayList<VectorF> fieldPositions = new ArrayList<>();
        for(AprilTagDetection detection : detections){
            fieldPositions.add(detection.metadata.fieldPosition);
        }
        return fieldPositions;
    }

    //returns list of distanceunits from april tag metadata
    public ArrayList<DistanceUnit> getDistanceUnits(){
        ArrayList<DistanceUnit> distanceUnits = new ArrayList<>();
        for(AprilTagDetection detection : detections){
            distanceUnits.add(detection.metadata.distanceUnit);
        }
        return distanceUnits;
    }

    //returns list of tagids from april tag
    public ArrayList<Integer> getTagIDs(){
        ArrayList<Integer> tagIDs = new ArrayList<>();
        for(AprilTagDetection detection : detections){
            tagIDs.add(detection.id);
        }
        return tagIDs;
    }

    //returns list of x positions from april tag
    public ArrayList<Double> getXPositions(){
        ArrayList<Double> xPositions = new ArrayList<>();
        for(AprilTagDetection detection : detections){
            xPositions.add(detection.ftcPose.x);
        }
        return xPositions;
    }

    //returns list of y positions from april tag
    public ArrayList<Double> getYPositions(){
        ArrayList<Double> yPositions = new ArrayList<>();
        for(AprilTagDetection detection : detections){
            yPositions.add(detection.ftcPose.y);
        }
        return yPositions;
    }

    //returns list of z positions from april tag
    public ArrayList<Double> getZPositions(){
        ArrayList<Double> zPositions = new ArrayList<>();
        for(AprilTagDetection detection : detections){
            zPositions.add(detection.ftcPose.z);
        }
        return zPositions;
    }

    //returns list of yaw angles from april tag
    public ArrayList<Double> getYawAngles(){
        ArrayList<Double> yawAngles = new ArrayList<>();
        for(AprilTagDetection detection : detections){
            yawAngles.add(detection.ftcPose.yaw);
        }
        return yawAngles;
    }

    //returns list of pitch angles from april tag
    public ArrayList<Double> getPitchAngles(){
        ArrayList<Double> pitchAngles = new ArrayList<>();
        for(AprilTagDetection detection : detections){
            pitchAngles.add(detection.ftcPose.pitch);
        }
        return pitchAngles;
    }

    //returns list of roll angles from april tag
    public ArrayList<Double> getRollAngles(){
        ArrayList<Double> rollAngles = new ArrayList<>();
        for(AprilTagDetection detection : detections){
            rollAngles.add(detection.ftcPose.roll);
        }
        return rollAngles;
    }

    //returns list of ranges from april tag
    public ArrayList<Double> getRanges(){
        ArrayList<Double> ranges = new ArrayList<>();
        for(AprilTagDetection detection : detections){
            ranges.add(detection.ftcPose.range);
        }
        return ranges;
    }

    //returns list of bearings from april tag
    public ArrayList<Double> getBearings(){
        ArrayList<Double> bearings = new ArrayList<>();
        for(AprilTagDetection detection : detections){
            bearings.add(detection.ftcPose.bearing);
        }
        return bearings;
    }

}
