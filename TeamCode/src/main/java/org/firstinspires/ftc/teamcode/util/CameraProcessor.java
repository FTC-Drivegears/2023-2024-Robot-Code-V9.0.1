package org.firstinspires.ftc.teamcode.util;

import android.graphics.Bitmap;
import android.graphics.Canvas;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.subsystems.WebcamSubsystem;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class CameraProcessor implements VisionProcessor {

    private final Size KERNEL = new Size(20, 20);

    private final FtcDashboard DASHBOARD;
    private final Gamepad GAMEPAD;

    private final Scalar UPPER_HSV;
    private final Scalar LOWER_HSV;

    public CameraProcessor(FtcDashboard dashboard, Gamepad gamepad,
                           WebcamSubsystem.PipelineName pipelineName) {
        DASHBOARD = dashboard;
        GAMEPAD = gamepad;

        if (pipelineName == WebcamSubsystem.PipelineName.CONTOUR_BLUE) {
            UPPER_HSV = new Scalar(138, 255, 255);
            LOWER_HSV = new Scalar(87, 218, 143);
        } else {
            UPPER_HSV = new Scalar(38, 255, 255);
            LOWER_HSV = new Scalar(0, 227, 102);
        }
    }

    // Runs when the VisionProcessor makes this pipeline

    private int width;
    private int height;

    public void init(int width, int height, CameraCalibration calibration) {
        this.width = width;
        this.height = height;
    }

    // Runs when a frame from the camera is drawn, but we don't do anything here
    public void onDrawFrame(Canvas canvas,
                            int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                            Object userContext) {
    }

    /**
     * Converts the Mat frame into a bitmap before sending it to the dashboard.
     *
     * @param frame            the Mat matrix object containing (ie. what the camera sees)
     * @param captureTimeNanos
     * @return
     */
    public Object processFrame(Mat frame, long captureTimeNanos) {

        // Only display contours if A is pressed
        if (GAMEPAD.a) {

            // blurring the input image to improve edge and contour detection
            Imgproc.blur(frame, frame, KERNEL);

            // convert blurred image into HSV scale
            Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGB2HSV);

            // create a HSV threshold that filters out yellow shades
            Core.inRange(frame, LOWER_HSV, UPPER_HSV, frame);
        }

        Bitmap bm = Bitmap.createBitmap(width, height, Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(frame, bm);
        DASHBOARD.sendImage(bm);
        return frame;
    }

}
