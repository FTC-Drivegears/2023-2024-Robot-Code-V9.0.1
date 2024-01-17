package org.firstinspires.ftc.teamcode.drive.opmode.Tests;

import org.opencv.core.*;
import org.opencv.core.Point;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;

public class LineDetection {

    public static void main(String[] args) {
        //load opencv library
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

        // Open a camera stream (use 0 for the default camera)
        VideoCapture capture = new VideoCapture(0);
        if (!capture.isOpened()) {
            System.out.println("Error: Camera not found!");
            return;
        }

        //current frame captured by the camera
        Mat frame = new Mat();
        //used to store the image in hsv (hue, saturation, value) form
        Mat hsv = new Mat();
        //image after thresholding red color (binary mask with pixels in threshold white, everything else black)
        Mat threshold = new Mat();
        //result of running edge detection on mask
        Mat edges = new Mat();

        while(true) {
            // Capture a frame from the camera
            capture.read(frame);

            // Convert the frame to HSV
            Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_BGR2HSV);

            // Threshold the image to isolate red color
            Core.inRange(hsv, new Scalar(0, 100, 100), new Scalar(10, 255, 255), threshold);

            // Apply Canny edge detection
            Imgproc.Canny(threshold, edges, 50, 150);

            // Use HoughLines to detect lines
            Mat lines = new Mat();
            Imgproc.HoughLines(edges, lines, 1, Math.PI / 180, 150);

            // Draw the detected lines on the original frame
            for (int i = 0; i < lines.rows(); i++) {
                double[] vec = lines.get(i, 0);
                double rho = vec[0];
                double theta = vec[1];
                double cosTheta = Math.cos(theta);
                double sinTheta = Math.sin(theta);
                double x0 = cosTheta * rho;
                double y0 = sinTheta * rho;
                Point pt1 = new Point(x0 + 1000 * (-sinTheta), y0 + 1000 * (cosTheta));
                Point pt2 = new Point(x0 - 1000 * (-sinTheta), y0 - 1000 * (cosTheta));
                Imgproc.line(frame, pt1, pt2, new Scalar(0, 0, 255), 2);
            }

        }

    }
}
