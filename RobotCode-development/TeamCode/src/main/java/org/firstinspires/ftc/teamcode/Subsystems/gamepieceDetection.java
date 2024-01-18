package org.firstinspires.ftc.teamcode.Subsystems;


import android.graphics.Canvas;


import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;



public class gamepieceDetection implements VisionProcessor {


    // Enumeration that helps set what color the game team prop is
    public enum Prop {
        BLUE, RED
    }


    // Enumeration that holds the location that the prop is detected at
    public enum Location {
        Left,
        Center,
        Right
    }
    private Location location;
    public Prop propColor = Prop.BLUE;


    // Scalars that provide the lowHSV and highHSV for image processing
    Scalar upperLBoundR = new Scalar(25, 255, 255);
    Scalar lowerLBoundR = new Scalar(0, 20, 5);
    Scalar upperUBoundR = new Scalar(360, 255, 255);
    Scalar lowerUBoundR = new Scalar(345, 20, 20);

    Scalar upperBoundB = new Scalar(130, 255, 255);
    Scalar lowerBoundB = new Scalar(85, 20, 5 );




    // Rects define the areas that the prop must be on to be considered left, right, or center
    // remove the multipliers when building on robot. x multipliers -3, y multipliers -2.25
    static final Rect LEFT_ROI = new Rect(
            new Point(0, 0),
            new Point(160, 480));
    static final Rect RIGHT_ROI = new Rect(
            new Point(480, 0),
            new Point(640,480));


    static final Rect CENTER_ROI = new Rect(
            new Point(160, 0),
            new Point(480,480));



    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Not useful
    }


    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGB2HSV); // Converts image to HSV color


        if (propColor == Prop.RED) {
            Core.inRange(frame, lowerLBoundR, upperLBoundR, frame);
        } else {
            Core.inRange(frame, lowerBoundB, upperBoundB, frame);
        }


        Mat left = frame.submat(LEFT_ROI);
        Mat right = frame.submat(RIGHT_ROI);
        Mat center = frame.submat(CENTER_ROI);

        int leftCount = Core.countNonZero(left);
        int centerCount = Core.countNonZero(center);
        int rightCount = Core.countNonZero(right);


        left.release();
        right.release();
        center.release();

        double max = Math.max(leftCount, Math.max(rightCount, centerCount));


        if (leftCount == max){
            location = Location.Left;
        } else if (rightCount == max) {
            location = Location.Right;
        } else {
            location = Location.Center;
        }

        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_GRAY2RGB);


        Scalar red = new Scalar(255, 0, 0);
        Scalar green = new Scalar(0, 255, 0);


        Imgproc.rectangle(frame, LEFT_ROI, location == Location.Left? green: red);
        Imgproc.rectangle(frame, RIGHT_ROI, location == Location.Right? green: red);
        Imgproc.rectangle(frame, CENTER_ROI, location == Location.Center? green: red);
        Imgproc.putText(frame, location.name(), new Point(220*3,60*2.25),1,5, red);


        return null; // No context object
    }


    public Location getLocation() {
        return location;
    }


    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // Not useful either
    }
}


