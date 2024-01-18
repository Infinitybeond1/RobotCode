package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpenCVTest;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.firstinspires.ftc.teamcode.Subsystems.gamepieceDetection;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

public class Camera {
    private AprilTagProcessor aprilTag;
    public VisionPortal visionPortal;
    private WebcamName armCam;
    public gamepieceDetection propDetector;
    Telemetry telemetry;


    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 480; // height of wanted camera resolution

    public Camera(HardwareMap hardwareMap, Telemetry telemetry1) {

        propDetector = new gamepieceDetection();
        armCam = hardwareMap.get(WebcamName.class, "armCam");
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "armCam"), propDetector);

        telemetry = telemetry1;



        propDetector.propColor = gamepieceDetection.Prop.RED;

        /*
        visionPortal = new VisionPortal.Builder()
                .setCamera(armCam)
                .addProcessors(aprilTag, propDetector)
                .build();
*/
    }

    public double[] updateDataAT(int targetID) {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        double tpx, tpy, tpz, tdp, tdr, tdy, tr, tb, te;

        // Step through the list of detections, check for targets and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == targetID) {
                tpx = detection.ftcPose.x;
                tpy = detection.ftcPose.y;
                tpz = detection.ftcPose.z;
                tdp = detection.ftcPose.pitch;
                tdr = detection.ftcPose.roll;
                tdy = detection.ftcPose.yaw;
                tr = detection.ftcPose.range;
                tb = detection.ftcPose.bearing;
                te = detection.ftcPose.elevation;
                double[] output = {tpx, tpy, tpz, tdp, tdr, tdy, tr, tb, te};
                return output;

            }

        }   // end for() c
        return null;
    }

    public String getGamepiecePos(){
        /*
        if(propDetector.getLocation() == gamepieceDetection.Location.Left){
            return "left";
        }
        if(propDetector.getLocation() == gamepieceDetection.Location.Right){
            return "right";
        }
        if(propDetector.getLocation() == gamepieceDetection.Location.Center){
            return "center";
        }
        return "none";

         */
        visionPortal.getProcessorEnabled(propDetector);
        return String.valueOf(propDetector.getLocation());
    }


}