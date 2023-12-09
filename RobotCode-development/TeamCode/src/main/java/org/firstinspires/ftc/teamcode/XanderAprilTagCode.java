package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Apriltag", group = "Concept")
//@Disabled
public class XanderAprilTagCode extends LinearOpMode{
    private static final boolean USE_WEBCAM = true;

    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;

    private int tid = 2;
    private double tpx, tpy, tpz, tdp, tdr, tdy, tr, tb, te;

    @Override
    public void runOpMode() {

        initAprilTag();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                functionAprilTag();

                // Push telemetry to the Driver Station.
                telemetry.update();

                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }

                // Share the CPU.
                sleep(20);
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end method runOpMode()

    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "front"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }

    }   // end method initAprilTag()

    private void functionAprilTag(){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if(detection.id == tid){
                telemetry.addLine("Target Acquired.");
                tpx = detection.ftcPose.x;tpy = detection.ftcPose.y;tpz = detection.ftcPose.z;
                tdp = detection.ftcPose.pitch;tdr = detection.ftcPose.roll; tdy = detection.ftcPose.yaw;
                tr = detection.ftcPose.range; tb = detection.ftcPose.bearing; te = detection.ftcPose.elevation;

                telemetry.addData("Robot will have to move right(inches) ", tpx);
                telemetry.addData(", forward ", tpy);
                telemetry.addData(" and up ", tpz);

                /*
                telemetry.addData("Robot will have turn up(degrees clockwise) ", tdp);
                telemetry.addData(", roll ", tdr);
                telemetry.addData(" and turn right", tdy);

                 */





            }
        }   // end for() loop
    }
}
