/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import static org.firstinspires.ftc.teamcode.Subsystems.Claw.*;


import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.Drone;
import org.firstinspires.ftc.teamcode.Subsystems.LinearSlide;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import java.util.List;




/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */


@TeleOp(name="Auton that works CLICK THIS", group="Linear OpMode")
//@Disabled
public class Auto4 extends LinearOpMode {


    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    int tid = 2;
    Arm arm;
    Claw claw;
    LinearSlide ls;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Arm init
        arm = new Arm(hardwareMap);

        //Claw init
        claw = new Claw(hardwareMap);

        //Linear Slide init
        ls = new LinearSlide(hardwareMap);


        // init parts
        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "lfd");
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "lbd");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "rfd");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "rbd");


        //the actual stuff here

        claw.close();

        while (arm.armPos < 0.8){
            arm.up();
        }
        //drive forward until purple pixel on tape
        leftFrontDrive.setPower(-0.5);
        leftBackDrive.setPower(-0.5);
        rightFrontDrive.setPower(-0.5);
        rightBackDrive.setPower(-0.5);
        sleep(1000);

        //go back
        leftFrontDrive.setPower(0.5);
        leftBackDrive.setPower(0.5);
        rightFrontDrive.setPower(0.5);
        rightBackDrive.setPower(0.5);
        sleep(500);


        //strafe until purple pixel is free of path

        leftFrontDrive.setPower(-0.5);
        leftBackDrive.setPower(0.5);
        rightFrontDrive.setPower(0.5);
        rightBackDrive.setPower(-0.5);
        sleep(400);
/*
        //turn 90 cw/ccw
        leftFrontDrive.setPower(-0.5);
        leftBackDrive.setPower(-0.5);
        rightFrontDrive.setPower(0.5);
        rightBackDrive.setPower(0.5);
        sleep(300);

        //forward until whole bot is in front of truss
        leftFrontDrive.setPower(-0.5);
        leftBackDrive.setPower(-0.5);
        rightFrontDrive.setPower(-0.5);
        rightBackDrive.setPower(-0.5);
        sleep(1500);

        //strafe until april tag is detected


        initAprilTag();
        double[] tData = {-1,-1};
        while(tData[1] < 0 ){
            tData = getTagDataXY(tid);

            leftFrontDrive.setPower(0.5);
            leftBackDrive.setPower(-0.5);
            rightFrontDrive.setPower(-0.5);
            rightBackDrive.setPower(0.5);

            sleep(10);

        }


        //strafe until april tag is in front of bot

        while(tData[0] > -100 ){
            tData = getTagDataXY(tid);

            leftFrontDrive.setPower(0.5);
            leftBackDrive.setPower(-0.5);
            rightFrontDrive.setPower(-0.5);
            rightBackDrive.setPower(0.5);

            sleep(10);

        }


        //forward until april tag is good distance
        while(tData[1] > 100 ){
            tData = getTagDataXY(tid);

            leftFrontDrive.setPower(-0.5);
            leftBackDrive.setPower(-0.5);
            rightFrontDrive.setPower(-0.5);
            rightBackDrive.setPower(-0.5);

            sleep(10);

        }

        //place pixel

        ls.up();
        sleep(400);
        ls.hold();

        while(arm.armPos < 0.5){
            arm.up();
        }
        claw.open();


*/
        //forward a bit more to park




    }


    //april tag funcs
    private void initAprilTag() {


        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)


                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)


                // ... these parameters are fx, fy, cx, cy.


                .build();


        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();


        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }


        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));


        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);


        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);


        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);


        // Set and enable the processor.
        builder.addProcessor(aprilTag);


        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();


        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);


    }   // end method initAprilTag()
    private double[] getTagDataXY(int target) {


        List<AprilTagDetection> currentDetections = aprilTag.getDetections();


        // Step through the list of detections and find target.
        double[] tData = {2};
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && detection.id == target) {


                tData[0] = (detection.ftcPose.x);
                tData[1] = detection.ftcPose.y;
                return tData;


            }
        }   // end for() loop
        tData[0] = -1;
        tData[1] = -1;
        return tData;


    }   // end method telemetryAprilTag()
}





