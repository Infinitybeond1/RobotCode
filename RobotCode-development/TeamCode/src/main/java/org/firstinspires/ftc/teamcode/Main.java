/* Copyright (c) 2021 FIRST. All rights reserved.
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

//import androidx.compose.ui.text.CacheTextLayoutInput;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
*/

//@Config
@TeleOp(name="Main", group="Linear OpMode")
//@Disabled
public class Main extends LinearOpMode {
    // Declare OpMode members for each of the 4 motors.
    private final ElapsedTime runtime = new ElapsedTime();
    public static volatile double STARTPOS_D = 0.648;
    public static volatile double LAUNCHPOS_D = 1;

    //Not calibrated
    public static volatile double ARM_BACK = 0;
    public static volatile double ARM_MID = 0.5;
    public static volatile double ARM_FRONT = 1;
    public static volatile double CLAW_CLOSE = 0.5;
    public static volatile double CLAW_OPEN = .75;

    IMU imu;

    @Override
    public void runOpMode() {
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        /// INIT
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "lfd");
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "lbd");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "rfd");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "rbd");

        //Change motor direction
        //rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        //rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);

        //Set modes
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Drone init
        Servo d1 = hardwareMap.get(Servo.class, "drone");

        d1.setPosition(STARTPOS_D);

        //Arm init
        Servo a1 = hardwareMap.get(Servo.class, "a1");
        Servo a2 = hardwareMap.get(Servo.class, "a2");

        a2.setDirection(Servo.Direction.REVERSE);

        a1.setPosition(ARM_MID);
        a2.setPosition(ARM_MID);

        //Claw init
        Servo c1 = hardwareMap.get(Servo.class, "claw");

        c1.setPosition(CLAW_OPEN);

        //Linear Slide init
        DcMotor ls1 = hardwareMap.get(DcMotor.class, "ls1");
        DcMotor ls2 = hardwareMap.get(DcMotor.class, "ls2");

        ls1.setDirection(DcMotor.Direction.FORWARD);
        ls2.setDirection(DcMotor.Direction.FORWARD);


        ls1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ls2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");

        //IMU init
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                )
        );

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        //Drivetrain
        while (opModeIsActive()){
            /*double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            /// Field Centric

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            leftFrontDrive.setPower(frontLeftPower);
            leftBackDrive.setPower(backLeftPower);
            rightFrontDrive.setPower(frontRightPower);
            rightBackDrive.setPower(backRightPower);*/

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            if(y != 0 || x != 0 || rx != 0) {
                leftFrontDrive.setPower(frontLeftPower);
                leftBackDrive.setPower(backLeftPower);
                rightFrontDrive.setPower(frontRightPower);
                rightBackDrive.setPower(backRightPower);
            }else {
                leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }


            /// Linear Slide
            //ls1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //ls2.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            if (gamepad2.dpad_up) {
                ls1.setPower(0.5);
                ls2.setPower(0.5);
            }else if (gamepad2.dpad_down) {
                ls1.setPower(-0.5);
                ls2.setPower(-0.5);
            }else {
                ls1.setPower(0);
                ls2.setPower(0);
            }

            ///Airplane Launcher
            if (gamepad2.x) {
                if (d1.getPosition() == STARTPOS_D) {
                    d1.setPosition(LAUNCHPOS_D);
                } else {
                    d1.setPosition(STARTPOS_D);
                }
            }

            // Arm; a2 is reversed
            if (gamepad2.right_bumper) {
                if (a1.getPosition() == ARM_MID) {
                    a1.setPosition(ARM_FRONT);
                    a2.setPosition(ARM_FRONT);
                } else {
                    a1.setPosition(ARM_MID);
                    a2.setPosition(ARM_MID);
                }
            }
            if (gamepad2.left_bumper) {
                if (a2.getPosition() == ARM_MID) {
                    a1.setPosition(ARM_BACK);
                    a2.setPosition(ARM_BACK);
                } else {
                    a2.setPosition(ARM_MID);
                    a2.setPosition(ARM_MID);
                }
            }

            if (gamepad2.options) {
                imu.resetYaw();
            }

            // Claw
            if (gamepad2
                    .b) {
                if (c1.getPosition() == CLAW_CLOSE) {
                    c1.setPosition(CLAW_OPEN);
                } else {
                    c1.setPosition(CLAW_CLOSE);
                }

            }

        }
    }
}
