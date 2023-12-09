package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.Drone;
import org.firstinspires.ftc.teamcode.Subsystems.LinearSlide;

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
@TeleOp(name = "Main", group = "Linear OpMode")
//@Disabled
public class Main extends LinearOpMode {
    //Not calibrated
    // Declare OpMode members for each of the 4 motors.
    private final ElapsedTime runtime = new ElapsedTime();
    IMU imu;
    Arm arm;
    Claw claw;
    Drone drone;
    LinearSlide ls;

    @Override
    public void runOpMode() {
        /// INIT
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "lfd");
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "lbd");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "rbd");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "rfd");

        /// Set modes
        //leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);


        // Drone init


        //Arm init

        arm = new Arm(hardwareMap);

        drone = new Drone(hardwareMap);

        //Claw init
        claw = new Claw(hardwareMap);

        //Linear Slide init
        ls = new LinearSlide(hardwareMap);

        //Wrist init
        //WristServo wrist = new WristServo();
        /*
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

         */

        // camera init
        // Camera camera = hardwareMap.get(Camera.class, "camera");


        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        //while(arm.armPos > 0.1){
        //    arm.up();
        //}

        //ls.up();
        //sleep(50);
        //ls.stop();;

        // run until the end of the match (driver presses STOP)
        //Drivetrain
        while (opModeIsActive()) {
            /// Field Centric

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            /*if (gamepad1.options) {
                imu.resetYaw();
            }

            double y = -gamepad1.left_stick_y / 2; // Remember, Y stick value is reversed
            double x = (gamepad1.left_stick_x * 1.1) / 2; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x / 2;

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            //double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            leftFrontDrive.setPower(frontLeftPower);
            leftBackDrive.setPower(backLeftPower);
            rightFrontDrive.setPower(frontRightPower);
            rightBackDrive.setPower(backRightPower);*/

/*
        double drive  = gamepad1.right_stick_y;
        double strafe = gamepad1.left_stick_y;
        double twist  = gamepad1.left_stick_x;

        /*
         * If we had a gyro and wanted to do field-oriented control, here
         * is where we would implement it.
         *
         * The idea is fairly simple; we have a robot-oriented Cartesian (x,y)
         * coordinate (strafe, drive), and we just rotate it by the gyro
         * reading minus the offset that we read in the init() method.
         * Some rough pseudocode demonstrating:
         *
         * if Field Oriented Control:
         *     get gyro heading
         *     subtract initial offset from heading
         *     convert heading to radians (if necessary)
         *     new strafe = strafe * cos(heading) - drive * sin(heading)
         *     new drive  = strafe * sin(heading) + drive * cos(heading)
         *
         * If you want more understanding on where these rotation formulas come
         * from, refer to
         * https://en.wikipedia.org/wiki/Rotation_(mathematics)#Two_dimensions
         */
/*
            // You may need to multiply some of these by -1 to invert direction of
            // the motor.  This is not an issue with the calculations themselves.
            double[] speeds = {
                    (drive + strafe + twist),
                    (drive - strafe - twist),
                    (drive - strafe + twist),
                    (drive + strafe - twist)
            };

            // Because we are adding vectors and motors only take values between
            // [-1,1] we may need to normalize them.

            // Loop through all values in the speeds[] array and find the greatest
            // *magnitude*.  Not the greatest velocity.
            double max = Math.abs(speeds[0]);
            for (double speed : speeds) {
                if (max < Math.abs(speed)) max = Math.abs(speed);
            }

            // If and only if the maximum is outside of the range we want it to be,
            // normalize all the other speeds based on the given speed value.
            if (max > 1) {
                for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
            }

            // apply the calculated values to the motors.
            leftFrontDrive.setPower(speeds[0]);
            rightFrontDrive.setPower(speeds[1]);
            leftBackDrive.setPower(speeds[2]);
            rightBackDrive.setPower(speeds[3]);
*/
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

            leftFrontDrive.setPower(frontLeftPower);
            leftBackDrive.setPower(backLeftPower);
            rightFrontDrive.setPower(frontRightPower);
            rightBackDrive.setPower(backRightPower);

            /// Linear Slide
            if (gamepad2.dpad_up) {
                ls.up();
            } else if (gamepad2.dpad_down) {
                ls.down();
            } else {
                ls.stop();
            }


            ///Airplane Launcher
            if (gamepad2.start) {
                drone.launch();
            } else if (gamepad2.back) {
                drone.reset();
            }

            // Arm
            if (gamepad2.x) {
                arm.up();
            }
            if (gamepad2.y) {
                arm.down();
            }
            /*
            if (gamepad1.dpad_right) {
                arm.right();
            }
            if (gamepad1.dpad_left) {
                arm.left();
            }*/


            /* Wrist
            if (gamepad1.dpad_left) {
                wrist.up();
            }
            if (gamepad1.dpad_right) {
                wrist.down();
            }

            if (gamepad2.options) {
                imu.resetYaw();
            }


             */

            // Claw
            if (gamepad2.left_bumper) {
                claw.close();
            }
            if (gamepad2.right_bumper) {
                claw.open();
            }


        }
    }
}
