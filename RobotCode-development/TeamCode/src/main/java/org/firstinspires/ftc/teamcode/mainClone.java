package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.Drone;
import org.firstinspires.ftc.teamcode.Subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.Subsystems.Camera;
import org.firstinspires.ftc.teamcode.Subsystems.WristServo;


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
@TeleOp(name = "mainClone", group = "Linear OpMode")
//@Disabled

public class mainClone extends LinearOpMode {


    //Not calibrated
    // Declare OpMode members for each of the 4 motors.
    private final ElapsedTime runtime = new ElapsedTime();
    //IMU imu;
    Arm arm;
    Drone drone;
    LinearSlide ls;
    Camera cam;
    Claw claw;
    WristServo wrist;
    double lastFrontLeftPower;
    double lastFrontRightPower;
    double lastBackLeftPower;
    double lastBackRightPower;
    boolean DTReversed = false;



    @Override
    public void runOpMode() {
        /// INIT
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices

        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "lfd");
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "lbd");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "rbd");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "rfd");

        /// Set modes

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);


        // Drone init


        //Arm init
        arm = new Arm(hardwareMap);

        //drone = new Drone(hardwareMap);

        //Claw init
        claw = new Claw(hardwareMap);

        //Wrist Init
        wrist = new WristServo(hardwareMap);

        //cam init
        cam = new Camera(hardwareMap, telemetry);

        //Linear Slide init
        //ls = new LinearSlide(hardwareMap, telemetry);

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
        //runtime.reset();
        //arm.setPos(arm.ARMPOSPICKUP);
        //wrist.setPos(wrist.WRISTPOSPICKUP);

        while (opModeIsActive()) {
            /*
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

            double acc = 0.02;

            frontLeftPower = frontLeftPower > lastFrontLeftPower + acc ? lastFrontLeftPower + acc : frontLeftPower;
            backLeftPower = backLeftPower > lastBackLeftPower + acc ? lastBackLeftPower + acc : backLeftPower;
            frontRightPower = frontRightPower > lastFrontRightPower + acc ? lastFrontRightPower + acc : frontRightPower;
            backRightPower = backRightPower > lastBackRightPower + acc ? lastBackRightPower + acc : backRightPower;

            frontLeftPower = frontLeftPower < lastFrontLeftPower - acc ? lastFrontLeftPower - acc : frontLeftPower;
            backLeftPower = backLeftPower < lastBackLeftPower - acc ? lastBackLeftPower - acc : backLeftPower;
            frontRightPower = frontRightPower < lastFrontRightPower - acc ? lastFrontRightPower - acc : frontRightPower;
            backRightPower = backRightPower < lastBackRightPower - acc ? lastBackRightPower - acc : backRightPower;


            leftFrontDrive.setPower(frontLeftPower);
            leftBackDrive.setPower(backLeftPower);
            rightFrontDrive.setPower(frontRightPower);
            rightBackDrive.setPower(backRightPower);

            lastFrontLeftPower = frontLeftPower;
            lastBackLeftPower = backLeftPower;
            lastFrontRightPower = frontRightPower;
            lastBackRightPower = backRightPower;

            if(gamepad1.a){
                DTReversed = !DTReversed;
                while(gamepad1.a);
            }

            if(DTReversed){
                leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
                leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
                rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
                rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
            }else{
                leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
                leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
                rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
                rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
            }



            /// Linear Slide
            ls.targetPos -=10 * gamepad2.right_stick_y;

            /// Claw
            if(gamepad2.right_bumper) {
                claw.claw1IsOpen = !claw.claw1IsOpen;
                claw.updatePos();
                telemetry.addData("claw1: ", claw.claw1IsOpen);
                telemetry.update();
                telemetry.addData("claw2: ", claw.claw2IsOpen);
                telemetry.update();
                while(gamepad2.right_bumper);
            }

            if(gamepad2.left_bumper) {
                claw.claw2IsOpen = !claw.claw2IsOpen;
                claw.updatePos();
                telemetry.addData("claw1: ", claw.claw1IsOpen);
                telemetry.update();
                telemetry.addData("claw2: ", claw.claw2IsOpen);
                telemetry.update();
                while(gamepad2.left_bumper);
            }

            if(gamepad2.a){
                if(claw.claw1IsOpen ^ claw.claw2IsOpen) {
                    claw.claw1IsOpen = false;
                    claw.claw2IsOpen = false;
                }else{
                    claw.claw1IsOpen = !claw.claw1IsOpen;
                    claw.claw2IsOpen = !claw.claw2IsOpen;
                }
                claw.updatePos();
                while(gamepad1.a);

            }

            ///Arm & Wist
            if (gamepad2.b) {
                arm.setPos(arm.ARMPOSSCORING);
                wrist.setPos(wrist.WRISTPOSSCORING);
            } else if (gamepad2.x) {
                arm.setPos(arm.ARMPOSPICKUP);
                wrist.setPos(wrist.WRISTPOSPICKUP);
            }

            ///Arm
            if(gamepad2.dpad_up){
                arm.up();
                telemetry.addData("armpos", arm.armPos);
                telemetry.update();
            } else if (gamepad2.dpad_down) {
                arm.down();
                telemetry.addData("armpos", arm.armPos);
                telemetry.update();
            }

            ///Wrist
            if(gamepad2.dpad_right){
                wrist.up();
                telemetry.addData("wristpos", wrist.wristPos);
                telemetry.update();
            } else if (gamepad2.dpad_left) {
                wrist.down();
                telemetry.addData("wristpos", wrist.wristPos);
                telemetry.update();
            }

            ///Drone
            if(gamepad2.y){
                telemetry.addLine("launhing drone");
                telemetry.update();
            }

            ///Airplane Launcher
            /*
            if (gamepad2.start) {
                drone.launch();
            } else if (gamepad2.back) {
                drone.reset();
            }

             */
           /* if(opModeIsActive()) {
                ls.staticTick();
            }

            telemetry.addData("wristpos", wrist.wristPos);
            telemetry.addData("armpos", arm.armPos);
            telemetry.addData("motor positions: ", "", leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
            telemetry.update();*/
        }

    }


}
