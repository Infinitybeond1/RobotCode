package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@TeleOp(name = "Main", group = "Linear OpMode")

public class Main extends LinearOpMode {


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
    //boolean DTReversed = false;
    boolean red = true;
    boolean robotCentric = true;

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
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        //Arm init
        arm = new Arm(hardwareMap);

        //Drone
        drone = new Drone(hardwareMap);

        //Claw init
        claw = new Claw(hardwareMap);

        //Wrist Init
        wrist = new WristServo(hardwareMap);

        //Linear Slide init
        ls = new LinearSlide(hardwareMap, telemetry);

        // camera init
        // Camera camera = hardwareMap.get(Camera.class, "camera");


        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        arm.setPos(arm.ARMPOSDEFAULT);
        wrist.setPos(wrist.WRISTPOSDEFAULT);
        claw.updatePos();


        while (opModeIsActive()) {

            double y;
            double x;
            double rx;
            double frontLeftPower;
            double backLeftPower;
            double frontRightPower;
            double backRightPower;

            if(robotCentric) {
                y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
                x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
                rx = gamepad1.right_stick_x;
                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                frontLeftPower = (y + x + rx) / denominator;
                backLeftPower = (y - x + rx) / denominator;
                frontRightPower = (y - x - rx) / denominator;
                backRightPower = (y + x - rx) / denominator;
            } else {
                x = -gamepad1.left_stick_y * 1.1; // Remember, Y stick value is reversed
                y = -gamepad1.left_stick_x; // Counteract imperfect strafing
                rx = gamepad1.right_stick_x;
                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                frontLeftPower = (y + x + rx) / denominator;
                backLeftPower = (y - x + rx) / denominator;
                frontRightPower = (y - x - rx) / denominator;
                backRightPower = (y + x - rx) / denominator;
            }


            if(gamepad1.start){
                robotCentric = !robotCentric;
                while(gamepad1.start);
            }


            double acc = 0.04;

            /*
            frontLeftPower = frontLeftPower > lastFrontLeftPower + acc ? lastFrontLeftPower + acc : frontLeftPower;
            backLeftPower = backLeftPower > lastBackLeftPower + acc ? lastBackLeftPower + acc : backLeftPower;
            frontRightPower = frontRightPower > lastFrontRightPower + acc ? lastFrontRightPower + acc : frontRightPower;
            backRightPower = backRightPower > lastBackRightPower + acc ? lastBackRightPower + acc : backRightPower;

            frontLeftPower = frontLeftPower < lastFrontLeftPower - acc ? lastFrontLeftPower - acc : frontLeftPower;
            backLeftPower = backLeftPower < lastBackLeftPower - acc ? lastBackLeftPower - acc : backLeftPower;
            frontRightPower = frontRightPower < lastFrontRightPower - acc ? lastFrontRightPower - acc : frontRightPower;
            backRightPower = backRightPower < lastBackRightPower - acc ? lastBackRightPower - acc : backRightPower;
            */

            leftFrontDrive.setPower(frontLeftPower);
            leftBackDrive.setPower(backLeftPower);
            rightFrontDrive.setPower(frontRightPower);
            rightBackDrive.setPower(backRightPower);

            lastFrontLeftPower = frontLeftPower;
            lastBackLeftPower = backLeftPower;
            lastFrontRightPower = frontRightPower;
            lastBackRightPower = backRightPower;

            /*
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

             */

            /// Linear Slide
            ls.targetPos -= 15 * gamepad2.right_stick_y;

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


            if(gamepad2.left_trigger > 0.5){
                claw.claw1IsOpen = true;
                claw.claw2IsOpen = true;
                claw.updatePos();
                while(gamepad2.left_trigger > 0.5);

            }

            if(gamepad2.right_trigger > 0.5){
                claw.claw1IsOpen = false;
                claw.claw2IsOpen = false;
                claw.updatePos();
                while(gamepad2.right_trigger > 0.5);

            }

            if(gamepad2.start){
                red = !red;
                while(gamepad2.start);
            }

            boolean scoring;
            boolean pickup;
            boolean backboard;
            boolean down;

            if(red) {
                telemetry.addLine("\n" +
                        "██████╗░███████╗██████╗░\n" +
                        "██╔══██╗██╔════╝██╔══██╗\n" +
                        "██████╔╝█████╗░░██║░░██║\n" +
                        "██╔══██╗██╔══╝░░██║░░██║\n" +
                        "██║░░██║███████╗██████╔╝\n" +
                        "╚═╝░░╚═╝╚══════╝╚═════╝░");

                leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
                leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
                rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
                rightBackDrive.setDirection(DcMotor.Direction.FORWARD);



                scoring = gamepad2.b;
                pickup = gamepad2.x;
                backboard = gamepad2.dpad_right;
                down = gamepad2.dpad_left;
            } else {
                telemetry.addLine("\n" +
                        "██████╗░██╗░░░░██╗░░░██╗███████╗\n" +
                        "██╔══██╗██║░░░░██║░░░██║██╔════╝\n" +
                        "██████╦╝██║░░░░██║░░░██║█████╗░░\n" +
                        "██╔══██╗██║░░░░██║░░░██║██╔══╝░░\n" +
                        "██████╦╝██████╗╚██████╔╝███████╗\n" +
                        "╚═════╝░╚═════╝░╚═════╝░╚══════╝");

                if (robotCentric){
                    leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
                    leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
                    rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
                    rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
                } else{
                    leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
                    leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
                    rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
                    rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
                }

                scoring = gamepad2.x;
                pickup = gamepad2.b;
                backboard = gamepad2.dpad_left;
                down = gamepad2.dpad_right;
            }

            if(!robotCentric){
                telemetry.addLine("akshatCentric");
            } else{
                telemetry.addLine("robotCentric");
            }

            if (scoring) { //scoring
                arm.setPos(arm.ARMPOSSCORING);
                wrist.setPos(wrist.WRISTPOSSCORING);
            } else if (pickup) { //pickup
                ls.targetPos = 0;
                arm.setPos(arm.ARMPOSPICKUP);
                wrist.setPos(wrist.WRISTPOSPICKUP);
            } else if (gamepad2.a) { //default
                arm.setPos(arm.ARMPOSDEFAULT);
                wrist.setPos(wrist.WRISTPOSDEFAULT);
                claw.claw1IsOpen = false;
                claw.claw2IsOpen = false;
                claw.updatePos();
            } else if (gamepad2.y) { //scoring
                arm.setPos(arm.ARMPOSSCORING);
                wrist.setPos(wrist.WRISTPOSCORRECT);
                claw.correctPos();
            }

            //Arm
            if(backboard){
                arm.up();
                telemetry.addData("armpos", arm.armPos);
                telemetry.update();
            } else if (down) {
                arm.down();
                telemetry.addData("armpos", arm.armPos);
                telemetry.update();
            }


            ///Wrist
            if(gamepad2.dpad_up){
                wrist.up();
                telemetry.addData("wristpos", wrist.wristPos);
                telemetry.update();
            } else if (gamepad2.dpad_down) {
                wrist.down();
                telemetry.addData("wristpos", wrist.wristPos);
                telemetry.update();
            }

            ///Airplane Launcher

            if (gamepad1.x) {
                drone.launch();
                drone.updatePos();
            }
            if (gamepad1.b) {
                drone.reset();
                drone.updatePos();
            }

            if(gamepad1.a){
                ls.ls1.setPower(1);
                ls.ls2.setPower(1);
                ls.targetPos = 3000;
                ls.ls1.setPower(0.5);
                ls.ls2.setPower(0.5);
            }

            ls.staticTick();

/*
            telemetry.addData("wristpos", wrist.wristPos);
            telemetry.addData("armpos", arm.armPos);
            telemetry.addData("dronepos: ", drone.dronePos);
            telemetry.addData("real ls1 pos: ", ls.ls1.getCurrentPosition());
            telemetry.addData("real ls2 pos(ls2 is reversed)[: ", ls.ls2.getCurrentPosition());
            telemetry.addData("theoretical ls1 pos: ", ls.targetPos);

            if(claw.claw1IsOpen) {
                telemetry.addLine("Claw 1(Right): Open");
            }else{
                telemetry.addLine("Claw 1(Right): Closed");
            }

            if(claw.claw2IsOpen) {
                telemetry.addLine("Claw 2(Left): Open");
            }else{
                telemetry.addLine("Claw 2(Left): Closed");
            }
            telemetry.update();*/

            telemetry.addData("lfd power: ", leftFrontDrive.getPower());
            telemetry.addData("rfd power: ", rightFrontDrive.getPower());
            telemetry.addData("lbd power: ", leftBackDrive.getPower());
            telemetry.addData("rbd power: ", rightBackDrive.getPower());
            telemetry.addData("ls1 power: ", ls.ls1.getPower());
            telemetry.addData("ls1 power: ", ls.ls1.getPower());
            telemetry.update();
        }
        customStop();

    }
    public void customStop() {
        super.stop();
    }
}
