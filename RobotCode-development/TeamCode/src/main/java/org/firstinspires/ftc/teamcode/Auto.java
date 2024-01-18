package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
@Disabled
@Autonomous(name="Auto", group="Linear OpMode")
public class Auto extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    public static volatile double ARM_MID = 0.5;
    public static volatile double ARM_DOWN = 0.1;
    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            /// INIT
            // Initialize the hardware variables. Note that the strings used here must correspond
            // to the names assigned during the robot configuration step on the DS or RC devices.
            DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "lfd");
            DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "lbd");
            DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "rfd");
            DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "rbd");


            //Set modes
            leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //Arm init
            com.qualcomm.robotcore.hardware.Servo a1 = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "a1");
            com.qualcomm.robotcore.hardware.Servo a2 = hardwareMap.get(Servo.class, "a2");

            //a2.setDirection(Servo.Direction.REVERSE);

            a1.setPosition(ARM_DOWN);
            a2.setPosition(1-ARM_DOWN);


            //bot goe forward
            leftFrontDrive.setPower(0.25);
            leftBackDrive.setPower(0.25);
            rightFrontDrive.setPower(0.25);
            rightBackDrive.setPower(0.25);
            sleep(800);
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);

            //bot drops pixel(make sure to raise arm or something so pixel doesnt catch)
            //a1.setPosition(ARM_MID);
            //a2.setPosition(ARM_MID);
            /*
            //bot goes a bit more forward
            leftFrontDrive.setPower(0.5);
            leftBackDrive.setPower(0.5);
            rightFrontDrive.setPower(0.5);
            rightBackDrive.setPower(0.5);
            sleep(2000);

            //bot turns 90 degrees
            leftFrontDrive.setPower(0.5);
            leftBackDrive.setPower(0.5);
            rightFrontDrive.setPower(-0.5);
            rightBackDrive.setPower(-0.5);
            sleep(1000);

            //bot drives forward more
            leftFrontDrive.setPower(0.5);
            leftBackDrive.setPower(0.5);
            rightFrontDrive.setPower(0.5);
            rightBackDrive.setPower(0.5);
            sleep(2000);
            //bot turns 90
            leftFrontDrive.setPower(0.5);
            leftBackDrive.setPower(0.5);
            rightFrontDrive.setPower(-0.5);
            rightBackDrive.setPower(-0.5);
            sleep(1000);
            //forward until bot is in front of parking space
            leftFrontDrive.setPower(0.5);
            leftBackDrive.setPower(0.5);
            rightFrontDrive.setPower(0.5);
            rightBackDrive.setPower(0.5);
            sleep(2000);
            //turn back 90 degrees
            leftFrontDrive.setPower(-0.5);
            leftBackDrive.setPower(-0.5);
            rightFrontDrive.setPower(0.5);
            rightBackDrive.setPower(0.5);
            sleep(1000);
            //bot drives into the wall
            leftFrontDrive.setPower(0.5);
            leftBackDrive.setPower(0.5);
            rightFrontDrive.setPower(0.5);
            rightBackDrive.setPower(0.5);
            sleep(1000);
            */
        }
    }
}
