package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;

@Autonomous(name="Time based auto blue", group="Linear OpMode")
public class AutoPixelCloseBlue extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    public static volatile double ARM_MID = 0.5;
    public static volatile double ARM_DOWN = 0.1;
    Arm arm;

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        runtime.reset();

        arm = new Arm(hardwareMap);

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

            while(arm.armPos > 0.1){
                arm.up();
            }

            sleep(13000);
            rightFrontDrive.setPower(-0.5);
            leftFrontDrive.setPower(-0.5);
            rightBackDrive.setPower(0.5);
            leftBackDrive.setPower(0.5);
            sleep(300);
            rightFrontDrive.setPower(0);
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);

            rightFrontDrive.setPower(0.5);
            leftFrontDrive.setPower(-0.5);
            rightBackDrive.setPower(0.5);
            leftBackDrive.setPower(-0.5);
            sleep(7000);
            leftFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
        }
    }
}
