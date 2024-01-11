package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.LinearSlide;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.PID;


    @Autonomous(name="PIDTestAuto", group="Linear OpMode")
//@Disabled
    public class PIDTestAuto extends LinearOpMode {


        // Declare OpMode members.
        private ElapsedTime runtime = new ElapsedTime();
        LinearSlide ls;
        PID pid;


        @Override
        public void runOpMode() {
            telemetry.addData("Status", "Initialized");
            telemetry.update();

            waitForStart();
            runtime.reset();

            //Linear Slide init
            ls = new LinearSlide(hardwareMap, telemetry);

            ls.movePID(1500);

            sleep(5000);

        }
    }
