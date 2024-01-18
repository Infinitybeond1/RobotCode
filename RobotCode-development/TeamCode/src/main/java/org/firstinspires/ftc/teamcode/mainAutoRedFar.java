package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.WristServo;

import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.Subsystems.Camera;
import org.firstinspires.ftc.teamcode.Subsystems.gamepieceDetection;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Objects;
//import org.firstinspires.ftc.teamcode.Subsystems.gamepieceDetection;
/*
public class MyOpmode extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory myTrajectory = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(10)
                .forward(5)
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(myTrajectory);
    }
}
*/


@Autonomous(name="mainAutoRedFar", group="Linear OpMode")
public class mainAutoRedFar extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    int tid =0;
    Arm arm;
    Claw claw;
    LinearSlide ls;
    Camera cam;
    WristServo wrist;
    String location;
    SampleMecanumDrive drive;
    Pose2d startPose;
    ///PURPLE PIXEL GOES IN RIGHT CLAW

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Arm init
        arm = new Arm(hardwareMap);

        //Claw init
        claw = new Claw(hardwareMap);

        //Linear Slide init
        ls = new LinearSlide(hardwareMap, telemetry);

        //Drive
        drive = new SampleMecanumDrive(hardwareMap);
        // drive = new Drive(hardwareMap, telemetry);

        //Wrist
        wrist = new WristServo(hardwareMap);

        //camera
        cam = new Camera(hardwareMap, telemetry);


        startPose = new Pose2d(60, 12, Math.toRadians(180) + 1e-6);

        drive.setPoseEstimate(startPose);

        Trajectory centerTraj1 = drive.trajectoryBuilder(startPose)
                .forward(24)
                .build();
        Trajectory centerTraj2 = drive.trajectoryBuilder(centerTraj1.end(), true)
                .splineTo(new Vector2d(36,44), Math.toRadians(90))
                .build();
        Trajectory parkingTraj = drive.trajectoryBuilder(centerTraj2.end())
                .strafeLeft(16)
                .build();
        Trajectory parkingTraj2 = drive.trajectoryBuilder(parkingTraj.end())
                 .back(16)
                 .build();


        Trajectory leftTraj = drive.trajectoryBuilder(centerTraj1.end())
                  .strafeLeft(12)
                  .build();







        waitForStart();
        runtime.reset();

        if(isStopRequested()) return;
        arm.setPos(arm.ARMPOSPICKUP);
        wrist.setPos(wrist.WRISTPOSPICKUP);
        claw.updatePos();




        // PIXEL MOVE TO GAMEPIECE LINE
        sleep(0);
        location = cam.getGamepiecePos();






        if(location == "Left"){
            telemetry.addData("Status", "left");
            telemetry.update();
            tid = 4;
            drive.followTrajectory(centerTraj1);
            drive.followTrajectory(leftTraj);
            telemetry.addData("position", drive.getPoseEstimate());
            telemetry.update();



            drive.turn(Math.toRadians(90));










            /*
            drive.left(200);
            drive.forward(400);
            claw.claw1IsOpen = true;
            claw.updatePos();
            claw.claw1IsOpen = false;
            claw.updatePos();

            drive.forward(100);
            drive.turnRight(200);
            drive.forward(50);

            */
        } else if(location == "Center"){
            telemetry.addData("Status", "center");
            telemetry.update();
            tid = 5;

            drive.followTrajectory(centerTraj1);
            telemetry.addData("position", drive.getPoseEstimate());
            telemetry.update();
                                                                              /*
            drive.forward(400);


            drive.turnRight(200);
            */
        } else if(location == "Right") {
            telemetry.addData("Status", "right");
            telemetry.update();
            tid = 6;
            drive.followTrajectory(centerTraj1);
            telemetry.addData("position", drive.getPoseEstimate());
            drive.turn(Math.toRadians(-90));
            telemetry.update();                                             /*
                                                                              drive.right(200);
            drive.forward(400);
            claw.claw1IsOpen = true;
            claw.updatePos();
            claw.claw1IsOpen = false;
            claw.updatePos();

            drive.forward(100);
            drive.turnRight(200);
            drive.backward(50);
            */
        } else {
            telemetry.addData("Status", location);
            telemetry.update();
            claw.claw1IsOpen = true;
            claw.updatePos();
            claw.claw1IsOpen = false;
            claw.updatePos();
        }

        claw.claw1IsOpen = true;
        claw.updatePos();
        sleep(100);



        drive.followTrajectory(centerTraj2);
        telemetry.addData("position", drive.getPoseEstimate());
        telemetry.update();

        arm.setPos(arm.ARMPOSSCORING);
        wrist.setPos(wrist.WRISTPOSSCORING);
        claw.claw2IsOpen = true;
        claw.updatePos();
        claw.claw2IsOpen = false;
        claw.updatePos();
        arm.setPos(arm.ARMPOSPICKUP);
        drive.followTrajectory(parkingTraj);
        drive.followTrajectory(parkingTraj2);

        //SCORING ON BOARD

        /* hard code
        drive.forward(1000);
        arm.setPos(arm.ARMPOSSCORING);
        wrist.setPos(wrist.WRISTPOSSCORING);
        claw.claw2IsOpen = true;
        claw.updatePos();
        claw.claw2IsOpen = false;
        claw.updatePos();
        */
/*
        //april tag
        double[] aprilTagData = cam.updateDataAT(tid);
        while (aprilTagData != null);
     {
            //drive.left(1);
            telemetry.addData("Move:", "True");
            telemetry.update();
        }
        telemetry.addData("Move:", "False");
        telemetry.update();

        while (aprilTagData[0] < -1) {
            drive.right(1);
        }
        while (aprilTagData[2] > 10) {
            drive.forward(1);
            aprilTagData = cam.updateDataAT(tid);
        }

        arm.setPos(arm.ARMPOSSCORING);
        wrist.setPos(wrist.WRISTPOSSCORING);
        claw.claw2IsOpen = true;
        claw.updatePos();
        claw.claw2IsOpen = false;
        claw.updatePos();

        wrist.setPos(wrist.WRISTPOSPICKUP);
        arm.setPos(arm.ARMPOSPICKUP);

        //PARKING
        if (tid == 4) {
            drive.right(100);
            drive.forward(10);
        }
        if (tid == 5) {
            drive.right(80);
            drive.forward(10);
        }
        if (tid == 6) {
            drive.right(60);
            drive.forward(10);
        } */
    }
}
