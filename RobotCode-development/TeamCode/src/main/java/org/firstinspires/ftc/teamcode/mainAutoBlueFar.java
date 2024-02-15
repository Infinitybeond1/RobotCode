/*
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Objects;



@Autonomous(name="mainAutoBlueFar", group="Linear OpMode")
public class mainAutoBlueFar extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
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
    public void runOpMode() throws InterruptedException  {
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

        //Wrist
        wrist = new WristServo(hardwareMap);

        //camera
        cam = new Camera(hardwareMap, telemetry);


        startPose = new Pose2d(-36, 60, Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        */
/*    NEAR AUTO
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


        Trajectory leftTraj1 = drive.trajectoryBuilder(startPose, true)
                  .splineTo(new Vector2d(36,30), Math.toRadians(90))
                  .build();


        *//*


        TrajectorySequence centerTraj = drive.trajectorySequenceBuilder(startPose)
                .forward(26)
                .build();



        TrajectorySequence centerTraj2 = drive.trajectorySequenceBuilder(centerTraj.end())
                .back(3)
                .turn(Math.toRadians(-90))
                .strafeLeft(4)
                .back(84)
                .build();

        TrajectorySequence centerTraj3 = drive.trajectorySequenceBuilder(centerTraj2.end())
                .back(5)
                .build();


        TrajectorySequence leftTraj = drive.trajectorySequenceBuilder(startPose)
                .forward(27.5)
                .turn(Math.toRadians(-90))
                .back(26)
                .build();



        TrajectorySequence leftTraj2 = drive.trajectorySequenceBuilder(leftTraj.end())
                .back(56)
                .strafeRight(2)
                .build();

        TrajectorySequence leftTraj3 = drive.trajectorySequenceBuilder(leftTraj2.end())
                .back(9)
                .build();

        TrajectorySequence rightTraj = drive.trajectorySequenceBuilder(startPose)
                .forward(26)
                .turn(Math.toRadians(-90))
                .back(4)
                .build();



        TrajectorySequence rightTraj2 = drive.trajectorySequenceBuilder(rightTraj.end())
                .back(82)
                .strafeLeft(2)
                .build();

        TrajectorySequence rightTraj3 = drive.trajectorySequenceBuilder(rightTraj2.end())
                .back(5)
                .build();

        TrajectorySequence parkingTrajL = drive.trajectorySequenceBuilder(leftTraj2.end())
                .strafeRight(13)
                .build();


        TrajectorySequence parkingTrajC = drive.trajectorySequenceBuilder(centerTraj2.end())

                .strafeRight(15)
                .build();


        TrajectorySequence parkingTrajR = drive.trajectorySequenceBuilder(rightTraj2.end())
                .strafeRight(17)
                .build();


        cam.propDetector.propColor = gamepieceDetection.Prop.BLUE;


        waitForStart();
        runtime.reset();

        if(isStopRequested()) return;
        arm.setPos(arm.ARMPOSPICKUP);
        wrist.setPos(wrist.WRISTPOSPICKUP);
        claw.claw2IsOpen = false;
        claw.claw1IsOpen = false;
        claw.updatePos();


        sleep(0);

        // PIXEL MOVE TO GAMEPIECE LINE
        for (int x=0; x < 10; x++) {
            location = cam.getGamepiecePos();
        }



        if(location == "Left"){

            telemetry.addData("Status", "left");
            telemetry.update();

            drive.followTrajectorySequence(leftTraj);
            claw.claw1IsOpen = true;
            claw.updatePos();
            sleep(500);
            arm.setPos(arm.ARMPOSDEFAULT);
            claw.claw1IsOpen = false;
            claw.updatePos();

            drive.followTrajectorySequence(leftTraj2);
            ls.up();
            sleep(500);
            arm.setPos(arm.ARMPOSSCORING);
            wrist.setPos(wrist.WRISTPOSSCORING);

            sleep(2000);
            drive.followTrajectorySequence(leftTraj3);
            claw.claw2IsOpen = true;
            claw.updatePos();
            sleep(500);
            arm.setPos(arm.ARMPOSDEFAULT                                                  );
            sleep(1000);

            drive.followTrajectorySequence(parkingTrajL);

        } else if(location == "Center"){

            telemetry.addData("Status", "center");
            telemetry.update();
            drive.followTrajectorySequence(centerTraj);
            claw.claw1IsOpen = true;
            claw.updatePos();
            sleep(500);
            arm.setPos(arm.ARMPOSDEFAULT);
            claw.claw1IsOpen = false;
            claw.updatePos();

            wrist.setPos(wrist.WRISTPOSSCORING);
            drive.followTrajectorySequence(centerTraj2);
            ls.up();
            sleep(500);
            arm.setPos(arm.ARMPOSSCORING);
            wrist.setPos(wrist.WRISTPOSSCORING);

            sleep(2000);
            drive.followTrajectorySequence(centerTraj3);
            claw.claw2IsOpen = true;
            claw.updatePos();
            sleep(500);
            arm.setPos(arm.ARMPOSDEFAULT);
            sleep(1000);
            drive.followTrajectorySequence(parkingTrajC);


        } else if(location == "Right") {
            telemetry.addData("Status", "right");
            telemetry.update();
            drive.followTrajectorySequence(rightTraj);
            claw.claw1IsOpen = true;
            claw.updatePos();
            sleep(500);
            arm.setPos(arm.ARMPOSDEFAULT);
            claw.claw1IsOpen = false;
            claw.updatePos();

            drive.followTrajectorySequence(rightTraj2);
            ls.up();
            sleep(500);
            arm.setPos(arm.ARMPOSSCORING);
            wrist.setPos(wrist.WRISTPOSSCORING);

            sleep(2000);
            drive.followTrajectorySequence(rightTraj3);
            claw.claw2IsOpen = true;
            claw.updatePos();
            sleep(500);
            arm.setPos(arm.ARMPOSDEFAULT);
            sleep(1000);

            drive.followTrajectorySequence(parkingTrajR);


        } else {
            telemetry.addData("Status", "No gamepiece detected");
        }

        arm.setPos(arm.ARMPOSPICKUP);
        sleep(200);
        wrist.setPos(wrist.WRISTPOSPICKUP);
        sleep(200);
        ls.down();
        sleep(500);
        claw.claw1IsOpen = false;
        claw.claw2IsOpen = false;
        claw.updatePos();
        telemetry.addData("position", drive.getPoseEstimate());
        telemetry.update();

    }
}
*/
