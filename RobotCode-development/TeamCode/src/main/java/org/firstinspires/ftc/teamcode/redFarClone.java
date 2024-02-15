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

@Autonomous(name="redFarClone", group="Linear OpMode")
public class redFarClone extends LinearOpMode {
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
    ///PURPLE PIXEL GOES IN RIGHT CLAW, YELLOW IN LEFT

    @Override
    public void runOpMode() throws InterruptedException  {
        while (opModeIsActive()){
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


        startPose = new Pose2d(-36, -60, Math.toRadians(90));

        drive.setPoseEstimate(startPose);



        TrajectorySequence centerTraj = drive.trajectorySequenceBuilder(startPose)
                .forward(24)
                .build();



        TrajectorySequence centerTraj2 = drive.trajectorySequenceBuilder(centerTraj.end())
                .back(3)
                .turn(Math.toRadians(90))
                .strafeRight(4.5)
                .back(84)
                .strafeRight(2)
                .build();

        TrajectorySequence centerTraj3 = drive.trajectorySequenceBuilder(centerTraj2.end())
                .back(6)
                .build();


        TrajectorySequence rightTraj = drive.trajectorySequenceBuilder(startPose)
                .forward(26)
                .turn(Math.toRadians(90))
                .back(25)
                .build();



        TrajectorySequence rightTraj2 = drive.trajectorySequenceBuilder(rightTraj.end())
                .back(55)
                .build();

        TrajectorySequence rightTraj3 = drive.trajectorySequenceBuilder(rightTraj2.end())
                .back(5)
                .build();

        TrajectorySequence leftTraj = drive.trajectorySequenceBuilder(startPose)
                .forward(26)
                .turn(Math.toRadians(90))
                .build();



        TrajectorySequence leftTraj2 = drive.trajectorySequenceBuilder(leftTraj.end())
                .back(84)
                .strafeRight(5)
                .build();

        TrajectorySequence leftTraj3 = drive.trajectorySequenceBuilder(leftTraj2.end())
                .back(10)
                .build();

        TrajectorySequence parkingTrajL = drive.trajectorySequenceBuilder(leftTraj2.end())
                .strafeLeft(20)
                .build();


        TrajectorySequence parkingTrajC = drive.trajectorySequenceBuilder(centerTraj2.end())
                .strafeLeft(17)
                .build();


        TrajectorySequence parkingTrajR = drive.trajectorySequenceBuilder(rightTraj2.end())
                .strafeLeft(15)
                .build();


        cam.propDetector.propColor = gamepieceDetection.Prop.RED;


        waitForStart();
        runtime.reset();

        if(isStopRequested()) return;
        arm.setPos(arm.ARMPOSPICKUP);
        wrist.setPos(wrist.WRISTPOSPICKUP);
        claw.claw2IsOpen = false;
        claw.claw1IsOpen = false;
        claw.updatePos();
        sleep(1000); //need



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
            arm.setPos(0.75);
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
            arm.setPos(arm.ARMPOSPICKUP);
            sleep(1000);

            drive.followTrajectorySequence(parkingTrajL);

        } else if(location == "Center"){

            telemetry.addData("Status", "center");
            telemetry.update();
            drive.followTrajectorySequence(centerTraj);
            claw.claw1IsOpen = true;
            claw.updatePos();
            sleep(500);
            arm.setPos(0.75);
            claw.claw1IsOpen = false;
            claw.updatePos();
            drive.followTrajectorySequence(centerTraj2);
            ls.up();
            sleep(500);
            arm.setPos(arm.ARMPOSSCORING);
            wrist.setPos(wrist.WRISTPOSSCORING);

            sleep(2000);
            drive.followTrajectorySequence(centerTraj3);
            claw.claw2IsOpen = true;
            claw.updatePos();
            sleep(1000);
            arm.setPos(arm.ARMPOSPICKUP);
            sleep(1000);
            drive.followTrajectorySequence(parkingTrajC);


        } else if(location == "Right") {
            telemetry.addData("Status", "right");
            telemetry.update();
            drive.followTrajectorySequence(rightTraj);
            claw.claw1IsOpen = true;
            claw.updatePos();
            sleep(500);
            arm.setPos(0.75);
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
            arm.setPos(arm.ARMPOSPICKUP);
            sleep(1000);

            drive.followTrajectorySequence(parkingTrajR);


        } else {
            telemetry.addData("Status", "No gamepiece detected");
        }
        ls.down();
        sleep(500);
        arm.setPos(arm.ARMPOSPICKUP);
        sleep(200);
        wrist.setPos(wrist.WRISTPOSPICKUP);
        sleep(200);
        claw.claw1IsOpen = false;
        claw.claw2IsOpen = false;
        claw.updatePos();

        telemetry.addData("position", drive.getPoseEstimate());
        telemetry.update();

    }}
}
*/
