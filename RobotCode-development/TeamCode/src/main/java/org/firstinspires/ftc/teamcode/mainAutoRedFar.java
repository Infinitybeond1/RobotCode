package org.firstinspires.ftc.teamcode;

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


@Autonomous(name="mainAutoRedFar", group="Linear OpMode")
public class mainAutoRedFar extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private gamepieceDetection propDetector;
    int tid;
    Arm arm;
    Claw claw;
    LinearSlide ls;
    Drive drive;
    Camera cam;
    WristServo wrist;


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
        drive = new Drive(hardwareMap, telemetry);

        //Wrist
        wrist = new WristServo(hardwareMap);

        //camera
        cam = new Camera(hardwareMap, telemetry);
        propDetector.propColor = gamepieceDetection.Prop.RED;


        waitForStart();
        runtime.reset();

        arm.setPos(arm.ARMPOSPICKUP);
        wrist.setPos(wrist.WRISTPOSPICKUP);
        claw.updatePos();

        cam.visionPortal.getProcessorEnabled(propDetector);

        // PIXEL MOVE TO GAMEPIECE LINE
        /*
        if(cam.getGamepiecePos() == "left"){
            tid = 4;

            drive.left(200);
            drive.forward(400);
            claw.claw1IsOpen = true;
            claw.updatePos();
            claw.claw1IsOpen = false;
            claw.updatePos();

            drive.forward(100);
            drive.turnRight(200);
            drive.forward(50);


        }

        if(cam.getGamepiecePos() == "center"){
            tid = 5;
            drive.forward(400);
            claw.claw1IsOpen = true;
            claw.updatePos();
            claw.claw1IsOpen = false;
            claw.updatePos();

            drive.turnRight(200);

        }

        if(cam.getGamepiecePos() == "right"){
            tid = 6;
            drive.right(200);
            drive.forward(400);
            claw.claw1IsOpen = true;
            claw.updatePos();
            claw.claw1IsOpen = false;
            claw.updatePos();

            drive.forward(100);
            drive.turnRight(200);
            drive.backward(50);

        }*/

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

        //april tag
        double[] aprilTagData = cam.updateDataAT(tid);
        while (aprilTagData[0] > 1) {
            //drive.left(1);
            telemetry.addData("Move:", "True");
            telemetry.update();
        }
        telemetry.addData("Move:", "False");
        telemetry.update();
/*
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
