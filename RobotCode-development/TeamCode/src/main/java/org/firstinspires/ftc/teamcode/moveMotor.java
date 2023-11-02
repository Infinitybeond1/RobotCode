package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Move Motor", group = "Concept")
//@Disabled

public class moveMotor extends LinearOpMode{

    private DcMotor m1 = null;

    public void runOpMode(){
        m1 = hardwareMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, "m1");

        telemetry.addData(">", "Press start gangy" );
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double leftY1 = gamepad1.left_stick_y;
            double rightY1 = gamepad1.right_stick_y;
            double leftX1 = gamepad1.left_stick_x;
            double rightX1 = gamepad1.right_stick_x;
            double leftY2 = gamepad2.left_stick_y;
            double rightY2 = gamepad2.right_stick_y;
            double leftX2 = gamepad2.left_stick_x;
            double rightX2 = gamepad2.right_stick_x;
            boolean a_down = gamepad1.a;
            boolean b_down = gamepad1.b;

            //m1.setPower(1);
            //sleep();

            if (a_down) {
                m1.setPower(rightY1);
            }else if (b_down) {
                m1.setPower(-.5);
            } else {m1.setPower(0);}

            telemetry.addData(">", a_down + " left: " + leftX2 + ", " + leftY2 + "  ||| right: " + rightX2 + ", " + rightY2 );
            telemetry.addData("Motor Power:", m1.getPower()); //Motor Power: 0.5
            telemetry.update();
        }

    }
}
