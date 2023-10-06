package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor

@TeleOp(name = "Kotlin Test", group = "Linear OpMode") //@Disabled
class KtServo() : LinearOpMode()  {
    override fun runOpMode() {
        val m1 = hardwareMap.get(DcMotor::class.java, "m1")
        telemetry.addData(">", "Press start gangy")
        telemetry.update()
        waitForStart()
        while (opModeIsActive()) {
            val rt: Double = gamepad1.right_trigger.toDouble()
            val lt: Double = gamepad1.left_trigger.toDouble()

            //m1.setPower(1);
            //sleep();
            if (lt != 0.0 && rt == 0.0) {
                m1.power = lt
            } else if (rt != 0.0 && lt == 0.0) {
                m1.power = rt * -1.0 // Make the positive num position negative
            } else if (rt != 0.0 && lt != 0.0) {
                m1.power = lt - rt
            } else {
                m1.power = 0.0
            }
            telemetry.addData(">", "Left trigger $lt\nRight trigger $rt"  )
            telemetry.update()
        }
    }
}