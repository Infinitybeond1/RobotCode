package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Subsystems.LinearSlide

@TeleOp(name = "LinearSlideTest", group = "Linear OpMode") //@Disabled
class LsTest : LinearOpMode() {
    override fun runOpMode() {
        //Linear Slide init
        val ls = LinearSlide(hardwareMap, telemetry);

        waitForStart()

        while (opModeIsActive()) {
            /// Linear Slide
            if (gamepad2.dpad_up) {
                ls.up()
            } else if (gamepad2.dpad_down) {
                ls.down()
            } else {
                ls.stop()
            }
        }
    }

}