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
                ls.ls1.power = 0.2;
                ls.ls2.power = 0.2;
            } else if (gamepad2.dpad_down) {
                ls.ls1.power = -0.2;
                ls.ls2.power = -0.2;
            } else {
                ls.ls1.power = 0.0;
                ls.ls2.power = 0.0;
            }
        }
    }

}