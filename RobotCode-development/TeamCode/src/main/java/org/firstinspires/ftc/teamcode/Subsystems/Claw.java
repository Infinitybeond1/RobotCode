package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    private static final double CLAW_CLOSE = .635;
    private static final double CLAW_OPEN = .427;

    public Servo claw;

    public Claw() {
        claw = hardwareMap.get(Servo.class, "claw");
    }

    public void open() {
        claw.setPosition(CLAW_OPEN);
    }

    public void close() {
        claw.setPosition(CLAW_CLOSE);
    }
}
