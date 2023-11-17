package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    private static final double ARM_BACK = 0;
    private static final double ARM_MID = 0.5;
    private static final double ARM_FRONT = 1;

    public Servo a1;
    public Servo a2;

    public Arm() {
        a1 = hardwareMap.get(Servo.class, "a1");
        a2 = hardwareMap.get(Servo.class, "a2");
    }
}
