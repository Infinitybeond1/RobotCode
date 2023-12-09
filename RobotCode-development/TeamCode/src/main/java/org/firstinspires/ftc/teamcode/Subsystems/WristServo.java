package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class WristServo {

    public Servo wrist;

    public double wristPos = 0;

    public WristServo(HardwareMap hardwareMap) {
        wrist = hardwareMap.get(Servo.class, "wrist");
    }

    public void up(){
        if (wristPos < .9) {
            wrist.setPosition(wristPos);
            wristPos += 0.005;
        }
    }

    public void down(){
        if (wristPos > 0.01) {
            wrist.setPosition(wristPos);
            wristPos -= 0.005;
        }
    }
}