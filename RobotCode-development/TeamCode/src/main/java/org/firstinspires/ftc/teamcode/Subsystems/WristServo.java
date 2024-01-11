package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class WristServo {

    public Servo wrist;

    public double wristPos = 0;
    public double WRISTPOSSCORING = 0.842;
    public double WRISTPOSPICKUP = 0.558;

    public WristServo(HardwareMap hardwareMap) {
        wrist = hardwareMap.get(Servo.class, "wrist");
    }

    public void up(){
            if(wristPos > 1) wristPos = 1;
            wrist.setPosition(wristPos);
            wristPos += 0.002;

    }

    public void down(){
        if(wristPos < 0) wristPos = 0;
        wrist.setPosition(wristPos);
        wristPos -= 0.002;

    }

    public void setPos(double tp){
        if(tp > 1.0) tp = 1;
        if(tp < 0.0) tp = 0;
        wristPos = tp;
        wrist.setPosition(wristPos);
    }
}