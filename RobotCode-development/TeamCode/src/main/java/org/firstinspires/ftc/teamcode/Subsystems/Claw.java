package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Claw {
    private double CLAWPOSOPEN = 0.35;
    private double CLAWPOSCLOSED = 0.6;
    private double CLAWPOSCORRECT = 0;

    public boolean claw1IsOpen = false;
    public boolean claw2IsOpen = false;

    public Servo claw1;
    public Servo claw2;

    public Claw(HardwareMap hardwareMap) {
        claw1 = hardwareMap.get(Servo.class, "clawR");
        claw2 = hardwareMap.get(Servo.class, "clawL");
    }

    public void updatePos(){

        if(claw1IsOpen) claw1.setPosition(CLAWPOSOPEN);
        else claw1.setPosition(CLAWPOSCLOSED);

        if(claw2IsOpen) claw2.setPosition(1-CLAWPOSOPEN);
        else claw2.setPosition(1-CLAWPOSCLOSED);
    }
    public void correctPos(){
        claw1.setPosition(CLAWPOSCORRECT);
        claw2.setPosition(1-CLAWPOSCORRECT);
    }
}
