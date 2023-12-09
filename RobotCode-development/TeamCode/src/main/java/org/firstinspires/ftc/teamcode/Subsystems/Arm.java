package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    /*private static final double ARM_BACK = 0;
    private static final double ARM_MID = 0.5;
    private static final double ARM_FRONT = 1;
*/
    public Servo a1r;
    public Servo a2l;

    public double armPos = 0;

    public Arm(HardwareMap hardwareMap) {
        a1r = hardwareMap.get(Servo.class, "a1r");
        a2l = hardwareMap.get(Servo.class, "a2l");
    }

    public void up(){
        if (armPos > 0.1) {
            a1r.setPosition(armPos);
            a2l.setPosition(1 - armPos);
              armPos -= 0.005;
       }

    }

    public void down(){
       if (armPos < 0.9) {
           a1r.setPosition(armPos);
           a2l.setPosition(1 - armPos);
           armPos += 0.005;
       }
    }
/*
    public void left(){
        // if (armPos < 0.5) {
        //    a1r.setPosition(armPos);
        //a2l.setPosition(1 - armPos);
        //    armPos -= 0.005;
        // }
        //armPos -= 0.01;
        a2l.setPosition(1);
    }

    public void right(){
        // if (armPos < 0.5) {
        //    a1r.setPosition(armPos);
        //a2l.setPosition(1 - armPos);
        //    armPos -= 0.005;
        // }
        //armPos -= 0.01;
        a2l.setPosition(0);
    }*/
}
