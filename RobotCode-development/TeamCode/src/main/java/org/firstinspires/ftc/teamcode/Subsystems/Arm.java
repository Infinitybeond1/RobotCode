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
    public double ARMPOSSCORING = 0.088;
    public double ARMPOSPICKUP = 0.822;

    public Arm(HardwareMap hardwareMap) {
        a1r = hardwareMap.get(Servo.class, "a1r");
        a2l = hardwareMap.get(Servo.class, "a2l");
    }

    public void up(){
        a1r.setPosition(1-armPos);
        a2l.setPosition(armPos);
        armPos -= 0.002;
        if(armPos < 0) armPos = 0;

    }

    public void down(){
        a1r.setPosition(1-armPos);
        a2l.setPosition(armPos);
        armPos += 0.002;
        if(armPos > 1) armPos = 1;
    }

    public void setPos(double tp){
        if(tp > 1.0) tp = 1;
        if(tp < 0.0) tp = 0;
        armPos = tp;
        a1r.setPosition(1-armPos);
        a2l.setPosition(armPos);

    }

}
