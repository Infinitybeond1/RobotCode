package org.firstinspires.ftc.teamcode.Subsystems;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
//import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Drone {
    public static volatile double STARTPOS_D = 0.648;
    public static volatile double LAUNCHPOS_D = 1;

    public Servo d1;

    public Drone(HardwareMap hardwareMap) {
        d1 = hardwareMap.get(Servo.class, "drone");
    }

    public void launch(){
        d1.setPosition(LAUNCHPOS_D);
    }

    public void reset() {
        d1.setPosition(STARTPOS_D);
    }
}
