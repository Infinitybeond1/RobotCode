package org.firstinspires.ftc.teamcode.Subsystems;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
//import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Drone {
    public static volatile double STARTPOS_D = 0.615;
    public static volatile double LAUNCHPOS_D = 0.270;
    public double dronePos = 0;

    public Servo d1;

    public Drone(HardwareMap hardwareMap) {

        d1 = hardwareMap.get(Servo.class, "drone");

    }

    public void launch(){
        dronePos =  LAUNCHPOS_D;
        d1.setPosition(LAUNCHPOS_D);
    }

    public void reset() {
        dronePos =  STARTPOS_D;
        d1.setPosition(STARTPOS_D);
    }

    public void updatePos(){
        if(dronePos > 1)dronePos = 1;
        if(dronePos < 0)dronePos = 0;
        d1.setPosition(dronePos);
    }

}
