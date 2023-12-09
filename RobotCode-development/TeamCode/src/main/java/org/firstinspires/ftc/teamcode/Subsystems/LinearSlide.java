package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LinearSlide {
    public DcMotor ls1;
    public DcMotor ls2;


    public LinearSlide(HardwareMap hardwareMap) {
        ls1 = hardwareMap.get(DcMotor.class, "ls1");
        ls2 = hardwareMap.get(DcMotor.class, "ls2");
        //ls1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //ls2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //ls1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //ls2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void move(double speed) {
        ls1.setPower(speed);
        ls2.setPower(speed);
    }

    public void up() {
        ls1.setDirection(DcMotor.Direction.REVERSE);
        ls2.setDirection(DcMotor.Direction.FORWARD);
        move(1);
    }

    public void down() {
        ls1.setDirection(DcMotor.Direction.FORWARD);
        ls2.setDirection(DcMotor.Direction.REVERSE);
        move(0.3);
    }

    public void stop() {
        ls1.setPower(0);
        ls2.setPower(0);
    }

    public void hold() {
        //ls1.setTargetPosition(ls1.getCurrentPosition());
        //ls2.setTargetPosition(ls2.getCurrentPosition());
        //ls1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //ls2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}

