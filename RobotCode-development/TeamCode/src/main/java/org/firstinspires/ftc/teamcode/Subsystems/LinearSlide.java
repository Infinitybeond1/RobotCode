package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;

public class LinearSlide {
    public DcMotor ls1;
    public DcMotor ls2;

    public LinearSlide() {
        ls1 = hardwareMap.get(DcMotor.class, "ls1");
        ls2 = hardwareMap.get(DcMotor.class, "ls2");
        ls1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ls2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void move() {
        ls1.setPower(0.5);
        ls2.setPower(0.5);
    }

    public void up() {
        ls1.setDirection(DcMotor.Direction.FORWARD);
        ls2.setDirection(DcMotor.Direction.REVERSE);
        move();
    }

    public void down() {
        ls1.setDirection(DcMotor.Direction.REVERSE);
        ls2.setDirection(DcMotor.Direction.FORWARD);
        move();
    }

    public void stop() {
        ls1.setPower(0);
        ls2.setPower(0);
    }
}
