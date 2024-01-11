package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.PID;

public class Drive {
    DcMotor lfd;
    DcMotor lbd;
    DcMotor rbd;
    DcMotor rfd;
    IMU imu;
    public double speed = 0.5;

    public Drive(HardwareMap hardwareMap, Telemetry telemetry) {
        lfd = hardwareMap.get(DcMotor.class, "lfd");
        lbd = hardwareMap.get(DcMotor.class, "lbd");
        rbd = hardwareMap.get(DcMotor.class, "rbd");
        rfd = hardwareMap.get(DcMotor.class, "rfd");

        lfd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lbd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rfd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbd.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lfd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lbd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rfd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rbd.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lfd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lbd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rfd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rbd.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lfd.setDirection(DcMotor.Direction.REVERSE);
        lbd.setDirection(DcMotor.Direction.REVERSE);
        rfd.setDirection(DcMotor.Direction.FORWARD);
        rbd.setDirection(DcMotor.Direction.FORWARD);

    }



    public void forward(int pos) {
       lfd.setTargetPosition(lfd.getCurrentPosition()+pos);
       lbd.setTargetPosition(lbd.getCurrentPosition()+pos);
       rfd.setTargetPosition(rfd.getCurrentPosition()+pos);
       rbd.setTargetPosition(rbd.getCurrentPosition()+pos);
       toPos();
       setSpeed();
       while(
               lfd.isBusy() || lbd.isBusy() || rfd.isBusy() || rbd.isBusy()
       );

    }

    public void backward(int pos) {
        lfd.setTargetPosition(lfd.getCurrentPosition()-pos);
        lbd.setTargetPosition(lbd.getCurrentPosition()-pos);
        rfd.setTargetPosition(rfd.getCurrentPosition()-pos);
        rbd.setTargetPosition(rbd.getCurrentPosition()-pos);
        toPos();
        setSpeed();
        while(
                lfd.isBusy() || lbd.isBusy() || rfd.isBusy() || rbd.isBusy()
        );

    }

    public void right(int pos) {
        lfd.setTargetPosition(lfd.getCurrentPosition()+pos);
        lbd.setTargetPosition(lbd.getCurrentPosition()-pos);
        rfd.setTargetPosition(rfd.getCurrentPosition()-pos);
        rbd.setTargetPosition(rbd.getCurrentPosition()+pos);
        toPos();
        setSpeed();
        while(
                lfd.isBusy() || lbd.isBusy() || rfd.isBusy() || rbd.isBusy()
        );

    }

    public void left(int pos) {
        lfd.setTargetPosition(lfd.getCurrentPosition()-pos);
        lbd.setTargetPosition(lbd.getCurrentPosition()+pos);
        rfd.setTargetPosition(rfd.getCurrentPosition()+pos);
        rbd.setTargetPosition(rbd.getCurrentPosition()-pos);
        toPos();
        setSpeed();
        while(
                lfd.isBusy() || lbd.isBusy() || rfd.isBusy() || rbd.isBusy()
        );

    }

    public void turnRight(int pos){
        lfd.setTargetPosition(lfd.getCurrentPosition()+pos);
        lbd.setTargetPosition(lbd.getCurrentPosition()+pos);
        rfd.setTargetPosition(rfd.getCurrentPosition()-pos);
        rbd.setTargetPosition(rbd.getCurrentPosition()-pos);
        toPos();
        setSpeed();
        while(
                lfd.isBusy() || lbd.isBusy() || rfd.isBusy() || rbd.isBusy()
        );


    }

    public void turnLeft(int pos){
        lfd.setTargetPosition(lfd.getCurrentPosition()-pos);
        lbd.setTargetPosition(lbd.getCurrentPosition()-pos);
        rfd.setTargetPosition(rfd.getCurrentPosition()+pos);
        rbd.setTargetPosition(rbd.getCurrentPosition()+pos);
        toPos();
        setSpeed();
        while(
                lfd.isBusy() || lbd.isBusy() || rfd.isBusy() || rbd.isBusy()
        );
    }

    public void setSpeed(){
        lfd.setPower(speed);
        lbd.setPower(speed);
        rfd.setPower(speed);
        rbd.setPower(speed);
    }

    private void toPos(){
        lfd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lbd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rfd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rbd.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}
