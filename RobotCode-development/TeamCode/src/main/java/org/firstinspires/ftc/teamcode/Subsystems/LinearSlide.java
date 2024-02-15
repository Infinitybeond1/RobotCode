package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LinearSlide {
    public DcMotor ls1;
    public DcMotor ls2;
    public double startPos1;
    public double startPos2;
    public int UPPER_LIMIT = 100;//random number
    public int LOWER_LIMIT = 0;
    public double speed = 0;
    public double targetPos = 0;
    public Telemetry telemetry;
    PID pid;


    public LinearSlide(HardwareMap hardwareMap, Telemetry telemetry1) {
        telemetry = telemetry1;
        ls1 = hardwareMap.get(DcMotor.class, "ls1");
        ls2 = hardwareMap.get(DcMotor.class, "ls2");
        ls1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ls2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ls1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ls2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ls1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ls2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ls1.setDirection(DcMotor.Direction.FORWARD);
        ls2.setDirection(DcMotor.Direction.REVERSE);
        pid = new PID(telemetry);


    }

    public void staticTick(){
        if(targetPos > 3000) targetPos = 3000;
        if(targetPos < 10) targetPos = 10;

        pid.tick(targetPos, 0.006, ls1, ls2);
    }



    public void movePID(double targetPos) {
        pid.move(targetPos, 0.001, 0, 0, ls1, ls2);
        telemetry.addData("ls1: ", ls1.getPower());
        telemetry.addData("ls2: ", ls2.getPower());
        telemetry.update();
    }

   /*private void move(double targetPos) {
            ls1.setPower(speed);
            ls2.setPower(speed);
    }*/

    public void PIDup() {
        targetPos += 600;
        if(ls1.getCurrentPosition() > 3000 || ls2.getCurrentPosition() > 3000) targetPos = 3000;

        ls1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ls2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        movePID(targetPos);
    }

    public void PIDdown() {
        targetPos -= 600;
        if(ls1.getCurrentPosition() < 0 || ls2.getCurrentPosition() < 0) targetPos = 0;

        ls1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ls2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        movePID(targetPos);
    }

    public void up() {

        targetPos+=600;

        ls1.setTargetPosition(ls1.getCurrentPosition()+600);
        ls2.setTargetPosition(ls1.getCurrentPosition()+600);

        if(ls1.getTargetPosition() > 3000 || ls2.getTargetPosition() > 3000) {
            targetPos = 3000;
            ls1.setTargetPosition(3000);
            ls2.setTargetPosition(3000);
        }

        ls1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ls2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ls1.setTargetPosition((int)targetPos);
        ls1.setPower(0.5);
        ls2.setPower(0.5);

        while(ls1.isBusy() || ls2.isBusy());
    }

    public void down() {

        targetPos-=600;

        ls1.setTargetPosition(ls1.getCurrentPosition()-600);
        ls2.setTargetPosition(ls1.getCurrentPosition()-600);

        if(ls1.getTargetPosition() < 0 || ls2.getTargetPosition() < 0) {
            targetPos = 0;
            ls1.setTargetPosition(0);
            ls2.setTargetPosition(0);
        }

        ls1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ls2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        ls1.setPower(0.5);
        ls2.setPower(0.5);

        while(ls1.isBusy() || ls2.isBusy());
    }

    public void stop() {
        ls1.setDirection(DcMotor.Direction.FORWARD);
        ls2.setDirection(DcMotor.Direction.REVERSE);
        //move(0);
    }


    public void hold() {
        //ls1.setTargetPosition(ls1.getCurrentPosition());
        //ls2.setTargetPosition(ls2.getCurrentPosition());
        //ls1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //ls2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}

