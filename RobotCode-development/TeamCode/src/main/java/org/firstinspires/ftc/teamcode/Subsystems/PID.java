package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PID {

    Telemetry telemetry;
    private ElapsedTime runtime = new ElapsedTime();

    public PID(Telemetry telemetry1){
        telemetry = telemetry1;
    }

    //PID.move takes 4 doubles describing, in order:
    //the target position, the proportional modifier, integral modifier and derivative modifier
    //lastly, it takes a variable number of DcMotor objects, the first motor of this being the motor that
    //must have an encoder, and that PID calculations will be used on
    //if the integral and derivative terms are 0, a separate loop is executed which leaves out
    //variables and calculations to save on runtime and memory
    //PID.move directly sets the power of the parameter motors in order to reach the target position

    public void tick(double reference, double Kp, DcMotor ... motors){
        if(motors.length < 1){
            return;
        }

        double encoderPosition = motors[1].getCurrentPosition();
        // calculate the error
        double error = reference - encoderPosition;

        for(DcMotor motor : motors) motor.setPower(Kp * error);

    }


    public void move(double reference, double Kp, double Ki, double Kd, DcMotor ... motors){

        if(motors.length < 1){
            return;
        }

        if(Ki == 0 && Kd == 0){
            while (reference != motors[1].getCurrentPosition()) {


                // obtain the encoder position
                double encoderPosition = motors[1].getCurrentPosition();
                // calculate the error
                double error = reference - encoderPosition;

                for(DcMotor motor : motors) motor.setPower(Kp * error);

                telemetry.addData("error: ", error);
                telemetry.update();
            }

            telemetry.addLine("done");
            return;
        }

        double integralSum = 0;

        double lastError = 0;

// Elapsed timer class from SDK, please use it, it's epic
        ElapsedTime timer = new ElapsedTime();


        while (reference != motors[1].getCurrentPosition()) {


            // obtain the encoder position
            double encoderPosition = motors[1].getCurrentPosition();
            // calculate the error
            double error = reference - encoderPosition;

            // rate of change of the error
            double derivative = (error - lastError) / timer.seconds();

            // sum of all error over time
            integralSum = integralSum + (error * timer.seconds());

            double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

            for(DcMotor motor : motors) motor.setPower(out);

            telemetry.addData("error: ", error);
            telemetry.addData(", integral sum: ", integralSum);
            telemetry.addData(", out: ", out);
            telemetry.update();

            lastError = error;
            // reset the timer for next time
            timer.reset();

        }

    }

    public void turn(){

    }

}

