package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {
    static final double GKP = 0.1;
    static final double GKI = 0.1;
    static final double GKD = 0.5;

    public void single(double targetSpins, DcMotor motor){
        double Kp = GKP;
        double Ki = GKI;
        double Kd = GKD;

        double reference = targetSpins;

        double integralSum = 0;

        double lastError = 0;

// Elapsed timer class from SDK, please use it, it's epic
        ElapsedTime timer = new ElapsedTime();

        while (reference != motor.getCurrentPosition()) {


            // obtain the encoder position
            double encoderPosition = motor.getCurrentPosition();
            // calculate the error
            double error = reference - encoderPosition;

            // rate of change of the error
            double derivative = (error - lastError) / timer.seconds();

            // sum of all error over time
            integralSum = integralSum + (error * timer.seconds());

            double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

            motor.setPower(out);

            lastError = error;

            // reset the timer for next time
            timer.reset();

        }
    }

    public void list(double[] targetSpinList, DcMotor[] motors){

        if(targetSpinList.length < 1 || targetSpinList.length != motors.length) return;

        int n = targetSpinList.length;

        double Kp = GKP;
        double Ki = GKI;
        double Kd = GKD;

        double[] integralSums = new double[n];

        double[] lastErrors = new double[n];

// Elapsed timer class from SDK, please use it, it's epic
        ElapsedTime timer = new ElapsedTime();

        while (targetSpinList[0] != motors[0].getCurrentPosition()) {

            for(int i = 0; i < n; i++) {

                // obtain the encoder position
                double encoderPosition = motors[i].getCurrentPosition();

                // calculate the error
                double error = targetSpinList[i] - encoderPosition;

                // rate of change of the error
                double derivative = (error - lastErrors[i]) / timer.seconds();

                // sum of all error over time
                integralSums[i] = integralSums[i] + (error * timer.seconds());

                double out = (Kp * error) + (Ki * integralSums[i]) + (Kd * derivative);

                motors[i].setPower(out);

                lastErrors[i] = error;

            }

            // reset the timer for next time
            timer.reset();

        }
    }
}
