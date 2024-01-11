package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class control {
    static final double GKP = 0.1;
    static final double GKI = 0.1;
    static final double GKD = 0.5;
    static final double maxA = 0.5;
    static final double maxV = 1;

    public control(){

    }

    double motion_profile(double max_acceleration, double max_velocity, double distance, double elapsed_time) {

        //Return the current reference position based on the given motion profile times, maximum acceleration, velocity, and current time.


        // calculate the time it takes to accelerate to max velocity
        double acceleration_dt = max_velocity / max_acceleration;

        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
        double halfway_distance = distance / 2;
        double acceleration_distance = 0.5 * max_acceleration * acceleration_dt * acceleration_dt;

        if (acceleration_distance > halfway_distance)
            acceleration_dt = Math.sqrt(halfway_distance / (0.5 * max_acceleration));

        acceleration_distance = 0.5 * max_acceleration * acceleration_dt * acceleration_dt;

        // recalculate max velocity based on the time we have to accelerate and decelerate
        max_velocity = max_acceleration * acceleration_dt;

        // we decelerate at the same rate as we accelerate
        double deceleration_dt = acceleration_dt;

        // calculate the time that we're at max velocity
        double cruise_distance = distance - 2 * acceleration_distance;
        double cruise_dt = cruise_distance / max_velocity;
        double deceleration_time = acceleration_dt + cruise_dt;

        // check if we're still in the motion profile
        double entire_dt = acceleration_dt + cruise_dt + deceleration_dt;
        if (elapsed_time > entire_dt)
            return distance;

        // if we're accelerating
        if (elapsed_time < acceleration_dt)
            // use the kinematic equation for acceleration
            return 0.5 * max_acceleration * elapsed_time * elapsed_time;

        // if we're cruising
  else if (elapsed_time < deceleration_time) {
            acceleration_distance = 0.5 * max_acceleration * acceleration_dt *acceleration_dt;
            double cruise_current_dt = elapsed_time - acceleration_dt;

            // use the kinematic equation for constant velocity
            return acceleration_distance + max_velocity * cruise_current_dt;
        }

        // if we're decelerating
        else {
            acceleration_distance = 0.5 * max_acceleration * acceleration_dt * acceleration_dt;
            cruise_distance = max_velocity * cruise_dt;
            deceleration_time = elapsed_time - deceleration_time;

            // use the kinematic equations to calculate the instantaneous desired position
            return acceleration_distance + cruise_distance + max_velocity * deceleration_time - 0.5 * max_acceleration * deceleration_time * deceleration_time;
        }
    }

    public void single2(double targetPos, DcMotor motor){
        double Kp = GKP;
        double Ki = GKI;
        double Kd = GKD;

        double reference = targetPos;

        double integralSum = 0;

        double lastError = 0;

// Elapsed timer class from SDK, please use it, it's epic
        ElapsedTime timer = new ElapsedTime();

        double lastTime = 0;

        while (reference != motor.getCurrentPosition()) {

            double instantTargetPosition = motion_profile(maxA,
                    maxV,
                    reference,
                    timer.seconds());
            // obtain the encoder position
            double timeDif = timer.seconds() - lastTime;
            lastTime = timer.seconds();
            double encoderPosition = motor.getCurrentPosition();
            // calculate the error
            double error = instantTargetPosition - encoderPosition;

            // rate of change of the error
            double derivative = (error - lastError) / timeDif;

            // sum of all error over time
            integralSum = integralSum + (error * timeDif);

            double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

            motor.setPower(out);

            lastError = error;

            // reset the timer for next time

        }
        timer.reset();

    }

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

        while (targetSpinList[0] != motors[0].getCurrentPosition()) {

            for(int i = 0; i < n; i++) {
                ElapsedTime timer = new ElapsedTime();

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

                timer.reset();

            }

        }
    }
}
