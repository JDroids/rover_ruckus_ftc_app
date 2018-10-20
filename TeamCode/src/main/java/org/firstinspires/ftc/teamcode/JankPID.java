package org.firstinspires.ftc.teamcode;

import android.util.Log;

/**
 * Created by dansm on 2/28/2018.
 */

public class JankPID {
    private double kp;
    private double ki;
    private double kd;

    private double now;
    private double lastTime = 0;
    private double timeChange;

    private double previousError;

    private double error;
    private double errSum = 0;
    private double dErr = 0;

    private double output;

    public void setCoeffecients(double Kp, double Ki, double Kd) {
        kp = Kp;
        ki = Ki;
        kd = Kd;
    }

    public double calculateOutput(double target, double currentValue) {
        //How long since we calculated
        now = System.currentTimeMillis();
        timeChange = (double) (now - lastTime);

        //Computes error working variable
        error = target - currentValue;
        errSum += (error * timeChange);
        dErr = (error - previousError) / timeChange;

        output = (kp * error) + (ki * errSum) + (kd * dErr);

        //Remember some stuff for next loop
        previousError = error;
        lastTime = System.currentTimeMillis();

        return output;

    }

    private boolean firstTime = true;
}
