package org.firstinspires.ftc.teamcode.Utilities.PID;

import org.firstinspires.ftc.teamcode.Utilities.Utils;

public class PID {

    private double proportional;
    private double integral;
    private double derivative;
    private double result = 0;
    private double integralSum = 0;
    private double previousError = 0;
    private long previousTime;

    private PIDRingBuffer errors;

    private boolean debugMode;

    public PID(double proportional, double integral, double derivative, int integralLength) {
        this(proportional, integral, derivative, integralLength, false);
    }

    public PID(double proportional, double integral, double derivative, int integralLength, boolean debugMode) {
        this.proportional = proportional;
        this.integral = integral;
        this.derivative = derivative;
        this.debugMode = debugMode;
        //errors = new PIDRingBuffer(integralLength);
        previousTime = System.currentTimeMillis();
    }

    public Double update(double error){
        //PIDRingBuffer.IntegralDerivativePair integralDerivativePair = errors.update(error, System.currentTimeMillis());
        integralSum += error;
        long currentTime = System.currentTimeMillis();
        double deltaTime = (currentTime - previousTime) / 1000.0;
        double rateOfChange = (error - previousError) / deltaTime;
        previousTime = currentTime;
        previousError = error;
        double pComponent = error * proportional;
        double iComponent = integralSum * integral;
        double dComponent = (rateOfChange * derivative);
        if(debugMode){
            Utils.telemetry.addData("Proportional", pComponent);
            Utils.telemetry.addData("Integral", iComponent);
            Utils.telemetry.addData("Derivative", dComponent);
        }
        this.result = pComponent + iComponent + dComponent;
        return result;
    }
    public double getResult() {
        return result;
    }
}

