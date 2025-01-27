package org.firstinspires.ftc.teamcode.OpModes;

public class PID {

    private double kP; // Proportional gain
    private double kI; // Integral gain
    private double kD; // Derivative gain

    private double targetPosition; // Desired encoder position

    private double integral; // Integral term accumulator
    private double previousError; // Previous error for derivative calculation

    public PID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.integral = 0.000001;
        this.previousError = 0;
        this.targetPosition = 0;
    }

    // Set the target position (desired encoder value)
    public void setTargetPosition(double targetPosition) {
        this.targetPosition = targetPosition;
        this.integral = 0; // Reset integral on new target
        this.previousError = 0; // Reset previous error on new target
    }

    // Calculate motor power for the current position
    public double calculatePower(double currentPosition) {
        // Calculate error
        double error = targetPosition - currentPosition;

        // Accumulate integral
        integral += error;

        // Calculate derivative
        double derivative = error - previousError;

        // Update previous error
        previousError = error;

        // Compute PID output
        return (kP * error) + (kI * integral) + (kD * derivative);
    }

    // Apply PID control to two motors

}
