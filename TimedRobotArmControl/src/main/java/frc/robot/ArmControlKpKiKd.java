package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmControlKpKiKd {

    private double Kp; // Proportional gain
    private double Ki; // Integral gain
    private double Kd; // Derivative gain

    private double previousShoulderError; 
    private double previousWristError; 

    public ArmControlKpKiKd(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.previousShoulderError = 0.0;
        this.previousWristError = 0.0;
    }

    public double calculateShoulderVelocity(double desiredShoulderPosition, double currentShoulderPosition) {
        double error = desiredShoulderPosition - currentShoulderPosition;
        double integral = 0.0; 
        double derivative = 0.0;

        if (Ki != 0) {
            integral = previousShoulderError + error; 
        }

        if (Kd != 0) {
            derivative = (error - previousShoulderError); 
        }

        double velocity = (Kp * error) + (Ki * integral) + (Kd * derivative); 
        velocity = Math.max(-0.1, Math.min(0.1, velocity)); // Limit velocity to +/- 0.1

        previousShoulderError = error;

        return velocity;
    }

    public double calculateWristVelocity(double desiredWristPosition, double currentWristPosition) {
        double error = desiredWristPosition - currentWristPosition;
        double integral = 0.0; 
        double derivative = 0.0;

        if (Ki != 0) {
            integral = previousWristError + error; 
        }

        if (Kd != 0) {
            derivative = (error - previousWristError); 
        }

        double velocity = (Kp * error) + (Ki * integral) + (Kd * derivative); 
        velocity = Math.max(-0.1, Math.min(0.1, velocity)); // Limit velocity to +/- 0.1

        previousWristError = error;

        return velocity;
    }
}