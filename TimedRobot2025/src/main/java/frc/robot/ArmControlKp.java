
package frc.robot;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Import necessary libraries
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;

// import com.revrobotics.AbsoluteEncoder; // in my original code I was not using the absolute encoder

public class ArmControlKp {

    private double Kp; // Proportional gain

    public ArmControlKp (double Kp) {
        this.Kp = Kp;
    }

    public double calculateShoulderVelocity(
    double desiredShoulderPosition, double currentShoulderPosition) {
        double error = desiredShoulderPosition - currentShoulderPosition;
        double velocity = Kp * error; 
        velocity = Math.max(-0.1, Math.min(0.1, velocity)); // Limit velocity to +/- 0.1
        return velocity;
    }

    public double calculateWristVelocity(
    double desiredWristPosition, double currentWristPosition) {
        double error = desiredWristPosition - currentWristPosition;
        double velocity = Kp * error;
        velocity = Math.max(-0.1, Math.min(0.1, velocity)); // Limit velocity to +/- 0.1
        return velocity;
    }
}