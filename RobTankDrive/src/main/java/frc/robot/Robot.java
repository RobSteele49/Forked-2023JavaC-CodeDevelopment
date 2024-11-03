// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
// import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
// import java.rmi.registry.Registry;


/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  // private DifferentialDrive m_robotDrive;
  private Joystick m_leftStick;
  private Joystick m_rightStick;

  // private final PWMSparkMax m_leftMotor = new PWMSparkMax(0);
  // private final PWMSparkMax m_rightMotor = new PWMSparkMax(1);

  private VictorSPX leftMotor1;
  private VictorSPX leftMotor2;
  private VictorSPX rightMotor1;
  private VictorSPX rightMotor2;

  @Override
  public void robotInit() {
    // SendableRegistry.addChild(m_robotDrive, m_leftMotor);
    // SendableRegistry.addChild(m_robotDrive, m_rightMotor);

    leftMotor1 = new VictorSPX(12); // CAN ID 12
    leftMotor2 = new VictorSPX(13); // CAN ID 13

    rightMotor1 = new VictorSPX(14); // CAN ID 14
    rightMotor2 = new VictorSPX(15); // CAN ID 15

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.

    // m_rightMotor.setInverted(true);
    // m_robotDrive = new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);

    m_leftStick = new Joystick(0);
    m_rightStick = new Joystick(1);

    // Set leftMotor2 to follow leftMotor1
    leftMotor2.follow(leftMotor1);
    rightMotor2.follow(rightMotor1);

  }

  @Override
  public void teleopPeriodic() {
    // m_robotDrive.tankDrive(-m_leftStick.getY(), -m_rightStick.getY());

    double leftSpeed = m_leftStick.getY(); // Get the Y axis value from the joystick
    double rightSpeed = m_rightStick.getY(); 

    leftMotor1.set(ControlMode.PercentOutput, leftSpeed); // Set the motor speed
    rightMotor1.set(ControlMode.PercentOutput, -rightSpeed); // set the motor speed
  }
}
