// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Joystick;

import frc.robot.Constants.CanBus;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private Joystick m_leftStick;
  private Joystick m_rightStick;
  private Joystick m_controlStick;

  private VictorSPX leftMotor1;
  private VictorSPX leftMotor2;
  private VictorSPX rightMotor1;
  private VictorSPX rightMotor2;

  private VictorSPX gripperMotor;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    leftMotor1 = new VictorSPX(CanBus.kLeftMotorAID);
    leftMotor2 = new VictorSPX(CanBus.kLeftMotorBID);

    rightMotor1 = new VictorSPX(CanBus.kRightMotorAID);
    rightMotor2 = new VictorSPX(CanBus.kRightMotorBID);

    gripperMotor = new VictorSPX(CanBus.kGripper);

    m_leftStick = new Joystick(0);
    m_rightStick = new Joystick(1);
    m_controlStick = new Joystick(2);

    // Set leftMotor2 to follow leftMotor1
    // Set rightMotor2 to follow rightMotor1
    
    leftMotor2.follow(leftMotor1);
    rightMotor2.follow(rightMotor1);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.

    rightMotor1.setInverted(true);
    rightMotor2.setInverted(true);

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double leftSpeed  = m_leftStick.getY(); // Get the Y axis value from the joystick
    double rightSpeed = m_rightStick.getY(); 

    leftMotor1.set(ControlMode.PercentOutput,  leftSpeed); // Set the motor speed
    rightMotor1.set(ControlMode.PercentOutput, rightSpeed); // set the motor speed

    boolean button4 = m_controlStick.getRawButton(4);
    boolean button6 = m_controlStick.getRawButton(6);
  
    if (button4) {
      gripperMotor.set(ControlMode.PercentOutput, 0.25);
    } else {
      gripperMotor.set(ControlMode.PercentOutput, 0.0);
      if (button6) {
        gripperMotor.set(ControlMode.PercentOutput, -0.25);
      } else {
          gripperMotor.set(ControlMode.PercentOutput, 0.0);
      }
    }


  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
