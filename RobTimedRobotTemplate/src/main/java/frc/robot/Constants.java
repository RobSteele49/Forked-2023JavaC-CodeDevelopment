package frc.robot;

public class Constants {
    public static class OperatorConstants {
      public static final int kDriverControllerPort = 0;
    }

    /*
     * Constants for the Can Bus ID
     */

    public static class CanBus {
      public static final int kGripper       =  1;
      public static final int kLowerArmJoint =  2;
      public static final int kUpperArmJoint =  3;
      public static final int kLeftMotorAID  = 13;
      public static final int kLeftMotorBID  = 14;
      public static final int kRightMotorAID = 15;
      public static final int kRightMotorBID = 16;
    }

    /*
     * Joystick port IDs
     */
    
     public static class JoystickPortID {
      public static final int kLeftJoystick    = 0;
      public static final int kRightJoystick   = 1;
      public static final int kControlJoystick = 2;
     }

}
