// Filename: Constants.java

package frc.robot;

public class Constants {
    public static class OperatorConstants {
      public static final int kDriverControllerPort = 0;
    }

    /*
     * Constants for the Can Bus ID
     */

    public static class CanBusID {
      public static final int kGripper       =  1;
      public static final int kShoulderJoint =  2;
      public static final int kWristJoint    =  3;
      public static final int kLeftSimA      = 13;
      public static final int kLeftSimB      = 14;
      public static final int kRightSimA     = 12;
      public static final int kRightSimB     = 15;
    }

    /*
     * Joystick port IDs
     */
    
     public static class JoystickPortID {
      public static final int kLeftJoystick  = 0;
      public static final int kRightJoystick = 1;
      public static final int kArmJoystick   = 2;
     }

     /*
      * Default Velocity +/- for the gripper
      */

      public static class Gripper {
        public static final double kGripperVelocity = 0.5;
      }

      /*
       * Will see about having this variable explicitely set to true
       * for simulation mode. Will need to set false for the real
       * robot.
       */

      public static class SimulationMode {
        public static Boolean simulate = true;
      }
      
}
