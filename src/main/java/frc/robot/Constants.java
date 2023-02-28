package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;

public class Constants {

  public static final double stickDeadband = 0.1;

  public static final class OIConstants {
    public static final double kDeadzoneThreshold = stickDeadband;
    public static final int kDriveControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static final class ArmConstants {

    // Phoenix ID's
    public static final int kArmMotor1 = 57;
    public static final int kArmMotor2 = 58;
    // public static final int kArmPulley = 60;

    // setpoints
    public static final int kArmHome = 0;
    public static final int kArmAcquireFromFloor = 10_000;
    public static final int kArmAcquireFromDOS = 10_000;
    public static final int kArmAcquireFromSIS = 20_000;

    public static final int kArmScoreInHighCone = 20_000;
    public static final int kArmScoreInHighCube = 10_000;
    public static final int kArmScoreInMid = 10_000;

    public static final class PIDF {
      // PID Constants
      public static final double kP = 0;
      public static final double kI = 0;
      public static final double kD = 0;
      public static final double kF = 0; // Not needed due to using ArmFeedforward

      // Feedforward Constants
      public static final double kS = 0;
      public static final double kG = 0;
      public static final double kV = 0;
      public static final ArmFeedforward arm_ff = new ArmFeedforward(kS, kG, kV);
    }
  }


  

  public static final class TelescopeConstants {
    public static final int kTelescopeMotor = 60;

    public static final double kTelescopeSpeed = 0.05;

    public static final double kTelescopeOffset = 0;
    public static final double kTelescopeExtended = 6000;
  }
}