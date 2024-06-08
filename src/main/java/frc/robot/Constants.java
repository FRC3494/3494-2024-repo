package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.I2C;

public final class Constants {
  /**
   * Defines how many seconds until the estimated end of
   * match the final routine should run
   */
  public static double END_OF_MATCH_ROUTINE_STARTING_TIME = 1.5;
  public static int PDH_CAN_ID = 24;

  public static final class Presets {
    public static double testArm = 0;
    public static double testElevator = 0;
    public static double testWrist = 0;
    public static double testClimber = 0;

    public static double pickupArm = -0.043
        + Arm.globalArmOffset;
    public static double pickupElevator = 0.0;
    public static double pickupElevatorHIGH = -10;
    public static double pickupWrist = 0.01;

    public static double ampArm = 0.21 + Arm.globalArmOffset;
    public static double ampElevator = -46;
    public static double ampWrist = 0.15;

    public static double storeArm = 0.22 + Arm.globalArmOffset;
    public static double storeElevator = 0;
    public static double storeWrist = -0.164;

    public static double safeArm = 0.18 + Arm.globalArmOffset;
    public static double safeWrist = 0.12;

    public static double trapArm = 0.254 + Arm.globalArmOffset;
    public static double trapElevator = -62.8;
    public static double trapWrist = 0.49;
    public static double trapWrist2 = 0.115;
    public static double trapWrist3 = 0.0;

  }

  public static final class Drivetrain {
    public static final class FrontLeftModule {
      public static int DRIVE_MOTOR_PORT = 18;
      public static int STEER_MOTOR_PORT = 16;

      public static int ENCODER_MOTOR_PORT = 3;

      // ON ARRIVAL OFFSET: 266.00
      // on arivla 1st fix: 26612fdfddf.00+2-1.2-3.0
      // New Module OFFSET:
      // State Module: 180.0+-30.8+2.401-1.2
      // Stripped Module: -26.3996+180
      public static double STEER_OFFSET = Math.toRadians(-18.0);
    }

    public static final class FrontRightModule {
      public static int DRIVE_MOTOR_PORT = 19;
      public static int STEER_MOTOR_PORT = 17;

      public static int ENCODER_MOTOR_PORT = 2;

      // ON ARRIVAL OFFSET: 55.501
      // THE REPLACEMENT: -297.887-23.204
      // State Module: -55.501+3.2+90.0+4.0-3.199
      public static double STEER_OFFSET = Math.toRadians(64.4);
    }

    public static final class BackLeftModule {
      public static int DRIVE_MOTOR_PORT = 30;
      public static int STEER_MOTOR_PORT = 2;

      public static int ENCODER_MOTOR_PORT = 1;

      // ON ARRIVAL OFFSET: 16.809
      // State Module: 16.809-194.003+180+0.8+1.2+2.4
      public static double STEER_OFFSET = Math.toRadians(-78.8);
    }

    public static final class BackRightModule {
      public static int DRIVE_MOTOR_PORT = 1;
      public static int STEER_MOTOR_PORT = 3;

      public static int ENCODER_MOTOR_PORT = 0;

      // ON ARRIVAL OFFSET: -65.203+180.0
      // THE REPLACEMENT: 180.0-4.8
      // State Module:180.0-4.8-1.19999-0.4
      public static double STEER_OFFSET = Math.toRadians(134);
    }

    public static final double TRACKWIDTH_METERS = 0.5222;
    public static final double TRACKLENGTH_METERS = 0.574675;

    public static SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
        // Front left
        new Translation2d(TRACKLENGTH_METERS / 2.0, -TRACKWIDTH_METERS / 2.0),
        // Front right
        new Translation2d(-TRACKLENGTH_METERS / 2.0, -TRACKWIDTH_METERS / 2.0),
        // Back left
        new Translation2d(TRACKLENGTH_METERS / 2.0, TRACKWIDTH_METERS / 2.0),
        // Back right
        new Translation2d(-TRACKLENGTH_METERS / 2.0, TRACKWIDTH_METERS / 2.0));

    /**
     * calculated max speed for model; to change actual driving speed use
     * OI.DRIVE_SPEED
     */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 5.36448;
    public static final double MAX_VOLTAGE = 12.0;
    public static final double MAX_STANDARD_DEVIATION_LIMELIGHT = 0.01;
  }

  public static final class OI {
    public static final int PRIMARY_CONTROLLER_PORT = 0;
    public static final int SECONDARY_LEFT_CONTROLLER_PORT = 1;
    public static final int SECONDARY_RIGHT_CONTROLLER_PORT = 2;
    public static final int DIANNA_RUMBLER_PORT = 3;
    public static final double DRIVE_SPEED = 2.0; // m/s
    public static final double TURN_SPEED = 2.5; // rad/s

    public static final double SLOW_DRIVE_SPEED = 1.5; // m/s
    public static final double SLOW_TURN_SPEED = 1.925; // rad/s

    public static final double DPAD_SPEED = 0.5;
  }

  public static final class Climber {
    public static final int pwmServoPort = 6;
    public static final int mainMotor = 15;// 15
    public static int bottomMagnetSensorDIO = 9;
  }

  public static final class Elevator {
    public static int mainMotor = 4;
    public static int bottomMagnetSensorDIO = 0;
    public static int topMagnetSensorDIO = 1;
  }

  public static final class Arm {
    public static int armMotor = 10;
    public static double manualPowerPOS = 0.01;
    public static double manualPowerAdjustUP = 0.3;
    public static double manualPowerAdjustDOWN = 0.2;

    public static double max_position = 0;
    public static double min_position = 0;
    public static double globalArmOffset = 0.0;
  }

  public static final class Wrist {
    public static int mainMotor = 12;
    public static float MIN_POSITION = 0f;
    public static float MAX_POSITION = 0f;
    public static double manualPowerAdjust = 0.2;
    public static double globalWristOffset = 0.0;
  }

  public static final class Intake {
    public static int mainMotor = 11;
    public static double manualPowerAdjust = 1.0;
    public static I2C.Port colorSensorID = I2C.Port.kOnboard;
  }
}
