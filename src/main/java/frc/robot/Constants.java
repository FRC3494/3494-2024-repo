package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {
  public static final class Drivetrain {
    public static final class FrontLeftModule {
      public static int DRIVE_MOTOR_PORT = 19;// 19;
      public static int STEER_MOTOR_PORT = 17;// 17;

      public static int ENCODER_MOTOR_PORT = 1;

      public static double STEER_OFFSET = Math.toRadians(262);
    }

    public static final class FrontRightModule {
      public static int DRIVE_MOTOR_PORT = 16;// 16;
      public static int STEER_MOTOR_PORT = 18;// 18;

      public static int ENCODER_MOTOR_PORT = 2;

      public static double STEER_OFFSET = Math.toRadians(20);
    }

    public static final class BackLeftModule {
      public static int DRIVE_MOTOR_PORT = 30;// 14;
      public static int STEER_MOTOR_PORT = 2;// 15;

      public static int ENCODER_MOTOR_PORT = 0;

      public static double STEER_OFFSET = Math.toRadians(236);
    }

    public static final class BackRightModule {
      public static int DRIVE_MOTOR_PORT = 3;// 5;
      public static int STEER_MOTOR_PORT = 1;// 4;

      public static int ENCODER_MOTOR_PORT = 3;

      public static double STEER_OFFSET = Math.toRadians(282);
    }

    public static final double TRACKWIDTH_METERS = 0.47625;
    public static final double TRACKLENGTH_METERS = 0.52705;

    public static SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
        // Front left
        new Translation2d(TRACKLENGTH_METERS / 2.0, -TRACKWIDTH_METERS / 2.0),
        // Front right
        new Translation2d(-TRACKLENGTH_METERS / 2.0, -TRACKWIDTH_METERS / 2.0),
        // Back left
        new Translation2d(TRACKLENGTH_METERS / 2.0, TRACKWIDTH_METERS / 2.0),
        // Back right
        new Translation2d(-TRACKLENGTH_METERS / 2.0, TRACKWIDTH_METERS / 2.0));
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 3.6576f;
    public static final double MAX_VOLTAGE = 12.0;
    public static final double MAX_STANDARD_DEVIATION_LIMELIGHT = 0.01;
  }

  public static final class OI {
    public static final int PRIMARY_CONTROLLER_PORT = 0;
    public static final int SECONDARY_LEFT_CONTROLLER_PORT = 1;
    public static final int SECONDARY_RIGHT_CONTROLLER_PORT = 2;

    public static final double DRIVE_SPEED = 3.5; // m/s
    public static final double TURN_SPEED = 5.5; // rad/s

    public static final double SLOW_DRIVE_SPEED = 1.5; // m/s
    public static final double SLOW_TURN_SPEED = 1.925; // rad/s

    public static final double DPAD_SPEED = 0.5;
  }

  public static final class Elevator {

    public static int mainMotor = 0;// TODO: add ports
    public static int followerMotor = 0;

  }
}
