package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;
import java.util.stream.Stream;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Pose2dHelpers;

public class Drivetrain extends SubsystemBase {
	public Pose2d limelightBotPoseLeft;
	public Pose2d limelightBotPoseRight;
	public Pose2d averagedPoses;
	public Pose2d limelightBotPoseMaster;

	SwerveModule frontLeft = Mk4iSwerveModuleHelper.createAnalogNeo(
			Shuffleboard.getTab("Drivetrain").getLayout("Front Left Module", BuiltInLayouts.kList)
					.withSize(2, 4)
					.withPosition(0, 0),
			Mk4iSwerveModuleHelper.GearRatio.L2, // l1
			Constants.Drivetrain.FrontLeftModule.DRIVE_MOTOR_PORT,
			Constants.Drivetrain.FrontLeftModule.STEER_MOTOR_PORT,
			Constants.Drivetrain.FrontLeftModule.ENCODER_MOTOR_PORT,
			Constants.Drivetrain.FrontLeftModule.STEER_OFFSET);
	SwerveModule frontRight = Mk4iSwerveModuleHelper.createAnalogNeo(
			Shuffleboard.getTab("Drivetrain").getLayout("Front Right Module", BuiltInLayouts.kList)
					.withSize(2, 4)
					.withPosition(2, 0),
			Mk4iSwerveModuleHelper.GearRatio.L2, // L1
			Constants.Drivetrain.FrontRightModule.DRIVE_MOTOR_PORT,
			Constants.Drivetrain.FrontRightModule.STEER_MOTOR_PORT,
			Constants.Drivetrain.FrontRightModule.ENCODER_MOTOR_PORT,
			Constants.Drivetrain.FrontRightModule.STEER_OFFSET);

	SwerveModule backLeft = Mk4iSwerveModuleHelper.createAnalogNeo(
			Shuffleboard.getTab("Drivetrain").getLayout("Back Left Module", BuiltInLayouts.kList)
					.withSize(2, 4)
					.withPosition(4, 0),
			Mk4iSwerveModuleHelper.GearRatio.L2, // L1
			Constants.Drivetrain.BackLeftModule.DRIVE_MOTOR_PORT,
			Constants.Drivetrain.BackLeftModule.STEER_MOTOR_PORT,
			Constants.Drivetrain.BackLeftModule.ENCODER_MOTOR_PORT,
			Constants.Drivetrain.BackLeftModule.STEER_OFFSET);

	SwerveModule backRight = Mk4iSwerveModuleHelper.createAnalogNeo(
			Shuffleboard.getTab("Drivetrain").getLayout("Back Right Module", BuiltInLayouts.kList)
					.withSize(2, 4)
					.withPosition(6, 0),
			Mk4iSwerveModuleHelper.GearRatio.L2, // L1
			Constants.Drivetrain.BackRightModule.DRIVE_MOTOR_PORT,
			Constants.Drivetrain.BackRightModule.STEER_MOTOR_PORT,
			Constants.Drivetrain.BackRightModule.ENCODER_MOTOR_PORT,
			Constants.Drivetrain.BackRightModule.STEER_OFFSET);
	private SwerveModule[] m_modules = new SwerveModule[] { frontRight, frontLeft, backLeft, backRight };

	NavX navX;

	private final SwerveDrivePoseEstimator m_poseEstimator;
	private final SwerveDrivePoseEstimator correctedEstimator;

	/** Creates a new DriveSubsystem. */
	public Drivetrain() {
		correctedEstimator = new SwerveDrivePoseEstimator(Constants.Drivetrain.SWERVE_KINEMATICS,
				getGyroscopeRotation(), getSwerveModulePositions(), new Pose2d());
		m_poseEstimator = new SwerveDrivePoseEstimator(Constants.Drivetrain.SWERVE_KINEMATICS,
				getGyroscopeRotation(), getSwerveModulePositions(), new Pose2d());

		AutoBuilder.configureHolonomic(
				this::getPose,
				this::resetPose,
				this::getRobotRelativeSpeeds,
				(speeds) -> {
					this.drive(speeds);
				},
				new HolonomicPathFollowerConfig(
						// TODO: Move to constants
						new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
						new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
						// TODO: Tune these
						Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, // Max module speed, in m/s
						Constants.Drivetrain.TRACKWIDTH_METERS / 2,
						new ReplanningConfig(false, false) // Default path
				),
				() -> {
					Optional<Alliance> alliance = DriverStation.getAlliance();
					if (alliance.isPresent()) {
						return alliance.get() == DriverStation.Alliance.Red;
					}
					return false;
				},
				this);
	}

	@Override
	public void periodic() {
		m_poseEstimator.update(getGyroscopeRotation(), getSwerveModulePositions());

		Pose2d correctedPose = new Pose2d(-m_poseEstimator.getEstimatedPosition().getY(),
				m_poseEstimator.getEstimatedPosition().getX(), m_poseEstimator.getEstimatedPosition().getRotation());
		correctedEstimator.resetPosition(getGyroscopeRotation(), getSwerveModulePositions(), correctedPose);

		limelightBotPoseLeft = LimelightHelpers.getBotPose2d_wpiBlue("limelight-left");
		limelightBotPoseRight = LimelightHelpers.getBotPose2d_wpiBlue("limelight-right");

		boolean leftNeitherXNorYAt0 = limelightBotPoseLeft.getX() != 0 &&
				limelightBotPoseLeft.getY() != 0;

		limelightBotPoseLeft = new Pose2d(limelightBotPoseLeft.getX(), // + 8.27,
				limelightBotPoseLeft.getY(), /// + 4.01,
				limelightBotPoseLeft.getRotation());

		boolean rightNeitherXNorYAt0 = limelightBotPoseRight.getX() != 0 &&
				limelightBotPoseRight.getY() != 0;

		limelightBotPoseRight = new Pose2d(limelightBotPoseRight.getX(), // + 8.27,
				limelightBotPoseRight.getY(), /// + 4.01,
				limelightBotPoseRight.getRotation());

		// taken from Sonic Squirrels FRC Team 2930
		Matrix<N3, N1> permissableError = VecBuilder.fill(0.9, 0.9, 0.9);

		// SET MASTER BOT POSE
		// ONLY SET while not auto aligning
		if (rightNeitherXNorYAt0 && leftNeitherXNorYAt0 &&
				!RobotState.isAutonomous()) {
			averagedPoses = Pose2dHelpers.meanCorrect(limelightBotPoseLeft,
					limelightBotPoseRight);

			m_poseEstimator.addVisionMeasurement(
					new Pose2d(averagedPoses.getY(),
							-averagedPoses.getX(),
							averagedPoses.getRotation()),
					Timer.getFPGATimestamp(),
					permissableError);
		} else if (rightNeitherXNorYAt0 && !RobotState.isAutonomous()) {
			m_poseEstimator.addVisionMeasurement(
					new Pose2d(limelightBotPoseRight.getY(),
							-limelightBotPoseRight.getX(),
							limelightBotPoseRight.getRotation()),
					Timer.getFPGATimestamp(),
					permissableError);
		} else if (leftNeitherXNorYAt0 && !RobotState.isAutonomous()) {
			m_poseEstimator.addVisionMeasurement(
					new Pose2d(limelightBotPoseLeft.getY(),
							-limelightBotPoseLeft.getX(),
							limelightBotPoseLeft.getRotation()),
					Timer.getFPGATimestamp(),
					permissableError);
		}

		SmartDashboard.putBoolean("Averaging", rightNeitherXNorYAt0 &&
				leftNeitherXNorYAt0);
		SmartDashboard.putNumber("Left Odo", limelightBotPoseLeft.getX());
		SmartDashboard.putNumber("Right Odo", limelightBotPoseRight.getX());
		SmartDashboard.putNumber("True Odo",
				m_poseEstimator.getEstimatedPosition().getX());

		Logger.recordOutput("Drivetrain/IsLocked", isLocked);
		Logger.recordOutput("Drivetrain/EstimatedPose", getPose());
		Logger.recordOutput("Drivetrain/RobotRotation", getPose().getRotation().getRadians());
		Logger.recordOutput("Drivetrain/States", getSwerveDriveStates().toArray(SwerveModuleState[]::new));
	}

	public double getNoteRotationPower() {
		double tx1 = LimelightHelpers.getTX("limelight-bottom");
		double ty1 = LimelightHelpers.getTX("limelight-bottom");
		double tx2 = LimelightHelpers.getTX("limelight-rightb");
		double ty2 = LimelightHelpers.getTX("limelight-rightb");
		if ((ty1 < ty2 || ty2 == 0) && ty1 != 0) {
			if (tx1 != 0) {
				double noteYaw = 11.0 + tx1;// wass 16+
				noteYaw /= 14.0;// 16 seems to little with intake down
				return noteYaw;
			}
		} else {
			if (tx2 != 0) {
				double noteYaw = -11.0 + tx2;// FIX ME NOT 11!!!
				noteYaw /= 14.0;// 16 seems to little with intake down
				return noteYaw;
			}
		}

		return 0;

	}

	public double getNoteRotationPowerPOWERFUL() {
		double tx1 = LimelightHelpers.getTX("limelight-bottom");
		double ty1 = LimelightHelpers.getTY("limelight-bottom");
		double tx2 = LimelightHelpers.getTX("limelight-rightb");
		double ty2 = LimelightHelpers.getTY("limelight-rightb");
		if ((ty1 < ty2 || ty2 == 0) && ty1 != 0) {
			if (tx1 != 0) {
				double noteYaw = 11.0 + tx1;// wass 16+
				noteYaw /= 8.0;// 16 seems to little with intake down
				return noteYaw;
			}
		} else {
			if (tx2 != 0) {
				double noteYaw = -11.0 + tx2;// FIX ME NOT 11!!!
				noteYaw /= 8.0;// 16 seems to little with intake down
				return noteYaw;
			}
		}

		return 0;

	}

	public boolean seesNote() {
		double tx1 = LimelightHelpers.getTX("limelight-bottom");
		double ty1 = LimelightHelpers.getTX("limelight-bottom");
		double tx2 = LimelightHelpers.getTX("limelight-rightb");
		double ty2 = LimelightHelpers.getTX("limelight-rightb");
		
		if (tx1 == 0 && ty1 == 0 && tx2 == 0 && ty2 == 0) {
			return false;
		}
		
		return true;
	}
	// public boolean seesNote(){
	// return LimelightHelpers.getTX("limelight-bottom") != 0;
	// }

	/**
	 * Returns the currently-estimated pose of the robot.
	 *
	 * @return The pose.
	 */
	public Pose2d getPose() {
		return correctedEstimator.getEstimatedPosition();
		// return m_poseEstimator.getEstimatedPosition();
	}

	public Pose2d getPoseCorrected() {
		return correctedEstimator.getEstimatedPosition();
	}

	/**
	 * Returns the current swerve kinematics.
	 *
	 * @return The swerve kinematics.
	 */
	public SwerveDriveKinematics getKinematics() {
		return Constants.Drivetrain.SWERVE_KINEMATICS;
	}

	/**
	 * Returns the relative speeds of the robot as a chassis speed
	 * 
	 * @return Relative speeds of the robot as a chassis speed
	 */
	public List<SwerveModuleState> getSwerveDriveStates() {
		Stream<SwerveModuleState> swerveDriveWheelStates = Stream.of(m_modules).map(
				(module) -> new SwerveModuleState(
						module.getVelocity(),
						Rotation2d.fromRadians(module.getSteerAngle())));

		return swerveDriveWheelStates.toList();
	}

	/**
	 * Resets the odometry to the specified pose.
	 *
	 * @param pose The pose to which to set the odometry.
	 */
	public void resetPose(Pose2d pose) {
		Pose2d poseT = new Pose2d(pose.getY(), -pose.getX(), pose.getRotation());
		m_poseEstimator.resetPosition(getGyroscopeRotation(), getSwerveModulePositions(), poseT);
	}

	/**
	 * Method to drive the robot using joystick info.
	 *
	 * @param xSpeed        Speed of the robot in the x direction (forward).
	 * @param ySpeed        Speed of the robot in the y direction (sideways).
	 * @param rot           Angular rate of the robot.
	 * @param fieldRelative Whether the provided x and y speeds are relative to the
	 *                      field.
	 */
	@SuppressWarnings("ParameterName")
	public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
		if (isLocked)
			return;

		var swerveModuleStates = Constants.Drivetrain.SWERVE_KINEMATICS.toSwerveModuleStates(
				fieldRelative
						? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getGyroscopeRotation())
						: new ChassisSpeeds(xSpeed, ySpeed, rot));

		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
				Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND);

		if (Math.abs(xSpeed) < .001 && Math.abs(ySpeed) < .001 && Math.abs(rot) < .001) {
			frontLeft.set(0, frontLeft.getState().angle.getRadians());
			frontRight.set(0, frontRight.getState().angle.getRadians());
			backLeft.set(0, backLeft.getState().angle.getRadians());
			backRight.set(0, backRight.getState().angle.getRadians());
			return;
		}

		setModuleStates(swerveModuleStates);
	}

	public void drive(ChassisSpeeds speeds) {
		ChassisSpeeds newSpeeds = new ChassisSpeeds(speeds.vyMetersPerSecond, -speeds.vxMetersPerSecond,
				speeds.omegaRadiansPerSecond);
		SwerveModuleState[] swerveModuleStates = Constants.Drivetrain.SWERVE_KINEMATICS
				.toSwerveModuleStates(newSpeeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
				Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND);

		setModuleStates(swerveModuleStates);
	}

	boolean isLocked = false;

	public void lock() {
		frontLeft.set(0, Math.toRadians(45));
		frontRight.set(0, -Math.toRadians(45));
		backLeft.set(0, -Math.toRadians(45));
		backRight.set(0, Math.toRadians(45));

		isLocked = true;
	}

	public void unlock() {
		isLocked = false;
	}

	/**
	 * Sets the swerve ModuleStates.
	 *
	 * @param desiredStates The desired SwerveModule states.
	 */
	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(
				desiredStates, Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND);

		Logger.recordOutput("Drivetrain/TargetStates", desiredStates);

		frontLeft.set(
				desiredStates[0].speedMetersPerSecond / Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND
						* Constants.Drivetrain.MAX_VOLTAGE,
				desiredStates[0].angle.getRadians());
		frontRight.set(
				desiredStates[1].speedMetersPerSecond / Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND
						* Constants.Drivetrain.MAX_VOLTAGE,
				desiredStates[1].angle.getRadians());
		backLeft.set(
				desiredStates[2].speedMetersPerSecond / Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND
						* Constants.Drivetrain.MAX_VOLTAGE,
				desiredStates[2].angle.getRadians());
		backRight.set(
				desiredStates[3].speedMetersPerSecond / Constants.Drivetrain.MAX_VELOCITY_METERS_PER_SECOND
						* Constants.Drivetrain.MAX_VOLTAGE,
				desiredStates[3].angle.getRadians());
	}

	public SwerveModulePosition[] getSwerveModulePositions() {
		return new SwerveModulePosition[] {
				frontLeft.getState(),
				frontRight.getState(),
				backLeft.getState(),
				backRight.getState()
		};
	}

	/**
	 * Returns the current heading of the chassis
	 *
	 * @return The current heading of the chassis.
	 */
	public Rotation2d getGyroscopeRotation() {
		if (NavX.getNavX().isMagnetometerCalibrated()) {
			// We will only get valid fused headings if the magnetometer is calibrated
			return Rotation2d.fromDegrees(NavX.getYaw());
		}
		return Rotation2d.fromDegrees(360.0 - NavX.getYaw());
	}

	private ChassisSpeeds getRobotRelativeSpeeds() {
		var states = getSwerveDriveStates().iterator();

		return Constants.Drivetrain.SWERVE_KINEMATICS.toChassisSpeeds(
				states.next(),
				states.next(),
				states.next(),
				states.next());
	}
}
