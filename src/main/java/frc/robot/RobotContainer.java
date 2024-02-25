// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist.Wrist;
import frc.robot.commands.AutoIntakePower;
import frc.robot.commands.TeleopAinterupptor;
import frc.robot.commands.TeleopArm;
import frc.robot.commands.TeleopClimber;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.TeleopElevator;
import frc.robot.commands.TeleopIntake;
import frc.robot.commands.TeleopWrist;
import frc.robot.commands.TeleopYinterupptor;
// import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Elevator.Elevator;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.Elevator.Elevator;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public final Drivetrain drivetrain;
  public final Climber climber;
  public final Arm arm;
  public final Elevator elevator;
  public final Wrist wrist;
  public final Camera camera;
  public final Intake intake;
  private ShuffleboardTab fieldTab;
  private ShuffleboardTab subsystemTab;
  private Field2d robotPosition;
  private final SendableChooser<Command> autoChooser;
  // Replace with CommandPS4Controller or CommandJoystick if needed

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    drivetrain = new Drivetrain();
    climber = new Climber();
    elevator = new Elevator();
    wrist = new Wrist();
    arm = new Arm();
    camera = new Camera();
    intake = new Intake();
    configureBindings();

    drivetrain.setDefaultCommand(new TeleopDrive(drivetrain));
    climber.setDefaultCommand(new TeleopClimber(climber));
    elevator.setDefaultCommand(new TeleopElevator(elevator));
    arm.setDefaultCommand(new TeleopArm(arm));
    wrist.setDefaultCommand(new TeleopWrist(wrist));
    intake.setDefaultCommand(new TeleopIntake(intake));
    NamedCommands.registerCommand("Print Command",
        new PrintCommand("AUTO HAS TRIGGERED A PRINT COMAND WOOOOOOOOOOOOOOO"));
    NamedCommands.registerCommand("To Amp Pos",
        Commands.sequence(
        new InstantCommand(() -> arm.setTargetAngle(Constants.Presets.safeArm, 0)),
        new WaitCommand(0.4),
        new InstantCommand(() -> wrist.setWristPosition(Constants.Presets.ampWrist, 0)),
        new InstantCommand(() -> elevator.setElevatorPosition(Constants.Presets.ampElevator, 0)),
        new WaitCommand(0.75),
        new InstantCommand(() -> arm.setTargetAngle(Constants.Presets.ampArm, 0))));
    NamedCommands.registerCommand("To Intake Pos", Commands.sequence(
        new InstantCommand(() -> wrist.setWristPosition(Constants.Presets.safeWrist, 0)),
        new WaitCommand(0.75),
        new InstantCommand(() -> arm.setTargetAngle(Constants.Presets.pickupArm, 0)),
        new InstantCommand(() -> elevator.setElevatorPosition(Constants.Presets.pickupElevator, 0)),
        new WaitCommand(0.75),
        new InstantCommand(() -> wrist.setWristPosition(Constants.Presets.pickupWrist, 0))));
    NamedCommands.registerCommand("Delayed Intake Pos", Commands.sequence(
        new WaitCommand(1),
        new InstantCommand(() -> wrist.setWristPosition(Constants.Presets.safeWrist, 0)),
        new WaitCommand(0.75),
        new InstantCommand(() -> arm.setTargetAngle(Constants.Presets.pickupArm, 0)),
        new InstantCommand(() -> elevator.setElevatorPosition(Constants.Presets.pickupElevator, 0)),
        new WaitCommand(0.75),
        new InstantCommand(() -> wrist.setWristPosition(Constants.Presets.pickupWrist, 0))));
    NamedCommands.registerCommand("Intake", new AutoIntakePower(intake, 1));
    NamedCommands.registerCommand("Delayed Intake",Commands.sequence(new WaitCommand(1), new AutoIntakePower(intake, 1)));
    NamedCommands.registerCommand("Stop Intake", new AutoIntakePower(intake, 0));
    // Configure the trigger bindings

    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    
    initShuffleboardObjects();
  }

  // public void periodic() {
  // eventLoop.poll();
  // }
  public void initShuffleboardObjects() {
    fieldTab = Shuffleboard.getTab("Field");
    subsystemTab = Shuffleboard.getTab("Subsystems");

    fieldTab.add("Auto Mode", autoChooser);

    robotPosition = new Field2d();
    robotPosition.setRobotPose(new Pose2d());

    fieldTab.add(robotPosition).withPosition(1, 0).withSize(7, 4);
    fieldTab.addDouble("NavX yaw", () -> NavX.getYaw());
    fieldTab.addDouble("OFFFSET", () -> OI.getDriveOffset());
    subsystemTab.addDouble("Clmber Ticks", () -> climber.getTicks());
    subsystemTab.addDouble("Climber Power", () -> climber.getMotorPower());
    subsystemTab.addDouble("ELevator TIcks", () -> elevator.getTicks());

    subsystemTab.addString("Current Climber Sensor State", () -> climber.getClimberSensorState().toString());
    subsystemTab.addString("Current Elevator Sensor State", () -> elevator.getElevatorSensorState().toString());
    subsystemTab.addDouble("Arm ABS encoder", () -> arm.getAbsoluteTicks());
    subsystemTab.addDouble("Wrist ABS encoder", () -> wrist.getAbsoluteTicks());
    subsystemTab.addDouble("Wrist REL encoder", () -> wrist.getRelativeTicks());
    subsystemTab.addDouble("NOTE YAW",()-> drivetrain.getNoteRotationPower());
    // subsystemTab.addDouble("Current Arm Position", () -> arm.getCurrentAngle());

    // fieldTab.add(camera.getCamera()).withPosition(1, 5).withSize(4, 4);
  }

  public void updateShuffleboardObjects() {
    // System.out.println(OI.presetTest());
    robotPosition.setRobotPose(drivetrain.getPoseCorrected());
  }

  private void configureBindings() {
    OI.intakeReverse().ifHigh(()->{
      intake.setMotorPower(-1);
    });
    OI.noteAlign().ifHigh(()->{
      TeleopDrive.NoteAligning = true;
    });
    OI.noteAlign().negate().ifHigh(()->{
      TeleopDrive.NoteAligning = false;
    });
    OI.pickupPreset().rising().ifHigh(() -> Commands.sequence(
        new InstantCommand(() -> wrist.setWristPosition(Constants.Presets.safeWrist, 0)),
        new WaitCommand(0.4),
        new InstantCommand(() -> arm.setTargetAngle(Constants.Presets.pickupArm, 0)),
        new InstantCommand(() -> elevator.setElevatorPosition(Constants.Presets.pickupElevator, 0)),
        new WaitCommand(0.1),
        new InstantCommand(() -> wrist.setWristPosition(Constants.Presets.pickupWrist, 0))).schedule());
    OI.ampPreset().rising().ifHigh(() -> Commands.sequence(
        new InstantCommand(() -> arm.setTargetAngle(Constants.Presets.safeArm, 0)),
        new WaitCommand(0.4),
        new InstantCommand(() -> wrist.setWristPosition(Constants.Presets.ampWrist, 0)),
        new InstantCommand(() -> elevator.setElevatorPosition(Constants.Presets.ampElevator, 0)),
        new WaitCommand(0.75),
        new InstantCommand(() -> arm.setTargetAngle(Constants.Presets.ampArm, 0))).schedule());

    OI.storePreset().rising().ifHigh(() -> Commands.sequence(
        new InstantCommand(() -> arm.setTargetAngle(Constants.Presets.safeArm, 0)),
        new WaitCommand(0.4
        ),
        new InstantCommand(() -> wrist.setWristPosition(Constants.Presets.storeWrist, 0)),
        new InstantCommand(() -> elevator.setElevatorPosition(Constants.Presets.storeElevator, 0)),
        new WaitCommand(0.65),
        new InstantCommand(() -> arm.setTargetAngle(Constants.Presets.storeArm, 0))).schedule());
    OI.trapPreset().rising().ifHigh(() -> Commands.sequence(

        new InstantCommand(() -> elevator.setElevatorPosition(Constants.Presets.trapElevator, 0)),
        new WaitCommand(2.0),
        new InstantCommand(() -> wrist.setWristPosition(Constants.Presets.trapWrist, 0)),
        new InstantCommand(() -> arm.setTargetAngle(Constants.Presets.ampArm, 0))).schedule());

    OI.stageLEFTAlign().rising().ifHigh(()->{
      Pose2d currentPose = drivetrain.getPose();
      Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d(Math.PI/2.0));
      Pose2d endPos = new Pose2d(12.39, 2.88, new Rotation2d(Units.degreesToRadians(120.0)));//currentPose.getTranslation().plus(new Translation2d(1.0, 0.0)), new Rotation2d());
      List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
      PathPlannerPath path = new PathPlannerPath(
          bezierPoints,
          new PathConstraints(
            3.0, 2,
            Units.degreesToRadians(360), Units.degreesToRadians(540)),
          new GoalEndState(0.0, new Rotation2d(Units.degreesToRadians(120.0))));
      path.preventFlipping = true;
      new TeleopAinterupptor().deadlineWith(AutoBuilder.followPath(path)).schedule();
    });
    SmartDashboard.putData("On-the-fly path", Commands.runOnce(() -> {
      Pose2d currentPose = drivetrain.getPose();

      // The rotation component in these poses represents the direction of travel
      Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
      Pose2d endPos = new Pose2d(14.5, 7.7, new Rotation2d());

      // currentPose.getTranslation().plus(new
      // Translation2d(1.0, 0.0)), new Rotation2d());

      List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
      PathPlannerPath path = new PathPlannerPath(
          bezierPoints,
          new PathConstraints(
              1.0, 1.0,
              Units.degreesToRadians(360), Units.degreesToRadians(540)),
          new GoalEndState(0.0, currentPose.getRotation()));

      // Prevent this path from being flipped on the red alliance, since the given
      // positions are already correct
      path.preventFlipping = true;
      AutoBuilder.followPath(path).schedule();
    }));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}
