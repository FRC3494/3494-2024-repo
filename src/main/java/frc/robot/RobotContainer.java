// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoIntakePower;
import frc.robot.commands.AutoNoteConfirm;
import frc.robot.commands.AutoPickupNote;
import frc.robot.commands.TeleopAinterupptor;
import frc.robot.commands.TeleopArm;
import frc.robot.commands.TeleopBackInterrupter;
import frc.robot.commands.TeleopBumperInterupptor;
import frc.robot.commands.TeleopClimber;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.TeleopDriveAutomated;
import frc.robot.commands.TeleopElevator;
import frc.robot.commands.TeleopIntake;
import frc.robot.commands.TeleopStartinterupptor;
import frc.robot.commands.TeleopWrist;
import frc.robot.subsystems.Arm;
// import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Wrist.Wrist;

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
  // public final Camera camera;
  public final Intake intake;
  private ShuffleboardTab fieldTab;
  private ShuffleboardTab subsystemTab;
  private Field2d robotPosition;
  private final LoggedDashboardChooser<Command> autoChooser;

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
    // camera = new Camera();
    intake = new Intake(OI.getEventLoop());
    configureBindings();

    drivetrain.setDefaultCommand(new TeleopDrive(drivetrain));
    climber.setDefaultCommand(new TeleopClimber(climber));
    elevator.setDefaultCommand(new TeleopElevator(elevator));
    arm.setDefaultCommand(new TeleopArm(arm));
    wrist.setDefaultCommand(new TeleopWrist(wrist));
    intake.setDefaultCommand(new TeleopIntake(intake, elevator));
    NamedCommands.registerCommand("VisionNoteGrab", new AutoPickupNote(drivetrain, intake, 10));
    NamedCommands.registerCommand("To Store Pos",
        Commands.sequence(
            new InstantCommand(() -> intake.inIntake = false),
            new InstantCommand(() -> arm.setTargetAngle(Constants.Presets.safeArm, 0)),
            new WaitCommand(0.4),
            new InstantCommand(() -> wrist.setWristPosition(Constants.Presets.storeWrist, 0)),
            new InstantCommand(() -> elevator.setElevatorPosition(Constants.Presets.storeElevator, 0)),
            new WaitCommand(0.65),
            new InstantCommand(() -> arm.setTargetAngle(Constants.Presets.storeArm, 0))));
    NamedCommands.registerCommand("SwerveZero",
        new TeleopDriveAutomated(drivetrain, 0, -0.3, 10.0));
    NamedCommands.registerCommand("Confirm Note",
        new AutoNoteConfirm(intake));
    NamedCommands.registerCommand("Move Wrist",
        new InstantCommand(() -> wrist.setWristPosition(Constants.Presets.trapWrist, 0)));
    NamedCommands.registerCommand("Print Command",
        new PrintCommand("AUTO HAS TRIGGERED A PRINT COMAND WOOOOOOOOOOOOOOO"));
    NamedCommands.registerCommand("To Amp Pos",
        Commands.sequence(
            new InstantCommand(() -> intake.inIntake = false),
            new InstantCommand(() -> arm.setTargetAngle(Constants.Presets.safeArm, 0)),
            new WaitCommand(0.4),
            new InstantCommand(() -> wrist.setWristPosition(Constants.Presets.ampWrist, 0)),
            new InstantCommand(() -> elevator.setElevatorPosition(Constants.Presets.ampElevator, 0)),
            new WaitCommand(0.75),
            new InstantCommand(() -> arm.setTargetAngle(Constants.Presets.ampArm, 0))));
    NamedCommands.registerCommand("To Intake Pos", Commands.sequence(
        new InstantCommand(() -> intake.inIntake = true),
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
        new InstantCommand(() -> intake.inIntake = true),
        new InstantCommand(() -> wrist.setWristPosition(Constants.Presets.pickupWrist, 0))));
    NamedCommands.registerCommand("Intake Pos", Commands.sequence(
        new InstantCommand(() -> wrist.setWristPosition(Constants.Presets.safeWrist, 0)),
        new WaitCommand(0.75),
        new InstantCommand(() -> arm.setTargetAngle(Constants.Presets.pickupArm, 0)),
        new InstantCommand(() -> elevator.setElevatorPosition(Constants.Presets.pickupElevator, 0)),
        new WaitCommand(0.75),
        new InstantCommand(() -> intake.inIntake = true),
        new InstantCommand(() -> wrist.setWristPosition(Constants.Presets.pickupWrist, 0))));
    NamedCommands.registerCommand("Intake", new AutoIntakePower(intake, 1));
    NamedCommands.registerCommand("Outtake", new AutoIntakePower(intake, -1));
    NamedCommands.registerCommand("Delayed Intake",
        Commands.sequence(new WaitCommand(1), new AutoIntakePower(intake, 1)));
    NamedCommands.registerCommand("Stop Intake", new AutoIntakePower(intake, 0));
    // Configure the trigger bindings

    autoChooser = new LoggedDashboardChooser<>("Auto Routine", AutoBuilder.buildAutoChooser());

    initShuffleboardObjects();
  }

  // public void periodic() {
  // eventLoop.poll();
  // }
  public void initShuffleboardObjects() {
    fieldTab = Shuffleboard.getTab("Field");
    subsystemTab = Shuffleboard.getTab("Subsystems");

    subsystemTab.add("Auto Mode", autoChooser.getSendableChooser());

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
    subsystemTab.addDouble("NOTE YAW", () -> drivetrain.getNoteRotationPower());
    subsystemTab.addBoolean("Note", () -> intake.hasNote());
    subsystemTab.addBoolean("Ratchet Engaged", () -> climber.ratchetEngaged);
    subsystemTab.addDouble("Note Prox", () -> intake.getSensorProximity());
    subsystemTab.addBoolean("Current Sens Enabled", () -> intake.isSensing());
    // subsystemTab.addDouble("Current Arm Position", () -> arm.getCurrentAngle());

    // fieldTab.add(camera.getCamera()).withPosition(1, 5).withSize(4, 4);
  }

  public void updateShuffleboardObjects() {
    // System.out.println(OI.presetTest());
    robotPosition.setRobotPose(drivetrain.getPoseCorrected());
  }

  private void configureBindings() {
    // OI.intakeReverse().ifHigh(() -> {
    // intake.setMotorPower(-1);
    // });
    OI.noteAlign().ifHigh(() -> {
      TeleopDrive.NoteAligning = true;
    });
    OI.noteAlign().negate().ifHigh(() -> {
      TeleopDrive.NoteAligning = false;
    });
    OI.pickupPreset().rising().ifHigh(() -> Commands.sequence(
        new InstantCommand(() -> wrist.setWristPosition(Constants.Presets.safeWrist, 0)),
        new InstantCommand(() -> intake.inIntake = true),
        new WaitCommand(0.4),
        new InstantCommand(() -> arm.setTargetAngle(Constants.Presets.pickupArm, 0)),
        new InstantCommand(() -> elevator.setElevatorPosition(Constants.Presets.pickupElevatorHIGH, 0)),
        new WaitCommand(0.1),
        new InstantCommand(() -> wrist.setWristPosition(Constants.Presets.pickupWrist, 0))).schedule());
    OI.ampPreset().rising().ifHigh(() -> Commands.sequence(
        new InstantCommand(() -> intake.inIntake = false),
        // new InstantCommand(() -> arm.setTargetAngle(Constants.Presets.safeArm, 0)),
        // new WaitCommand(0.4),
        new InstantCommand(() -> wrist.setWristPosition(Constants.Presets.ampWrist, 0)),
        new InstantCommand(() -> elevator.setElevatorPosition(Constants.Presets.ampElevator, 0)),
        // new WaitCommand(0.75),
        new InstantCommand(() -> arm.setTargetAngle(Constants.Presets.ampArm, 0))).schedule());

    OI.storePreset().rising().ifHigh(() -> Commands.sequence(
        new InstantCommand(() -> intake.inIntake = false),
        new InstantCommand(() -> arm.setTargetAngle(Constants.Presets.safeArm, 0)),
        new WaitCommand(0.4),
        new InstantCommand(() -> wrist.setWristPosition(Constants.Presets.storeWrist, 0)),
        new InstantCommand(() -> elevator.setElevatorPosition(Constants.Presets.storeElevator, 0)),
        new WaitCommand(0.65),
        new InstantCommand(() -> arm.setTargetAngle(Constants.Presets.storeArm, 0))).schedule());
    OI.trapPreset().rising().ifHigh(() -> Commands.sequence(
        new InstantCommand(() -> intake.inIntake = false),
        new InstantCommand(() -> wrist.setWristPosition(Constants.Presets.ampWrist, 0)),
        new WaitCommand(0.5),
        new InstantCommand(() -> elevator.setElevatorPosition(Constants.Presets.trapElevator, 0)),
        new WaitCommand(0.5),
        new InstantCommand(() -> wrist.setWristPosition(Constants.Presets.trapWrist, 0)),
        new InstantCommand(() -> arm.setTargetAngle(Constants.Presets.ampArm, 0))).schedule());
    OI.trapPreset2().rising().ifHigh(() -> Commands.sequence(
        new InstantCommand(() -> intake.inIntake = false),
        new InstantCommand(() -> wrist.setWristPosition(Constants.Presets.trapWrist2, 0))).schedule());
    OI.stageBACKAlign().rising().ifHigh(() -> {
      Optional<Alliance> teamColor = DriverStation.getAlliance();
      if (teamColor.get() == Alliance.Red) {
        Pose2d currentPose = drivetrain.getPose();
        Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d(Math.PI / 2.0));// 10.52, 4.0
        Pose2d endPos = new Pose2d(10.45, 4.0100, new Rotation2d(Units.degreesToRadians(0)));// currentPose.getTranslation().plus(new
                                                                                             // Translation2d(1.0,
                                                                                             // 0.0)), new
                                                                                             // Rotation2d());
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
        PathPlannerPath path = new PathPlannerPath(
            bezierPoints,
            new PathConstraints(
                3.0, 2,
                Units.degreesToRadians(360), Units.degreesToRadians(540)),
            new GoalEndState(0.0, new Rotation2d(Units.degreesToRadians(0))));
        path.preventFlipping = true;
        new TeleopBackInterrupter().deadlineWith(AutoBuilder.followPath(path)).schedule();
      } else {
        Pose2d currentPose = drivetrain.getPose();
        Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d(Math.PI / 2.0));// 10.52, 4.0
        Pose2d endPos = new Pose2d(6.10, 4.0100, new Rotation2d(Units.degreesToRadians(0)));// currentPose.getTranslation().plus(new
                                                                                            // Translation2d(1.0,
                                                                                            // 0.0)), new
                                                                                            // Rotation2d());
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
        PathPlannerPath path = new PathPlannerPath(
            bezierPoints,
            new PathConstraints(
                3.0, 2,
                Units.degreesToRadians(360), Units.degreesToRadians(540)),
            new GoalEndState(0.0, new Rotation2d(Units.degreesToRadians(180))));
        path.preventFlipping = true;
        new TeleopBackInterrupter().deadlineWith(AutoBuilder.followPath(path)).schedule();
      }
    });
    OI.stageLEFTAlign().rising().ifHigh(() -> {
      Optional<Alliance> teamColor = DriverStation.getAlliance();
      if (teamColor.get() == Alliance.Red) {
        Pose2d currentPose = drivetrain.getPose();
        Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d(Math.PI / 2.0));
        Pose2d endPos = new Pose2d(12.34, 2.8, new Rotation2d(Units.degreesToRadians(120.0)));// currentPose.getTranslation().plus(new
                                                                                              // Translation2d(1.0,
                                                                                              // 0.0)), new
                                                                                              // Rotation2d());
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
        PathPlannerPath path = new PathPlannerPath(
            bezierPoints,
            new PathConstraints(
                3.0, 2,
                Units.degreesToRadians(360), Units.degreesToRadians(540)),
            new GoalEndState(0.0, new Rotation2d(Units.degreesToRadians(120.0))));
        path.preventFlipping = true;
        new TeleopBumperInterupptor().deadlineWith(AutoBuilder.followPath(path)).schedule();
      } else {
        Pose2d currentPose = drivetrain.getPose();
        Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d(Math.PI / 2.0));
        Pose2d endPos = new Pose2d(4.21, 5.13, new Rotation2d(Units.degreesToRadians(120.0)));// currentPose.getTranslation().plus(new
                                                                                              // Translation2d(1.0,
                                                                                              // 0.0)), new
                                                                                              // Rotation2d());
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
        PathPlannerPath path = new PathPlannerPath(
            bezierPoints,
            new PathConstraints(
                3.0, 2,
                Units.degreesToRadians(360), Units.degreesToRadians(540)),
            new GoalEndState(0.0, new Rotation2d(Units.degreesToRadians(30.0 + 90.0 + 180))));
        path.preventFlipping = true;
        new TeleopBumperInterupptor().deadlineWith(AutoBuilder.followPath(path)).schedule();
      }
    });
    OI.stageRIGHTAlign().rising().ifHigh(() -> {
      Optional<Alliance> teamColor = DriverStation.getAlliance();
      if (teamColor.get() == Alliance.Red) {
        Pose2d currentPose = drivetrain.getPose();
        Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d(Math.PI / 2.0));// 10.52, 4.0
        Pose2d endPos = new Pose2d(12.25, 5.0100, new Rotation2d(Units.degreesToRadians(45)));// currentPose.getTranslation().plus(new
        // Translation2d(1.0,
        // 0.0)), new
        // Rotation2d());
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
        PathPlannerPath path = new PathPlannerPath(
            bezierPoints,
            new PathConstraints(
                3.0, 2,
                Units.degreesToRadians(360), Units.degreesToRadians(540)),
            new GoalEndState(0.0, new Rotation2d(Units.degreesToRadians(60 + 180))));
        path.preventFlipping = true;
        new TeleopStartinterupptor().deadlineWith(AutoBuilder.followPath(path)).schedule();
      } else {
        Pose2d currentPose = drivetrain.getPose();
        Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d(Math.PI / 2.0));// 10.52, 4.0
        Pose2d endPos = new Pose2d(4.27, 3.0, new Rotation2d(Units.degreesToRadians(45)));// currentPose.getTranslation().plus(new
                                                                                          // Translation2d(1.0,
                                                                                          // 0.0)), new
                                                                                          // Rotation2d());
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
        PathPlannerPath path = new PathPlannerPath(
            bezierPoints,
            new PathConstraints(
                3.0, 2,
                Units.degreesToRadians(360), Units.degreesToRadians(540)),
            new GoalEndState(0.0, new Rotation2d(Units.degreesToRadians(60))));
        path.preventFlipping = true;
        new TeleopStartinterupptor().deadlineWith(AutoBuilder.followPath(path)).schedule();
      }
    });

    OI.autoTrapDrive().rising().ifHigh(() -> {
      new TeleopAinterupptor().deadlineWith(Commands.sequence(
          new InstantCommand(() -> climber.setElevatorPosition(-86.5, 0)), // climber.setElevatorPosition(-80, 0)),
          new WaitCommand(0.5),
          // AutoBuilder.followPath(PathPlannerPath.fromPathFile("ChainEngage")),
          new TeleopDriveAutomated(drivetrain, 0, -0.7, 0.5),
          new WaitCommand(0.6),
          new TeleopDriveAutomated(drivetrain, 0, 0.0, 1.0),
          new InstantCommand(() -> climber.setElevatorPosition(-77, 0)), // climber.setElevatorPosition(-74, 0)),
          new WaitCommand(0.5),
          new TeleopDriveAutomated(drivetrain, 0, 0.6, 0.5),
          new WaitCommand(0.6),
          new TeleopDriveAutomated(drivetrain, 0, 0.0, 0.5),
          new InstantCommand(() -> intake.inIntake = false),
          new InstantCommand(() -> wrist.setWristPosition(Constants.Presets.ampWrist, 0)),
          new WaitCommand(0.5),
          new InstantCommand(() -> elevator.setElevatorPosition(Constants.Presets.trapElevator, 0)),
          new WaitCommand(0.5),
          new InstantCommand(() -> wrist.setWristPosition(Constants.Presets.trapWrist, 0)),
          new InstantCommand(() -> arm.setTargetAngle(Constants.Presets.ampArm, 0)),
          new WaitCommand(0.5),
          new TeleopDriveAutomated(drivetrain, 0, -0.6, 0.5),
          new WaitCommand(0.5),
          new TeleopDriveAutomated(drivetrain, 0, 0.0, 0.5),
          new InstantCommand(() -> climber.setElevatorPosition(-70, 0))

      )).schedule();
    });
    OI.ratchetEvent().rising().ifHigh(() -> {
      if (climber.ratchetEngaged == false) {
        climber.engageRatchet();
      } else {
        climber.disenageRatchet();
        Commands.sequence(
            new WaitCommand(1.0),
            new InstantCommand(() -> climber.setElevatorPosition(climber.getCurrentPosition() + 4, 0))).schedule();
      }
    });
    OI.toggleCurrentSensor().rising().ifHigh(() -> {
      System.out.println("OI Dispatched");
      intake.toggleCurrentSensing();
    });
    OI.autoTrap().rising().ifHigh(() -> Commands.sequence(
        new InstantCommand(() -> intake.inIntake = false),
        new InstantCommand(() -> elevator.setElevatorPosition(Constants.Presets.trapElevator, 0)),
        // new WaitCommand(1.0),//Waiting is for losers
        new InstantCommand(() -> wrist.setWristPosition(Constants.Presets.trapWrist, 0)),
        new InstantCommand(() -> arm.setTargetAngle(Constants.Presets.ampArm, 0)),
        // After in trap POS, CLIMB
        new InstantCommand(() -> climber.setElevatorPosition(0, 0)),
        new InstantCommand(() -> climber.engageRatchet()),
        new WaitCommand(1.5),
        new InstantCommand(() -> wrist.setWristPosition(Constants.Presets.trapWrist3, 0))).schedule());
    OI.autoDownClimb().rising().ifHigh(() -> Commands.sequence(
        new InstantCommand(() -> wrist.setWristPosition(Constants.Presets.trapWrist, 0)),
        new WaitCommand(0.1),
        new InstantCommand(() -> climber.setElevatorPosition(-72.0, 0))).schedule());

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
    return autoChooser.get();
  }
}
