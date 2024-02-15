// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist.Wrist;
import frc.robot.commands.TeleopArm;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.TeleopElevator;
// import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.Elevator.Elevator;

import java.util.List;

import edu.wpi.first.math.util.Units;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final Drivetrain drivetrain;
  // public final Arm arm;
  
  // public final Elevator elevator;
  // public final Wrist wrist;
  // public final Camera camera;
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
    // elevator = new Elevator();
    // wrist = new Wrist();
    // arm = new Arm();
    // camera = new Camera();
    drivetrain.setDefaultCommand(new TeleopDrive(drivetrain));
    // elevator.setDefaultCommand(new TeleopElevator(elevator));
    // arm.setDefaultCommand(new TeleopArm(arm));
    NamedCommands.registerCommand("Print Command",
        new PrintCommand("AUTO HAS TRIGGERED A PRINT COMAND WOOOOOOOOOOOOOOO"));

    // Configure the trigger bindings
    configureBindings();
    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);
    initShuffleboardObjects();
  }

  public void initShuffleboardObjects() {
    fieldTab = Shuffleboard.getTab("Field");
    subsystemTab = Shuffleboard.getTab("Subsystems");
    robotPosition = new Field2d();
    robotPosition.setRobotPose(new Pose2d());
    fieldTab.add(robotPosition).withPosition(1, 0).withSize(7, 4);
    fieldTab.addDouble("NavX yaw", () -> NavX.getYaw());
    fieldTab.addDouble("OFFFSET", () -> OI.getDriveOffset());
    // subsystemTab.addDouble("Current Arm Position", () -> arm.getCurrentAngle());
    

    // fieldTab.add(camera.getCamera()).withPosition(1, 5).withSize(4, 4);
  }

  public void updateShuffleboardObjects() {
    robotPosition.setRobotPose(drivetrain.getPoseCorrected());
  }

  private void configureBindings() {
    // OI.presetTest().rising().ifHigh(()->{
    //   elevator.setElevatorPosition(Constants.Presets.testElevator,0);
    //   arm.setTargetAngle(Constants.Presets.testArm);
    //   wrist.setWristPosition(Constants.Presets.testWrist, 0);
    //   System.out.println("RANN COMMAND---------------------------------------");
    // });
    SmartDashboard.putData("On-the-fly path", Commands.runOnce(() -> {
      Pose2d currentPose = drivetrain.getPose();

      // The rotation component in these poses represents the direction of travel
      Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
      Pose2d endPos = new Pose2d(14.5, 7.7, new Rotation2d());//currentPose.getTranslation().plus(new Translation2d(1.0, 0.0)), new Rotation2d());
      List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
      PathPlannerPath path = new PathPlannerPath(
          bezierPoints,
          new PathConstraints(
              1.0, 1.0,
              Units.degreesToRadians(360), Units.degreesToRadians(540)),
          new GoalEndState(0.0, currentPose.getRotation()));

      // }
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
