// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pigeon;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public final Drivetrain drivetrain;

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
    configureBindings();

    drivetrain.setDefaultCommand(new TeleopDrive(drivetrain));

    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`

    initShuffleboardObjects();
  }

  // public void periodic() {
  // eventLoop.poll();
  // }
  public void initShuffleboardObjects() {
    fieldTab = Shuffleboard.getTab("Field");
    subsystemTab = Shuffleboard.getTab("Subsystems");

    subsystemTab.add("Auto Mode", autoChooser);

    robotPosition = new Field2d();
    robotPosition.setRobotPose(new Pose2d());

    fieldTab.add(robotPosition).withPosition(1, 0).withSize(7, 4);
    fieldTab.addDouble("NavX yaw", () -> Pigeon.getYaw());
    fieldTab.addDouble("OFFFSET", () -> OI.getDriveOffset());
  }

  public void updateShuffleboardObjects() {
    // System.out.println(OI.presetTest());
    robotPosition.setRobotPose(drivetrain.getPoseCorrected());
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}
