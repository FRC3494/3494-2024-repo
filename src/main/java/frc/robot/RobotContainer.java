// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber.RatchetServo;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private ShuffleboardTab kennanTab;
  private GenericEntry servoValue;
  private RatchetServo ratchetServo = new RatchetServo(0);

  public RobotContainer() {
    configureBindings();
    initShuffleboardObjects();
  }

  public void periodic() {
    double value = servoValue.getDouble(0);

    ratchetServo.set(value);
  }

  public void initShuffleboardObjects() {
    kennanTab = Shuffleboard.getTab("Kennan");
    servoValue = kennanTab.add("servo value", 0).getEntry();
  }

  public void updateShuffleboardObjects() {
  }

  private void configureBindings() {
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
