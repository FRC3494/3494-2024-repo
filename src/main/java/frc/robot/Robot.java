// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private boolean endOfMatchTriggered = false; 

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    // m_robotContainer.intake.distOnboard.setAutomaticMode(true);

    loggingInit();
  }

  @SuppressWarnings({ "all", "resource" })
  public void loggingInit() {
    Logger.recordMetadata("Project Name", "3494-2024"); // Set a metadata value

    Logger.recordMetadata("Build Time", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("Build Timestamp", Long.toString(BuildConstants.BUILD_UNIX_TIME));

    Logger.recordMetadata("Git SHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("Git Commit Time", BuildConstants.GIT_DATE);
    Logger.recordMetadata("Git Branch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata("Git Status", BuildConstants.DIRTY == 1 ? "Dirty" : "Clean");

    Logger.recordMetadata("Event Name", DriverStation.getEventName());

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables

      new PowerDistribution(Constants.PDH_CAN_ID, ModuleType.kRev); // Enables power distribution logging
    } else {
      setUseTiming(false); // Run as fast as possible
      String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)

      Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    Logger.start();
  }


  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    OI.update();

    // Run the end of match subroutine
    double timeTillEndOfMatch = Timer.getMatchTime();
    boolean timeIsValid = timeTillEndOfMatch != -1;
    boolean withinEndOfMatch = timeTillEndOfMatch < Constants.END_OF_MATCH_ROUTINE_STARTING_TIME;

    if (isTeleop() && withinEndOfMatch && timeIsValid && !endOfMatchTriggered) {
      endOfMatch();
      endOfMatchTriggered = true;
    }

    // System.out.println(endOfMatchTriggered ? "End of match triggered" : "end of match Not yet");

    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    m_robotContainer.updateShuffleboardObjects();
  }

  /**
   * This function is called within a second count from the end of the estimated match time
   * defined by the value of Constants.END_OF_MATCH_ROUTINE_STARTING_TIME
   * 
   * @see Constants
   */
  public void endOfMatch() {
    m_robotContainer.climber.engageRatchet();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    // m_robotContainer.climber.engageRatchet();
    
    // m_robotContainer.intake.distOnboard.setAutomaticMode(false);
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    endOfMatchTriggered = false;
    m_robotContainer.climber.disenageRatchet();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    endOfMatchTriggered = false;
    m_robotContainer.climber.disenageRatchet();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    Optional<Alliance> teamColor = DriverStation.getAlliance();
    if(teamColor.isPresent()){
      if(teamColor.get() == Alliance.Red){
        OI.setRedOffset();
      }
      else if(teamColor.get() == Alliance.Blue){
        OI.setBlueOffset();
      }
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
