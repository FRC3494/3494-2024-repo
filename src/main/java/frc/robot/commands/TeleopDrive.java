
package frc.robot.commands;

import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.Drivetrain;

public class TeleopDrive extends Command {
    Drivetrain drivetrain;
    PathPlannerPath path;
    public static boolean NoteAligning = false;

    public TeleopDrive(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        if (!NoteAligning) {
            // System.out.println(OI.teleopXVelocity());
            drivetrain.drive(OI.teleopXVelocity(), OI.teleopYVelocity(),
                    -OI.teleopTurnVelocity(), true);
            // drivetrain.drive(0, 0.1,
            // 0, false);
            if (OI.resetHeadingEventDUMB()) {
                OI.zeroControls();
            }
        } else {
            // WHEN we are NOTE aligning
            // drivetrain.drive(-OI.teleopYVelocity(), -OI.teleopXVelocity(),
            // -drivetrain.getNoteRotationPower(), false);
            // drivetrain.drive( OI.teleopXVelocity(),OI.teleopYVelocity(),
            // -drivetrain.getNoteRotationPower(), false);
            drivetrain.drive(0.0, OI.rawTeleopYVelocity(),
                    -drivetrain.getNoteRotationPowerPOWERFUL(), false);
        }

        if (OI.autoAlignAMP()) {
            Optional<Alliance> teamColor = DriverStation.getAlliance();
            if (teamColor.isPresent()) {
                if (teamColor.get() == Alliance.Red) {
                    Pose2d currentPose = drivetrain.getPose();
                    // The rotation component in these poses represents the direction of travel
                    Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d(Math.PI / 2.0));
                    Pose2d endPos = new Pose2d(14.64, 7.8, new Rotation2d(Math.PI / 2.0));// currentPose.getTranslation().plus(new
                                                                                          // Translation2d(1.0, 0.0)),
                                                                                          // new
                                                                                          // Rotation2d());
                    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
                    PathPlannerPath path = new PathPlannerPath(
                            bezierPoints,
                            new PathConstraints(
                                    1.5, 2.0,
                                    Units.degreesToRadians(360), Units.degreesToRadians(540)),
                            new GoalEndState(0.0, new Rotation2d(Math.PI / 2.0)));
                    path.preventFlipping = true;
                    new TeleopYInterupptor().deadlineWith(AutoBuilder.followPath(path)).schedule();
                } else {// (teamColor.get() == Alliance.Blue){
                    Pose2d currentPose = drivetrain.getPose();
                    // The rotation component in these poses represents the direction of travel
                    Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d(Math.PI / 2.0));
                    Pose2d endPos = new Pose2d(1.8, 7.8, new Rotation2d(Math.PI / 2.0));// currentPose.getTranslation().plus(new
                                                                                        // Translation2d(1.0, 0.0)), new
                                                                                        // Rotation2d());
                    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
                    PathPlannerPath path = new PathPlannerPath(
                            bezierPoints,
                            new PathConstraints(
                                    1.5, 2.0,
                                    Units.degreesToRadians(360), Units.degreesToRadians(540)),
                            new GoalEndState(0.0, new Rotation2d(Math.PI / 2.0)));
                    path.preventFlipping = true;
                    new TeleopYInterupptor().deadlineWith(AutoBuilder.followPath(path)).schedule();
                }

            }

            // AutoBuilder.followPath(path).deadlineWith(new
            // TeleopYinterupptor()).schedule();
        }

    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(0, 0, 0, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
