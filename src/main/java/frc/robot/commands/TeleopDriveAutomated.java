package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class TeleopDriveAutomated extends Command {
    private Timer timer;
    private Drivetrain drivetrain;
    private double xPow;
    private double yPow;

    public TeleopDriveAutomated(Drivetrain drivetrain, double xPow, double yPow, double time) {
        this.drivetrain = drivetrain;

        this.timer = new Timer();
        this.timer.start();
        addRequirements(drivetrain);
        this.xPow = xPow;
        this.yPow = yPow;
    }

    @Override
    public void execute() {
        drivetrain.drive(xPow, yPow,
                0.0, false);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
