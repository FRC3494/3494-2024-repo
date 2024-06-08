package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopRumble extends Command {
    private XboxController rumbler;
    private Timer timer;
    private double length;

    public TeleopRumble(XboxController rumbler, double length) {
        this.rumbler = rumbler;
        this.timer = new Timer();
        this.timer.start();
        this.length = length;
    }

    @Override
    public void execute() {
        rumbler.setRumble(RumbleType.kBothRumble, 1.0);
    }

    @Override
    public boolean isFinished() {
        if (timer.hasElapsed(this.length)) {
            rumbler.setRumble(RumbleType.kBothRumble, 0.0);
            return true;
        }
        return false;
    }
}
