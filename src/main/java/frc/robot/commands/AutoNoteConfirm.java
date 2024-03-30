package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class AutoNoteConfirm extends Command {
    Intake intake;

    public AutoNoteConfirm(Intake intake) {
        this.intake = intake;
    }

    @Override
    public boolean isFinished() {
        if (intake.hasNote())
            return true;
        return false;

    }
}
