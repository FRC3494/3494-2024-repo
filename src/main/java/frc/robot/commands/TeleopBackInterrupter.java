package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;

public class TeleopBackInterrupter extends Command {
	@Override
	public boolean isFinished() {
		if (OI.isBackHeld()) {
			return false;
		}

		return true;
	}
}
