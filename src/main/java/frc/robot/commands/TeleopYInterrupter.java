package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;

public class TeleopYInterrupter extends Command {
	@Override
	public boolean isFinished() {
		if (OI.isYHeld()) {
			return false;
		}

		return true;
	}
}
