package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;

public class TeleopStartinterupptor extends Command {
	@Override
	public boolean isFinished() {
		if (OI.isStartHeld()) {
			return false;
		}

		return true;
	}
}
