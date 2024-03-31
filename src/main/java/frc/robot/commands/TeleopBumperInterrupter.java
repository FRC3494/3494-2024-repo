package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;

public class TeleopBumperInterrupter extends Command {
	@Override
	public boolean isFinished() {
		if (OI.isBumperHeld()) {
			return false;
		}

		return true;
	}
}
