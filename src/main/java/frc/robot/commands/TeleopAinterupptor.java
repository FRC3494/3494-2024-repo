package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;

public class TeleopAinterupptor extends Command {
	@Override
	public boolean isFinished() {
		if (OI.isAHeld()) {
			return false;
		}

		return true;
	}
}
