package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.Wrist.Wrist;

public class TeleopWrist extends Command {
    private Wrist wrist;
    private double wristPower = 0;

    public TeleopWrist(Wrist wrist) {
        this.wrist = wrist;
        addRequirements(wrist);
    }

    @Override
    public void execute() {
        wristPower = OI.deadband(OI.getWristPower(), 0.05);
        // System.out.println(wristPower);
        if (wristPower != 0 || (wrist.getManualMotorPower() != 0 && wristPower == 0)) {
            wrist.setWristPower(wristPower);
        }

    }

}
