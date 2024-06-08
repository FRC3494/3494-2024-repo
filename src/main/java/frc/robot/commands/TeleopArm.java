package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.subsystems.Arm;

public class TeleopArm extends Command {
    Arm arm;
    private double armPower = 0;

    public TeleopArm(Arm arm) {
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        armPower = OI.deadband(OI.getArmPower(), 0.05);
        // if(armPower != 0 || (arm.getManualMotorPower() != 0 && armPower == 0)){
        // arm.setMotorPower(armPower);
        // }
        if (armPower != 0 || (arm.getManualMotorPower() != 0 && armPower == 0)) {
            arm.setTargetAngle(arm.getTargetPosition() + armPower * Constants.Arm.manualPowerPOS, 0);
        }

    }

}
