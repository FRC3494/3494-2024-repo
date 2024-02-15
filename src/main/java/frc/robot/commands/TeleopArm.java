package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator.Elevator;

public class TeleopArm extends Command {
    Arm arm;
    private double armPower = 0;
    public TeleopArm(Arm arm){
        this.arm = arm;
        addRequirements(arm);
    }
    @Override
    public void execute() {
        armPower = OI.deadband(OI.getArmPower(), 0.05);
        arm.setMotorPower(armPower);

    }
    
}
