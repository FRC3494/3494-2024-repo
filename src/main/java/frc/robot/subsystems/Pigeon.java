package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import java.util.OptionalDouble;
import java.util.Queue;

public class Pigeon {
    private static final Pigeon2 pigeon = new Pigeon2(Constants.Drivetrain.PIGEON_PORT);
    private static final StatusSignal<Double> yaw = pigeon.getYaw();

    public Pigeon(){
        pigeon.setYaw(0);
    }
    public double getYaw() {
        return pigeon.getYaw().getValueAsDouble()%360.0;
    }

}
