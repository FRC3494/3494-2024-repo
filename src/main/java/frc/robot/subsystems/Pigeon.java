package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Pigeon {
    private static Pigeon2 pigeon = new Pigeon2(Constants.Drivetrain.PigeonID);

    public static double getYaw() {
        return pigeon.getAngle() % 360;
    }

    public static double getPitch() {
        return pigeon.getPitch().getValueAsDouble();
    }

    public static double getRoll() {
        return pigeon.getPitch().getValueAsDouble();
    }

    public static void putShuffleBoardData() {
        SmartDashboard.putNumber("Current Angle", pigeon.getYaw().getValueAsDouble());
    }

    public static void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Gyro");
        builder.addDoubleProperty("Value", Pigeon::getYaw, null);
    }
}