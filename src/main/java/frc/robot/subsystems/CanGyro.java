package frc.robot.subsystems;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;;

public class CanGyro {
    Canandgyro gyro = new Canandgyro(0);
    double offset = 0;

    public CanGyro() {
        offset = gyro.getYaw();
    }

    public double getYaw() {
        return ((gyro.getYaw() - offset))*360;
    }

}
