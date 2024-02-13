package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import frc.robot.subsystems.NavX;

public final class OI {
    private static EventLoop eventLoop = new EventLoop();
    private static XboxController primaryController = new XboxController(Constants.OI.PRIMARY_CONTROLLER_PORT);
    private static Joystick leftButtonBoard = new Joystick(Constants.OI.SECONDARY_LEFT_CONTROLLER_PORT);
    private static Joystick rightButtonBoard = new Joystick(Constants.OI.SECONDARY_RIGHT_CONTROLLER_PORT);
    private static double offset = 0;

    
    public static void zeroControls() {
        // offset = -NavX.getYaw() - 90;
        offset = -NavX.getYaw();
    }

    public static double getDriveOffset() {
        return offset;
    }

    public static void setRedOffset() {
        offset = 95.05;
    }

    public static void setBlueOffset() {
        offset = -83.72;
    }

    public static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            }

            return (value + deadband) / (1.0 - deadband);
        }

        return 0.0;
    }

    private static double modifyAxis(double value) {
        // Deadband
        value = deadband(value, 0.0001);

        // Square the axis
        value = Math.copySign(Math.pow(value, 3), value);

        return value;
    }

    public static void update() {
        eventLoop.poll();
    }

    public static boolean isDPadPressed() {
        return (primaryController.getPOV() != -1);
    }

    public static double teleopXVelocity() {
        double driveSpeed = slowMode() ? Constants.OI.SLOW_DRIVE_SPEED : Constants.OI.DRIVE_SPEED;
        double forward = primaryController.getLeftY();
        double left = primaryController.getLeftX();
        double dPadPower = ((primaryController.getPOV() == 180) ? -Constants.OI.DPAD_SPEED : 0.0)
                + ((primaryController.getPOV() == 0) ? Constants.OI.DPAD_SPEED : 0.0);

        double angle = (Math.atan2(forward, left) + Math.toRadians(offset)) % (2 * Math.PI);
        driveSpeed += dPadPower;
        double velocity = Math.min(Math.sqrt(Math.pow(forward, 2) + Math.pow(left, 2) + Math.pow(dPadPower, 2)),
                driveSpeed);

        if (dPadPower == 0) {
            return modifyAxis(-velocity) * Math.cos(angle) * driveSpeed;
        } else {
            return dPadPower;
        }

    }

    public static double teleopYVelocity() {
        double driveSpeed = slowMode() ? Constants.OI.SLOW_DRIVE_SPEED : Constants.OI.DRIVE_SPEED;
        double forward = primaryController.getLeftY();
        double left = primaryController.getLeftX();
        double dPadPower = ((primaryController.getPOV() == 90) ? -Constants.OI.DPAD_SPEED : 0)
                + ((primaryController.getPOV() == 270) ? Constants.OI.DPAD_SPEED : 0);

        double angle = (Math.atan2(forward, left) + Math.toRadians(offset)) % (2 * Math.PI);
        driveSpeed += dPadPower;
        double velocity = Math.min(Math.sqrt(Math.pow(forward, 2) + Math.pow(left, 2)), driveSpeed);

        if (dPadPower == 0) {
            return modifyAxis(velocity) * Math.sin(angle) * driveSpeed;
        } else {
            return dPadPower;
        }
    }

    public static double teleopTurnVelocity() {
        double turnSpeed = slowMode() ? Constants.OI.SLOW_TURN_SPEED : Constants.OI.TURN_SPEED;
        return modifyAxis(primaryController.getRightX()) * turnSpeed;
    }

    public static boolean slowMode() {
        return ((primaryController.getLeftTriggerAxis() >= 0.1) || (primaryController.getRightTriggerAxis() >= 0.1));
    }

    public static BooleanEvent resetHeadingEvent() {
        return primaryController.x(eventLoop);
    }

    public static boolean resetHeadingEventDUMB() {
        return primaryController.getXButtonPressed();
    }
    public static boolean autoAlign(){
        return primaryController.getYButtonPressed();
    }
    public static boolean isYHeld(){
        return primaryController.getYButton();
    }

    public static double getElevatorPower() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getElevatorPower'");
    }

    public static double getClimberPower() {
        return 0;
    }
}
