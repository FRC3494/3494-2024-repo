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

    public static XboxController getPrimaryController() {
        return primaryController;
    }

    public static XboxController getDiannaRumbler() {
        return new XboxController(Constants.OI.DIANNA_RUMBLER_PORT);
    }

    public static void zeroControls() {
        // offset = -NavX.getYaw() - 90;
        offset = -NavX.getYaw();
    }

    public static double getDriveOffset() {
        return offset;
    }

    public static void setRedOffset() {
        offset = 91.6 + 180.0;
    }

    public static void setBlueOffset() {
        offset = 91.6;// 2.23;//-83.72;
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
        // return 0.1;
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

    public static double rawTeleopYVelocity() {
        double driveSpeed = slowMode() ? Constants.OI.SLOW_DRIVE_SPEED : Constants.OI.DRIVE_SPEED;
        double forward = primaryController.getLeftY();

        return forward * driveSpeed;

    }

    public static boolean slowMode() {
        return primaryController.getLeftBumper() || primaryController.getRightBumper();
        // return ((primaryController.getLeftTriggerAxis() >= 0.1) ||
        // (primaryController.getRightTriggerAxis() >= 0.1));
    }

    public static BooleanEvent resetHeadingEvent() {
        return primaryController.x(eventLoop);
    }

    public static boolean resetHeadingEventDUMB() {
        return primaryController.getXButtonPressed();
    }

    public static boolean autoAlignAMP() {
        return primaryController.getYButtonPressed();
    }

    public static boolean isYHeld() {
        return primaryController.getYButton();
    }

    public static boolean isAHeld() {
        return primaryController.getAButton();
    }

    public static boolean isBackHeld() {
        return primaryController.getBackButton();
    }

    public static boolean isStartHeld() {
        return primaryController.getStartButton();
    }

    public static boolean isBumperHeld() {
        return primaryController.getLeftBumper();
    }

    public static BooleanEvent noteAlign() {
        return primaryController.b(eventLoop);
    }

    public static double getElevatorPower() {
        double speed = rightButtonBoard.getRawAxis(0);
        // System.out.println(speed);
        return speed * Constants.Wrist.manualPowerAdjust;
        // TODO Auto-generated method stub
        // return
        // primaryController.getLeftTriggerAxis()-primaryController.getRightTriggerAxis();
    }

    public static double getArmPower() {
        double speed = leftButtonBoard.getRawAxis(1);
        // if (speed>0){
        // return speed * Constants.Arm.manualPowerAdjustUP;
        // }
        return speed * Constants.Arm.manualPowerAdjustDOWN;
    }

    public static double getWristPower() {
        double speed = rightButtonBoard.getRawAxis(1);
        // System.out.println(speed);
        return speed * Constants.Wrist.manualPowerAdjust;
        // return
        // primaryController.getLeftTriggerAxis()-primaryController.getRightTriggerAxis();

    }

    public static double getClimberPower() {
        // return 0;
        return primaryController.getLeftTriggerAxis() - primaryController.getRightTriggerAxis();
    }

    public static double getIntakePower() {
        // System.out.println(leftButtonBoard.getRawAxis(0)*Constants.Intake.manualPowerAdjust);
        return Math.abs(leftButtonBoard.getRawAxis(0) * -Constants.Intake.manualPowerAdjust)
                + (leftButtonBoard.getRawButton(7) ? -1.0 : 0)
                + (leftButtonBoard.getRawButton(9) ? -0.3 : 0)
                + (leftButtonBoard.getRawButton(10) ? 0.3 : 0);
    }

    public static BooleanEvent presetTest() {
        return primaryController.a(eventLoop);
    }

    public static boolean aButton() {
        return primaryController.getAButtonPressed();
    }

    public static BooleanEvent intakeReverse() {
        return leftButtonBoard.button(7, eventLoop);
    }

    public static BooleanEvent climbPreset() {
        return leftButtonBoard.button(8, eventLoop);
    }

    public static BooleanEvent liftPreset() {
        return leftButtonBoard.button(5, eventLoop);
    }

    public static BooleanEvent armPreset() {
        return leftButtonBoard.button(10, eventLoop);
    }

    public static BooleanEvent wristPreset() {
        return leftButtonBoard.button(7, eventLoop);
    }

    public static BooleanEvent pickupPreset() {
        return leftButtonBoard.button(2, eventLoop);
    }

    public static BooleanEvent ratchetEvent() {
        return rightButtonBoard.button(5, eventLoop).or(rightButtonBoard.button(10, eventLoop));
    }

    public static BooleanEvent toggleCurrentSensor() {
        System.out.println("Current Sensor Button Pressed; Event Dispatched");
        return rightButtonBoard.button(8, eventLoop);
    }

    public static BooleanEvent ampPreset() {
        return leftButtonBoard.button(1, eventLoop);
    }

    public static BooleanEvent trapPreset() {
        return rightButtonBoard.button(1, eventLoop);
    }

    public static BooleanEvent trapPreset2() {
        return rightButtonBoard.button(2, eventLoop);
    }

    public static BooleanEvent autoTrap() {
        return rightButtonBoard.button(3, eventLoop);
    }

    public static BooleanEvent autoDownClimb() {
        return rightButtonBoard.button(4, eventLoop);
    }

    public static BooleanEvent storePreset() {
        return leftButtonBoard.button(4, eventLoop);
    }

    public static BooleanEvent autoTrapDrive() {
        return primaryController.a(eventLoop);
    }

    public static BooleanEvent stageRIGHTAlign() {
        return primaryController.start(eventLoop);
    }

    public static BooleanEvent stageBACKAlign() {
        return primaryController.back(eventLoop);
    }

    public static BooleanEvent stageLEFTAlign() {
        // return primaryController.start(eventLoop);
        return primaryController.leftBumper(eventLoop);
    }

    public static EventLoop getEventLoop() {
        return eventLoop;
    }
}
