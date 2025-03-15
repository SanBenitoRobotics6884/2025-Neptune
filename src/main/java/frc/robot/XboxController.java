package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class XboxController implements ControllerInterface {
    private Joystick joystick;

    // Controller Button Mappings
    public static final int BUTTON_A = 2;
    public static final int BUTTON_B = 3;
    public static final int BUTTON_X = 4;
    public static final int BUTTON_Y = 5;
    public static final int BUTTON_LEFT_BUMPER = 7;
    public static final int BUTTON_RIGHT_BUMPER = 8;
    public static final int BUTTON_BACK = 7;
    public static final int BUTTON_START = 8;
    public static final int BUTTON_LEFT_STICK = 9;
    public static final int BUTTON_RIGHT_STICK = 10;

    // Controller Axis Mappings
    public static final int AXIS_LEFT_X = 0;
    public static final int AXIS_LEFT_Y = 1;
    public static final int AXIS_RIGHT_X = 2;
    public static final int AXIS_RIGHT_Y = 3;
    public static final int AXIS_LEFT_TRIGGER = 4;
    public static final int AXIS_RIGHT_TRIGGER = 5;

    public XboxController(int port) {
        joystick = new Joystick(port);
    }

    public boolean getButton(int button) {
        return joystick.getRawButton(button);
    }

    public double getAxis(int axis) {
        return joystick.getRawAxis(axis);
    }

    public boolean getButtonA() {
        return getButton(BUTTON_A);
    }

    public boolean getButtonB() {
        return getButton(BUTTON_B);
    }

    public boolean getButtonX() {
        return getButton(BUTTON_X);
    }

    public boolean getButtonY() {
        return getButton(BUTTON_Y);
    }

    public boolean getLeftBumper() {
        return getButton(BUTTON_LEFT_BUMPER);
    }

    public boolean getRightBumper() {
        return getButton(BUTTON_RIGHT_BUMPER);
    }

    public boolean getBackButton() {
        return getButton(BUTTON_BACK);
    }

    public boolean getStartButton() {
        return getButton(BUTTON_START);
    }

    public boolean getLeftStickButton() {
        return getButton(BUTTON_LEFT_STICK);
    }

    public boolean getRightStickButton() {
        return getButton(BUTTON_RIGHT_STICK);
    }

    public double getLeftXAxis() {
        return getAxis(AXIS_LEFT_X);
    }

    public double getLeftYAxis() {
        return getAxis(AXIS_LEFT_Y);
    }

    public double getRightXAxis() {
        return getAxis(AXIS_RIGHT_X);
    }

    public double getRightYAxis() {
        return getAxis(AXIS_RIGHT_Y);
    }

    public double getLeftTriggerAxis() {
        return getAxis(AXIS_LEFT_TRIGGER);
    }

    public double getRightTriggerAxis() {
        return getAxis(AXIS_RIGHT_TRIGGER);
    }
}