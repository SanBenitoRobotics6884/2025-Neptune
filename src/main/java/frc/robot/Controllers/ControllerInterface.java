package frc.robot.Controllers;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public interface ControllerInterface {
    public default boolean getButton(int button) {return false;}

    public default  double getAxis(int axis) {
        return 0;
    }

    public default int getPOV(){
        return -1;
    }

    public default  boolean getButtonA() {
        return getButton(1);
    }

    public default  boolean getButtonB() {
        return getButton(2);
    }

    public default  boolean getButtonX() {
        return getButton(3);
    }

    public default  boolean getButtonY() {
        return getButton(4);
    }

    public default  boolean getLeftBumper() {
        return getButton(5);
    }

    public default  boolean getRightBumper() {
        return getButton(6);
    }

    public default  boolean getButtonBack() {
        return getButton(7);
    }

    public default  boolean getButtonStart() {
        return getButton(8);
    }

    public default  boolean getButtonLeftStick() {
        return getButton(9);
    }

    public default  boolean getButtonRightStick() {
        return getButton(10);
    }

    public default  boolean getButtonDPadUp() {
        return getButton(11);
    }
    public default  boolean getButtonDPadDown() {
        return getButton(12);
    }
    public default  boolean getButtonDPadLeft() {
        return getButton(13);
    }
    public default  boolean getButtonDPadRight() {
        return getButton(14);
    }

    public default  double getLeftXAxis() {
        return getAxis(0);
    }

    public default  double getLeftYAxis() {
        return getAxis(1);
    }

    public default  double getRightXAxis() {
        return getAxis(4);
    }

    public default  double getRightYAxis() {
        return getAxis(5);
    }

    public default  double getLeftTrigger() {
        return getAxis(2);
    }

    public default  double getRightTrigger() {
        return getAxis(3);
    }
}
