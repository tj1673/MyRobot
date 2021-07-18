package frc.robot.operatorinterface;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class DualShock4Controller extends GenericHID {

    public DualShock4Controller(int controllerPort) {
        super(controllerPort);
    }
    
    @Override
    public double getX(Hand hand) {
        switch (hand) {
            case kLeft:
                return this.getRawAxis(ButtonMap.LeftXAxis);
            case kRight:
                return this.getRawAxis(ButtonMap.RightXAxis);
            default:
                return 0;
        }
    }

    @Override
    public double getY(Hand hand) {
        switch (hand) {
            case kLeft:
                return -this.getRawAxis(ButtonMap.LeftYAxis);
            case kRight:
                return -this.getRawAxis(ButtonMap.RightYAxis);
            default:
                return 0;
        }
    }    

    public JoystickButton getLeftBumper() {
        return new JoystickButton(this, ButtonMap.LeftBumper);
    }

    public JoystickButton getLeftStick() {
        return new JoystickButton(this, ButtonMap.LeftStick);
    }

    private final class ButtonMap {
        public static final int LeftXAxis = 0;
        public static final int LeftYAxis = 1;
        public static final int LeftBumper = 5;
        public static final int LeftStick = 10;
        public static final int RightXAxis = 4;
        public static final int RightYAxis = 5;
    }
}
