package frc.robot.operatorinterface;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class DualShock4Controller {

    public DualShock4Controller(int controllerPort) {
        joystick = new Joystick(controllerPort);
    }
    
    public double getLeftYAxis() {
        return -joystick.getRawAxis(ButtonMap.LeftYAxis);
    }

    public double getRightXAxis() {
        return joystick.getRawAxis(ButtonMap.RightXAxis);
    }

    public JoystickButton getLeftBumper() {
        return new JoystickButton(joystick, ButtonMap.LeftBumper);
    }

    public JoystickButton getLeftStick() {
        return new JoystickButton(joystick, ButtonMap.LeftStick);
    }


    private final Joystick joystick;

    private final class ButtonMap {
        public static final int LeftYAxis = 1;
        public static final int LeftBumper = 5;
        public static final int LeftStick = 10;
        public static final int RightXAxis = 4;
        
    }
}
