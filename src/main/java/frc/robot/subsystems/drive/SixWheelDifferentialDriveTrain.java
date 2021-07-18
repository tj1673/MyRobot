package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;

public class SixWheelDifferentialDrivetrain {
    private final PWMSparkMax leftFront = new PWMSparkMax(DriveConstants.MotorChannels.LEFT_FRONT);
    private final PWMSparkMax leftMiddle = new PWMSparkMax(DriveConstants.MotorChannels.LEFT_MIDDLE);
    private final PWMSparkMax leftRear = new PWMSparkMax(DriveConstants.MotorChannels.LEFT_REAR);

    private final PWMSparkMax rightFront = new PWMSparkMax(DriveConstants.MotorChannels.RIGHT_FRONT);
    private final PWMSparkMax rightMiddle = new PWMSparkMax(DriveConstants.MotorChannels.RIGHT_MIDDLE);
    private final PWMSparkMax rightRear = new PWMSparkMax(DriveConstants.MotorChannels.RIGHT_REAR);

    private final SpeedControllerGroup leftGroup = new SpeedControllerGroup(leftFront, leftMiddle, leftRear);
    private final SpeedControllerGroup rightGroup = new SpeedControllerGroup(rightFront, rightMiddle, rightRear);

    private final DifferentialDrive drivetrain = new DifferentialDrive(leftGroup, rightGroup);

    public void arcadeDrive(double speed, double rotation) {
        drivetrain.arcadeDrive(speed, rotation);
    }

    public void setMaxOutput(double maxOutput) {
        drivetrain.setMaxOutput(maxOutput);
    }

    public double getLeftSpeed() {
        return leftGroup.get();
    }
    
    public double getRightSpeed() {
        return rightGroup.get();
    }
}
