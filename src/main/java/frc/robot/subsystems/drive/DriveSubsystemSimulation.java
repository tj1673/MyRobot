package frc.robot.subsystems.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystemSimulation {
    private final DifferentialDrivetrainSim driveTrainSim = DifferentialDrivetrainSim.createKitbotSim(
        KitbotMotor.kDualCIMPerSide,
        KitbotGearing.k10p71,        
        KitbotWheelSize.SixInch,     
        null                         
      );
      
      private AnalogGyroSim gyroSim;
      private EncoderSim leftEncoderSim;
      private EncoderSim rightEncoderSim;
      private DoubleSupplier leftSpeed;
      private DoubleSupplier rightSpeed;
      
      public DriveSubsystemSimulation(
          AnalogGyro gyro,
          Encoder leftEncoder,
          Encoder rightEncoder,
          DoubleSupplier _leftSpeed,
          DoubleSupplier _rightSpeed
      ) {
        gyroSim = new AnalogGyroSim(gyro);
        leftEncoderSim = new EncoderSim(leftEncoder);
        rightEncoderSim = new EncoderSim(rightEncoder);
        leftSpeed = _leftSpeed;
        rightSpeed = _rightSpeed;
      }

      public void update() {
        double leftVoltageVolts = leftSpeed.getAsDouble() * RobotController.getInputVoltage();
        double rightVoltageVolts = -rightSpeed.getAsDouble() * RobotController.getInputVoltage();
    
        driveTrainSim.setInputs(leftVoltageVolts, rightVoltageVolts);
        driveTrainSim.update(DriveConstants.SIMULATION_REFRESH_RATE);
    
        leftEncoderSim.setDistance(driveTrainSim.getLeftPositionMeters());
        leftEncoderSim.setRate(driveTrainSim.getLeftVelocityMetersPerSecond());
        rightEncoderSim.setDistance(driveTrainSim.getRightPositionMeters());
        rightEncoderSim.setRate(driveTrainSim.getRightVelocityMetersPerSecond());
        gyroSim.setAngle(-driveTrainSim.getHeading().getDegrees());
      }
}
