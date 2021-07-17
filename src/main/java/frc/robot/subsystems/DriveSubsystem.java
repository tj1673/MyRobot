// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  private final PWMSparkMax leftFront = new PWMSparkMax(DriveConstants.MotorChannels.LEFT_FRONT);
  private final PWMSparkMax leftMiddle = new PWMSparkMax(DriveConstants.MotorChannels.LEFT_MIDDLE);
  private final PWMSparkMax leftRear = new PWMSparkMax(DriveConstants.MotorChannels.LEFT_REAR);
  
  private final PWMSparkMax rightFront = new PWMSparkMax(DriveConstants.MotorChannels.RIGHT_FRONT);
  private final PWMSparkMax rightMiddle = new PWMSparkMax(DriveConstants.MotorChannels.RIGHT_MIDDLE);
  private final PWMSparkMax rightRear = new PWMSparkMax(DriveConstants.MotorChannels.RIGHT_REAR);

  private final SpeedControllerGroup leftGroup = new SpeedControllerGroup(leftFront, leftMiddle, leftRear);
  private final SpeedControllerGroup rightGroup = new SpeedControllerGroup(rightFront, rightMiddle, rightRear);

  private final DifferentialDrive driveTrain = new DifferentialDrive(leftGroup, rightGroup);

  private final Encoder leftEncoder = new Encoder(DriveConstants.EncoderChannels.LEFT_A, DriveConstants.EncoderChannels.LEFT_B);
  private final Encoder rightEncoder = new Encoder(DriveConstants.EncoderChannels.RIGHT_A, DriveConstants.EncoderChannels.RIGHT_B);

  private final AnalogGyro gyro = new AnalogGyro(DriveConstants.GYRO_CHANNEL);
  private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
  
  private final DifferentialDrivetrainSim driveTrainSim = DifferentialDrivetrainSim.createKitbotSim(
    KitbotMotor.kDualCIMPerSide,
    KitbotGearing.k10p71,        
    KitbotWheelSize.SixInch,     
    null                         
  );
  
  private final AnalogGyroSim gyroSim = new AnalogGyroSim(gyro);
  private final EncoderSim leftEncoderSim = new EncoderSim(leftEncoder);
  private final EncoderSim rightEncoderSim = new EncoderSim(rightEncoder);
  private final Field2d field = new Field2d();

  public DriveSubsystem() {
    setSlowSpeed();
    initEncoders();
    initDashboard();
  }

  @Override
  public void periodic() {
    Pose2d pose = updateOdometry();
    updateRobotPositionOnField(pose);
  }

  public void manualDrive(double speed, double rotation) {
    driveTrain.arcadeDrive(speed, rotation);
  }

  public void setSlowSpeed() {
    driveTrain.setMaxOutput(DriveConstants.SLOW_SPEED);
  }

  public void setFastSpeed() {
    driveTrain.setMaxOutput(DriveConstants.FAST_SPEED);
  }

  public void simulationPeriodic() {
    double leftVoltageVolts = leftGroup.get() * RobotController.getInputVoltage();
    double rightVoltageVolts = -rightGroup.get() * RobotController.getInputVoltage();

    driveTrainSim.setInputs(leftVoltageVolts, rightVoltageVolts);
    driveTrainSim.update(DriveConstants.SIMULATION_REFRESH_RATE);

    leftEncoderSim.setDistance(driveTrainSim.getLeftPositionMeters());
    leftEncoderSim.setRate(driveTrainSim.getLeftVelocityMetersPerSecond());
    rightEncoderSim.setDistance(driveTrainSim.getRightPositionMeters());
    rightEncoderSim.setRate(driveTrainSim.getRightVelocityMetersPerSecond());
    gyroSim.setAngle(-driveTrainSim.getHeading().getDegrees());
  }

  private void initEncoders() {
    double distancePerPulse = 2 * Math.PI * DriveConstants.WHEEL_RADIUS / DriveConstants.ENCODER_RESOLUTION;
    
    leftEncoder.setDistancePerPulse(distancePerPulse);
    rightEncoder.setDistancePerPulse(distancePerPulse);

    leftEncoder.reset();
    rightEncoder.reset();
  }

  private void  initDashboard() {
    SmartDashboard.putData("Field", field);
  }

  private Pose2d updateOdometry() {
    odometry.update(
      gyro.getRotation2d(),
      leftEncoder.getDistance(),
      rightEncoder.getDistance()
    );

    return odometry.getPoseMeters();
  }

  private void updateRobotPositionOnField(Pose2d pose) {
    field.setRobotPose(pose);
  }
}
