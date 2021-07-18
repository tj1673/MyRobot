// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  private final SixWheelDifferentialDrivetrain drivetrain = new SixWheelDifferentialDrivetrain();
  private final Encoder leftEncoder = new Encoder(DriveConstants.EncoderChannels.LEFT_A, DriveConstants.EncoderChannels.LEFT_B);
  private final Encoder rightEncoder = new Encoder(DriveConstants.EncoderChannels.RIGHT_A, DriveConstants.EncoderChannels.RIGHT_B);
  private final AnalogGyro gyro = new AnalogGyro(DriveConstants.GYRO_CHANNEL);
  private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(gyro.getRotation2d());

  private final DriveSubsystemSimulation simulation = new DriveSubsystemSimulation(
    gyro,
    leftEncoder,
    rightEncoder,
    () -> drivetrain.getLeftSpeed(),
    () -> drivetrain.getRightSpeed()
  );

  private Field2d field = new Field2d();

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

  public void arcadeDrive(double speed, double rotation) {
    drivetrain.arcadeDrive(speed, rotation);
  }

  public void setSlowSpeed() {
    drivetrain.setMaxOutput(DriveConstants.SLOW_SPEED);
  }

  public void setFastSpeed() {
    drivetrain.setMaxOutput(DriveConstants.FAST_SPEED);
  }

  public void simulationPeriodic() {
    simulation.update();
  }

  private void initEncoders() {
    double distancePerPulse = Math.PI * DriveConstants.WHEEL_DIAMETER / DriveConstants.ENCODER_RESOLUTION;
    
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
