// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveManuallyCommand extends CommandBase {
  private DriveSubsystem driveSubsystem;
  private DoubleSupplier speed;
  private DoubleSupplier rotation;

  public DriveManuallyCommand(DriveSubsystem _driveSubsystem, DoubleSupplier _speed, DoubleSupplier _rotation) {
    driveSubsystem = _driveSubsystem;
    speed = _speed;
    rotation = _rotation;

    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    driveSubsystem.manualDrive(speed.getAsDouble(), rotation.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
