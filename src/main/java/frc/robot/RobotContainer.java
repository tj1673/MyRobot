// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.DriveManuallyCommand;
import frc.robot.operatorinterface.DualShock4Controller;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
  private final DualShock4Controller pilotInput = new DualShock4Controller(OIConstants.PILOT_JOYSTICK_PORT);
  
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();

  private final DriveManuallyCommand driveManuallyCommand = new DriveManuallyCommand(
    driveSubsystem,
    () -> pilotInput.getLeftYAxis(),
    () -> pilotInput.getRightXAxis()
  );

  public RobotContainer() {
    driveSubsystem.setDefaultCommand(driveManuallyCommand);
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    pilotInput.getLeftBumper()
      .whenPressed(() -> driveSubsystem.setFastSpeed())
      .whenReleased(() -> driveSubsystem.setSlowSpeed());
  }
}
