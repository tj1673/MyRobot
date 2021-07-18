// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.operatorinterface.DualShock4Controller;
import frc.robot.subsystems.drive.DriveSubsystem;

public class RobotContainer {
  private final DualShock4Controller pilotInput = new DualShock4Controller(OIConstants.PILOT_JOYSTICK_PORT);
  
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();

  private final Command rotateClockwiseCommand = new RunCommand(() -> driveSubsystem.arcadeDrive(0, 0.5), driveSubsystem);
  private final Command rotateCounterClockwiseCommand = new RunCommand(() -> driveSubsystem.arcadeDrive(0, -0.5), driveSubsystem);

  private final Command manualDriveCommand = new RunCommand(() -> driveSubsystem.arcadeDrive(
    pilotInput.getY(Hand.kLeft),
    pilotInput.getX(Hand.kRight)),
    driveSubsystem
  );

  SendableChooser<Command> autonomousChooser = new SendableChooser<>();

  public RobotContainer() {
    driveSubsystem.setDefaultCommand(manualDriveCommand);
    configureButtonBindings();
    initAutonomousChooser();
  }

  public Command getAutonomousCommand() {
    return autonomousChooser.getSelected();
  }

  private void configureButtonBindings() {
    pilotInput.getLeftBumper()
      .whenPressed(() -> driveSubsystem.setFastSpeed())
      .whenReleased(() -> driveSubsystem.setSlowSpeed());
  }

  private void initAutonomousChooser() {
    autonomousChooser.setDefaultOption("Rotate Clockwise", rotateClockwiseCommand);
    autonomousChooser.addOption("Rotate Counter Clockwise", rotateCounterClockwiseCommand);

    Shuffleboard.getTab("Autonomous").add(autonomousChooser);
  }
}
