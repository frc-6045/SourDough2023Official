// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.PoseConstants;
import frc.robot.subsystems.Swerve.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SwerveToNearestStation extends InstantCommand {
  private final DriveSubsystem m_robotDrive;
  private double yDistance;
  private double xDistance;
  public SwerveToNearestStation(DriveSubsystem m_robotDrive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_robotDrive = m_robotDrive;
    yDistance = m_robotDrive.getPose().getY();
    xDistance = m_robotDrive.getPose().getX();
    addRequirements(m_robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    if(yDistance > 10 && yDistance < 11)
    {
      new SwerveToPoseWithTrajectory(m_robotDrive, PoseConstants.pickUpStation).schedule();
    }
  }
}
