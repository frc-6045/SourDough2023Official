// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.SwerveToNearest;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.PoseConstants;
import frc.robot.commands.AutoCommands.SwerveToMethods.SwerveToPoseWithTrajectory;
import frc.robot.subsystems.Swerve.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SwerveToNearestPole extends InstantCommand {
  private final DriveSubsystem m_robotDrive;
  private double yDistance;
  private double xDistance;
  public SwerveToNearestPole(DriveSubsystem m_robotDrive) {
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

    if(yDistance > 1 && yDistance < 2)
    {
      new SwerveToPoseWithTrajectory(m_robotDrive, PoseConstants.firstConeHighPosition1).schedule();
    }
    if(yDistance > 2 && yDistance < 3)
    {
      new SwerveToPoseWithTrajectory(m_robotDrive, PoseConstants.secondConeHighPosition1).schedule();
    }
    if(yDistance > 3 && yDistance < 4)
    {
      new SwerveToPoseWithTrajectory(m_robotDrive, PoseConstants.thirdConeHighPosition1).schedule();
    }
    if(yDistance > 4 && yDistance < 5)
    {
      new SwerveToPoseWithTrajectory(m_robotDrive, PoseConstants.fourthConeHighPosition1).schedule();
    }



    
  }
}
