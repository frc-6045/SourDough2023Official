// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.PIDSwerveConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Swerve.DriveSubsystem;

public class ReflectiveSwervePID extends CommandBase {
  private final ProfiledPIDController m_ThetaController;
  private final ProfiledPIDController m_XController;
  private final ProfiledPIDController m_YController;
  private final DriveSubsystem m_robotDrive;
  private double setPoint;
  private final LimelightSubsystem m_LimelightSubsystem;
  /** Creates a new SwerveWithPIDY. */

  public ReflectiveSwervePID(DriveSubsystem robotDrive, LimelightSubsystem m_LimelightSubsystem) 
  {
    m_ThetaController = PIDSwerveConstants.thetaController;
    m_XController = PIDSwerveConstants.m_YController;
    m_YController = PIDSwerveConstants.m_XController;

    m_ThetaController.enableContinuousInput(Math.PI, -Math.PI);
    m_ThetaController.setTolerance(0.1);

    m_XController.setTolerance(0.1);
    m_YController.setTolerance(0.1);



    m_robotDrive = robotDrive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_robotDrive);
    this.m_LimelightSubsystem = m_LimelightSubsystem;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    double x_SetPoint = 0;
    double x_Speed = m_XController.calculate(m_LimelightSubsystem.getSkew(), x_SetPoint);
    double y_SetPoint = 2;
    double y_Speed = m_YController.calculate(m_LimelightSubsystem.getYaw(), y_SetPoint);
    double thetaSetPoint = 0;
    double rot_Speed = m_ThetaController.calculate(m_LimelightSubsystem.getSkew(), thetaSetPoint);

    m_robotDrive.drive(x_Speed, y_Speed, rot_Speed, true);


    
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_XController.atSetpoint() == true 
        && m_YController.atSetpoint() == true 
        && m_ThetaController.atSetpoint() == true)
            return true;
    return false;
  }
}
