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

public class AutoBalance extends CommandBase {

  private final PIDController m_YController;
  private final DriveSubsystem m_robotDrive;

  /** Creates a new SwerveWithPIDY. */

  public AutoBalance(DriveSubsystem robotDrive) 
  {


    m_YController = new PIDController(0.0075, 0, 0);

    m_YController.setTolerance(0.1);



    m_robotDrive = robotDrive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_robotDrive);
  

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    System.out.println("getting scheduled");

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
      double y_SetPoint = -1.6;
    double y_Speed =  m_YController.calculate(m_robotDrive.getRoll(), y_SetPoint);
    

    m_robotDrive.drive(y_Speed, 0, 0, true);
    System.out.println("Balance running");


    
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_robotDrive.setX();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_YController.atSetpoint() == true )
           return true;
    return false;
  }
}
