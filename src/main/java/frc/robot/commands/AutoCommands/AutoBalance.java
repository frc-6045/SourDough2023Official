// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.PIDSwerveConstants;
import frc.robot.subsystems.Swerve.DriveSubsystem;

public class AutoBalance extends CommandBase {

  private final PIDController m_XController;
  private final DriveSubsystem m_robotDrive;
  private double x_SetPoint = -1.6;
  private boolean pastTimer;
  private Timer timer;
  private double time;

  /** Creates a new SwerveWithPIDY. */

  public AutoBalance(DriveSubsystem robotDrive) 
  {


    m_XController = new PIDController(0.005, 0, 0); // was 0.005

    m_XController.setTolerance(0.05);



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
      
    double x_Speed =  m_XController.calculate(m_robotDrive.getRoll(), x_SetPoint);
    

    m_robotDrive.drive(x_Speed, 0, 0, true);
    System.out.println("Balance running");

    if(time == 0 && m_XController.atSetpoint())
    {
      timer.start();
    }
    else if(m_XController.atSetpoint() == false)
    {
      timer.stop();
      timer.reset();
    }

    



    
    

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
  return (m_XController.atSetpoint() && (timer.get() > 0.25));
  }
}
 