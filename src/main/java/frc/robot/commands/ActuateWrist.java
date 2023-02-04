// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;

public class ActuateWrist extends CommandBase {
  /** Creates a new ActuateWristDown. */
  private final WristSubsystem m_WristSubsystem;
  private Supplier speedSupplier;
  /** Creates a new ActuateUp. */
  public ActuateWrist(WristSubsystem m_WristSubsystem, Supplier speedSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_WristSubsystem = m_WristSubsystem;
    this.speedSupplier = speedSupplier;
    addRequirements(m_WristSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    // if(m_WristSubsystem.getAbsoluteEncoder().getPosition() > 0.8)
    // {
    //       m_WristSubsystem.stop();
    // }
    // else
    //{
    

    double speed = -(double)speedSupplier.get();
    if(speed > 0.1 || speed < -0.1)
    {
      if(speed > 0.5)
        speed = 0.5;
      else if(speed < -0.5)
        speed = -0.5;
      m_WristSubsystem.setSpeed(speed);
    }
    else
    {
      m_WristSubsystem.stop();
    }

  
    // }

    // System.out.println("wristEncoder: " + m_WristSubsystem.getAbsoluteEncoder().getPosition());
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_WristSubsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
