// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;

public class ActuateWristUp extends CommandBase {
  private final WristSubsystem m_WristSubsystem;
  /** Creates a new ActuateUp. */
  public ActuateWristUp(WristSubsystem m_WristSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_WristSubsystem = m_WristSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    // if(m_WristSubsystem.getAbsoluteEncoder().getPosition() > 0.268  && m_WristSubsystem.getAbsoluteEncoder().getPosition() < 0.4)
    // {
    //       m_WristSubsystem.stop();
    // }
    // else
    // {
      m_WristSubsystem.setSpeed(0.5);
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
