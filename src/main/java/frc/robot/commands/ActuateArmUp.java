// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ActuateArmUp extends CommandBase {
  private final ArmSubsystem m_ArmSubsystem;
  /** Creates a new ActuateUp. */
  public ActuateArmUp(ArmSubsystem m_ArmSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_ArmSubsystem = m_ArmSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    // if(m_ArmSubsystem.getAbsoluteEncoder().getPosition() > 0.268);
    //       m_ArmSubsystem.stop();
    m_ArmSubsystem.setSpeed(0.75);
    System.out.println("Arm going up!");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ArmSubsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
