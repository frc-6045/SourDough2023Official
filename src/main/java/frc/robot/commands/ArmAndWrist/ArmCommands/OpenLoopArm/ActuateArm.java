// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmAndWrist.ArmCommands.OpenLoopArm;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.ArmSubsystem;

public class ActuateArm extends CommandBase {
  private final ArmSubsystem m_ArmSubsystem;
  private Supplier speedSupplier;
  
  /** Creates a new ActuateUp. */
  public ActuateArm(ArmSubsystem m_ArmSubsystem, Supplier speedSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_ArmSubsystem = m_ArmSubsystem;
    addRequirements(m_ArmSubsystem);
    this.speedSupplier = speedSupplier;
    

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    // if(m_ArmSubsystem.getAbsoluteEncoder().getPosition() > 0.268);
    //     m_ArmSubsystem.stop();

    double speed = (double)speedSupplier.get();
    if(speed > 0.17 || speed < -0.17)
    {
      if(speed > 0.6)
        speed = 0.6;
      else if(speed < -0.6)
        speed = -0.6;
      m_ArmSubsystem.setSpeed(speed);
    }
    else
    {
      m_ArmSubsystem.stop();
    }

  
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
