// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmAndWrist.WristCommands.ClosedLoopWrist;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Wrist.WristSubsystem;

public class PIDWristCommand extends CommandBase {
  /** Creates a new PIDWristCommand. */
  private PIDController m_WristPIDController;
  private final WristSubsystem m_WristSubsystem;
  private double setPoint;
  public PIDWristCommand(WristSubsystem m_WristSubsystem, double setPoint) {
    this.m_WristSubsystem = m_WristSubsystem;
    m_WristPIDController = new PIDController(4, 0., 0.025);
    m_WristPIDController.enableContinuousInput(0, 1);
    m_WristPIDController.setTolerance(0.003); //0.0035
    this.setPoint = setPoint;
    addRequirements(m_WristSubsystem);

    

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    double feedforward = 0.16; //0.15
    double speed = m_WristPIDController.calculate(m_WristSubsystem.getAbsoluteEncoderCounts(), setPoint);
    speed = (speed > 0) ? speed + feedforward : speed - feedforward;
    m_WristSubsystem.setSpeed(speed);
    SmartDashboard.putNumber("Wrist output: ", speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_WristSubsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_WristPIDController.atSetpoint())
          return true;
    return false;

  }

  public void setPoint(double setPoint)
  {
    this.setPoint = setPoint;
  }
}
