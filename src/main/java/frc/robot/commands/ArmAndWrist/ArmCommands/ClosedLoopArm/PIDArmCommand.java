// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmAndWrist.ArmCommands.ClosedLoopArm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.ArmSubsystem;

public class PIDArmCommand extends CommandBase {
  /** Creates a new PIDWristCommand. */
  private PIDController m_ArmPIDController;
  private final ArmSubsystem m_ArmSubsystem;
  private double setPoint;
  public PIDArmCommand(ArmSubsystem m_ArmSubsystem, double setPoint) {
    this.m_ArmSubsystem = m_ArmSubsystem;
    m_ArmPIDController = new PIDController(5, 0, 0); //0.15
    m_ArmPIDController.enableContinuousInput(0, 1);
    m_ArmPIDController.setTolerance(0.0038); //038
    this.setPoint = setPoint;
    addRequirements(m_ArmSubsystem);
    

    

    // Use addRequirements() here to declare subsystem dependencies.
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    double feedforward = 0.08; //0.1
    double speed = -m_ArmPIDController.calculate(m_ArmSubsystem.getAbsoluteEncoderPosition(), setPoint);
    speed = (speed > 0) ? speed + feedforward : speed - feedforward; 
    m_ArmSubsystem.setSpeed(speed);
    SmartDashboard.putNumber("PIDArm output: ", speed);
    SmartDashboard.putNumber("set point: ", m_ArmPIDController.getSetpoint());
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    m_ArmSubsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_ArmPIDController.atSetpoint())
          return true;
    return false;

  }

  

  public void setPoint(double setPoint)
  {
    this.setPoint = setPoint;
  }
}
