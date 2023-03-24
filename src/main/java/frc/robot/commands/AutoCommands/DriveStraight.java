// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve.DriveSubsystem;

public class DriveStraight extends CommandBase {
  /** Creates a new DriveStraight. */
  private DriveSubsystem m_robotDrive;
  private PIDController m_PidController;
  private double targetX;
  public DriveStraight(DriveSubsystem m_robotDrive, Pose2d targetPose) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_robotDrive);
    this.m_robotDrive = m_robotDrive;
    m_PidController = new PIDController(0.45, 0, 0);
    targetX = targetPose.getX();
    m_PidController.setSetpoint(targetX);
    m_PidController.setTolerance(0.1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    double maxSpeed = 0.7;

    double speed = m_PidController.calculate(m_robotDrive.getPose().getX());
    if (speed > maxSpeed)
        speed = maxSpeed;
    else if(speed < -maxSpeed)
        speed = -maxSpeed;

    m_robotDrive.drive(speed, 0, 0, true);
    System.out.println("am at" + m_robotDrive.getPose().getX());
    System.out.println("setpoint" + m_PidController.getSetpoint());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_PidController.atSetpoint();

  }
}
