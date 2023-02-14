// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PIDSwerveConstants;
import frc.robot.subsystems.Swerve.DriveSubsystem;

public class SwerveToPose extends CommandBase {
  /** Creates a new SwerveToPose. */
  private final DriveSubsystem m_robotDrive;
  private Pose2d desiredPose;
  private final double targetXPose;
  private final double targetYPose;
  private final double targetRotationPose;
  private final ProfiledPIDController m_ThetaController;
  private final ProfiledPIDController m_XController;
  private final ProfiledPIDController m_YController;

  public SwerveToPose(Pose2d desiredPose, DriveSubsystem m_robotDrive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_robotDrive = m_robotDrive;
    this.desiredPose = desiredPose;
    addRequirements(m_robotDrive);

    m_ThetaController = PIDSwerveConstants.thetaController;
    m_XController = PIDSwerveConstants.m_YController;
    m_YController = PIDSwerveConstants.m_XController;

    m_ThetaController.enableContinuousInput(Math.PI, -Math.PI);
    m_ThetaController.setTolerance(0.1);

    m_XController.setTolerance(0.1);
    m_YController.setTolerance(0.1);

    targetXPose = desiredPose.getX();
    targetYPose = desiredPose.getY();
    targetRotationPose = desiredPose.getRotation().getDegrees();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {

    double x_Speed = m_XController.calculate(m_robotDrive.getPose().getX(), targetXPose);

    double y_Speed = m_YController.calculate(m_robotDrive.getPose().getX(), targetYPose);

    double rot_Speed = m_ThetaController.calculate(m_robotDrive.getPose().getRotation().getDegrees(), targetRotationPose);

    m_robotDrive.drive(x_Speed, y_Speed, rot_Speed, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
