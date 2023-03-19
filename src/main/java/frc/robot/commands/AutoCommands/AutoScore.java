// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PositionConstants;
import frc.robot.commands.SetArmWithWristPosition;
import frc.robot.commands.AutoCommands.SwerveToMethods.SwerveToPoseWithTrajectory;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Swerve.DriveSubsystem;
import frc.robot.subsystems.Wrist.WristIntake;
import frc.robot.subsystems.Wrist.WristSubsystem;
import frc.robot.Constants.PoseConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class AutoScore extends SequentialCommandGroup {
  /** Creates a new AutoScore. */

  private final DriveSubsystem m_robotDrive;
private final WristSubsystem m_WristSubsytem;
private final WristIntake m_WristIntake;
private final ArmSubsystem m_ArmSubsystem;

// private double yDistance;
// private double xDistance;

  public AutoScore(DriveSubsystem m_robotDrive, WristSubsystem m_WristSubsystem, WristIntake m_WristIntake, ArmSubsystem m_ArmSubsystem) {
    this.m_robotDrive = m_robotDrive;
    this.m_ArmSubsystem = m_ArmSubsystem;
    this.m_WristSubsytem = m_WristSubsystem;
    this.m_WristIntake = m_WristIntake;
    // yDistance = m_robotDrive.getPose().getY();
    // xDistance = m_robotDrive.getPose().getX();
    
    addRequirements(m_robotDrive, m_WristSubsystem, m_WristIntake, m_ArmSubsystem);
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands
    (
      new SetArmWithWristPosition(m_WristSubsytem, PositionConstants.ScoreHighWristPosition, m_ArmSubsystem, PositionConstants.ScoreHighArmPosition),
      new SwerveToPoseWithTrajectory(m_robotDrive, PoseConstants.secondConeHighPosition2));

  }
}
