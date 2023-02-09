// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.VisonConstants;
import frc.robot.subsystems.BadLimelightFactFinding;
import frc.robot.subsystems.DriveSubsystem;

public class BadVisonSeeking extends CommandBase {
  private final DriveSubsystem m_DriveSubsystem;
  private final BadLimelightFactFinding m_Limelight;
  private double x, y, v;
  /** Creates a new BadVisonSeeking. */
  public BadVisonSeeking(DriveSubsystem drive, BadLimelightFactFinding vison) {
    m_DriveSubsystem = drive;
    m_Limelight = vison;
    addRequirements(m_DriveSubsystem, m_Limelight);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    x = m_Limelight.getTx();
    y = m_Limelight.getTy();
    v = m_Limelight.getTv();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(v != 1.0 &&  x >= 5.0 || x <= -5.0){
      m_DriveSubsystem.drive(0, 0, 0.5, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveSubsystem.zeroHeading();
  }
 
    
    
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_Limelight.getDistance() >= 12.0){ //note distance is in inches
      return true;
    }
    return false;
  }
}
