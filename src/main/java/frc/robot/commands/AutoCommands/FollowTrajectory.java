// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;



import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Swerve.DriveSubsystem;

import java.io.IOException;
import java.nio.file.Path;

public class FollowTrajectory extends CommandBase {

  private final DriveSubsystem drive;
  private Trajectory trajectory;
  private boolean toReset;

  public FollowTrajectory(DriveSubsystem drive, String trajectoryFilePath, boolean toReset) {
    this.drive = drive;
    this.toReset = toReset;
    addRequirements(drive);

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryFilePath);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException e) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryFilePath,
          e.getStackTrace());
    }
  }

  @Override
  public void initialize() {
    if (toReset) {
      drive.resetOdometry(trajectory.getInitialPose());
    }

    final ProfiledPIDController thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    new SwerveControllerCommand(
        trajectory,
        drive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        drive::setModuleStates,
        drive).andThen(() -> drive.drive(0, 0, 0, false)).schedule(); // Stops the robot

    // Reset odometry to the starting pose of the trajectory.
    // drive.resetOdometry(trajectory.getInitialPose());
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    drive.drive(0, 0, 0, false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
