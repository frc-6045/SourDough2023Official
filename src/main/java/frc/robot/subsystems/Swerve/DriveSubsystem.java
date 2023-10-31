// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  //Create Slew rate limiters
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset,
      "m_frontLeft");

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset,
      "m_frontRight");

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset,
      "m_rearLeft");

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset,
      "m_rearLeft");

  

  private final PIDController m_VisionLockController = new PIDController(0.014, 0, 0);
  

  // The gyro sensor
private final AHRS m_gyro = new AHRS(SPI.Port.kMXP, (byte) 200);

      private final SwerveDrivePoseEstimator m_poseEstimator =
      new SwerveDrivePoseEstimator(
          DriveConstants.kDriveKinematics,
          Rotation2d.fromDegrees(getHeadingDegrees()),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
          },
          new Pose2d(0, 0, new Rotation2d(0)), //
          VecBuilder.fill(0.85, 0.85, Units.degreesToRadians(0.5)), // initiial was 0.05 for both on top and 0.5 for bottom, 0.05, 0.05, 0.65
          VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(60))); // 0.5, 0.5, 50 
          
      LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("limelight");
      ShuffleboardTab limeLightTab = Shuffleboard.getTab("limelight");
      Field2d m_field = new Field2d();
      boolean limelightToggledOn = true;
      
      


  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    this.xLimiter = new SlewRateLimiter(1.8);
    this.yLimiter = new SlewRateLimiter(1.8);
    this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond);

    m_VisionLockController.setSetpoint(0);

    zeroHeading();
    AutoBuilder.configureHolonomic(
      this::getPose,
      this::resetOdometry,
      this::getChassisSpeeds,
      this::setRobotRelativeSpeeds, 
      AutoConstants.autoBuilderPathConfig,
      this);
   
   


  }

  @Override
  public void periodic() {
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() 
  {
    return m_poseEstimator.getEstimatedPosition();
  }

  public double getPoseHeading()
  {
    return m_poseEstimator.getEstimatedPosition().getRotation().getDegrees();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(
        Rotation2d.fromDegrees(getHeadingDegrees()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  public void updateOdometry() 
  {
    m_poseEstimator.update(
        Rotation2d.fromDegrees(getHeadingDegrees()),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        });
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) 
  {
    rot *= 0.3;


    xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kMaxSpeedMetersPerSecond;
    ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kMaxSpeedMetersPerSecond;
    rot = turningLimiter.calculate(rot) * DriveConstants.kMaxAngularSpeed;
    double m_HeadingDegrees = getPose().getRotation().getDegrees();

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(m_HeadingDegrees))             : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
    
  }

  public void DriveWithVisionLockOn(double xSpeed, double ySpeed, double armPosition, boolean fieldRelative)
  {

    double rot;
    if(armPosition > 0.15 && LimelightHelpers.getCurrentPipelineIndex("limelight-bottom") == 1)
    {
      LimelightHelpers.setPipelineIndex("limelight-bottom", 0);
    }
    else if(armPosition < 0.15 && LimelightHelpers.getCurrentPipelineIndex("limelight-bottom") == 0 )
    {
      LimelightHelpers.setPipelineIndex("limelight-bottom", 1);
    }
    rot = m_VisionLockController.calculate(LimelightHelpers.getTX("limelight-bottom"));

    xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kMaxSpeedMetersPerSecond;
    ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kMaxSpeedMetersPerSecond;
    rot = turningLimiter.calculate(rot) * DriveConstants.kMaxAngularSpeed;
    double m_HeadingDegrees = DriverStation.getAlliance() == Alliance.Red ? getPose().getRotation().getDegrees() : getPose().getRotation().getDegrees();

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(m_HeadingDegrees))             : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public void setModuleDriveVoltage(double voltage) 
  {
    m_frontLeft.setDriveVoltage(voltage);
    m_frontRight.setDriveVoltage(voltage);
    m_rearLeft.setDriveVoltage(voltage);
    m_rearRight.setDriveVoltage(voltage);
  }

public void setModuleTurnVoltage(double voltage) 
{
  m_frontLeft.setTurnVoltage(voltage);
  m_frontRight.setTurnVoltage(voltage);
  m_rearLeft.setTurnVoltage(voltage);
  m_rearRight.setTurnVoltage(voltage);
}



  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() 
  {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }



  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) 
  {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  public void setRobotRelativeSpeeds(ChassisSpeeds chassisSpeeds)
  {
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
      swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
  m_frontLeft.setDesiredState(swerveModuleStates[0]);
  m_frontRight.setDesiredState(swerveModuleStates[1]);
  m_rearLeft.setDesiredState(swerveModuleStates[2]);
  m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public SwerveModuleState[] getModuleStates()
  {
    SwerveModuleState[] moduleStates = new SwerveModuleState[4];
    MAXSwerveModule[] modules = getMaxSwerveModules();
    for(int i = 0; i < modules.length; i++)
    {
       moduleStates[i] = modules[i].getState();
    }
    return moduleStates;
  }

  public ChassisSpeeds getChassisSpeeds()
  {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() 
  {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() 
  {
    m_gyro.zeroYaw();  
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeadingDegrees() {
    // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
    return (m_gyro.getAngle() *-1);
  }


  public double getRoll()
  {
    return m_gyro.getRoll();
  }

  public double getEstimatedX()
  {
    return m_poseEstimator.getEstimatedPosition().getX();
  }

  public double getEstimatedY()
  {
    return m_poseEstimator.getEstimatedPosition().getY();
  }
  



  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  

  public MAXSwerveModule[] getMaxSwerveModules()
  {
    MAXSwerveModule[] maxArray = {
      m_frontLeft, 
      m_frontRight, 
      m_rearLeft, 
      m_rearRight};
    return maxArray;
  }

  public double getFrontLeftRot()
  {
    return m_frontLeft.getEncoderCounts();
  }

  public double getFrontRightRot()
  {
    return m_frontRight.getEncoderCounts();
  }

  public double getBackLeftRot()
  {
    return m_rearLeft.getEncoderCounts();
  }

  public double getBackRightRot()
  {
    return m_rearRight.getEncoderCounts();
  }

   public void toggleLimelight()
   {
    if(limelightToggledOn)
    {
    LimelightHelpers.setLEDMode_ForceOn("limelight");
    LimelightHelpers.setLEDMode_ForceOn("limelight-bottom");
    limelightToggledOn = false;
    }
    else if(!limelightToggledOn)
    {
      LimelightHelpers.setLEDMode_ForceOff("limelight");
      LimelightHelpers.setLEDMode_ForceOff("limelight-bottom");
      limelightToggledOn = true;
    }

   }








  
}
