// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class Wristconstants {

    public static final double kWristMotorP = 0.1; 
    public static final double kWristMotorI = 0.0;
    public static final double kWristMotorD = 0.0;
    public static final double kWristMotorMaxVelocity = 0.2;
    public static final double kWristMotorMaxAcceleration = 0.5;
    public static final double kWristEncoderPositionFactor = (Math.PI * 2); //radians
    public static final double kWristEncoderVelocityFactor = (Math.PI * 2) / 60.0; //radians per second | note: less sure on this one, stealing it from kTurningEncoderVelocityFactor |
    public static final int kWristMotorCanId = 32;
    public static final double kWristOffset = 0; //in radians
    public static final double kWristMinOutput = -0.2;
    public static final double kWristMaxOutput = .2;
    

  }
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    
    //Slew Constants
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
    kMaxAngularSpeed / 4;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 2;


    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(30);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2; //-Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 20;
    public static final int kRearLeftDrivingCanId = 24;
    public static final int kFrontRightDrivingCanId = 22;
    public static final int kRearRightDrivingCanId = 26;

    public static final int kFrontLeftTurningCanId = 21;
    public static final int kRearLeftTurningCanId = 25;
    public static final int kFrontRightTurningCanId = 23;
    public static final int kRearRightTurningCanId = 27;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

        //TODO: maybe remove this multiplier
    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI ) 
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kDriverControllerPort2 = 1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  
    public static final TrapezoidProfile.Constraints kDriveControllerConstraints = new TrapezoidProfile.Constraints(
    DriveConstants.kMaxSpeedMetersPerSecond, DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);

    public static final double maxAutoSpeed = 4.0;
    public static final double maxAutoAcceleration = 2.0;

    public static final HashMap<String, Command> eventMap = new HashMap<>();


  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class ArmIntakeConstants
  {
    public static final int leftIntakeMotorCanId = 33;
    public static final int rightIntakeMotorCanId = 34;
  }

  public static final class ArmConstants
  {
    public static final int kArmCANID = 31;
    public static final double maxAmperageOnArm = 60;
    public static final int kArmMotorCanId = 0;
    public static final double kArmMotorP = 0;
    public static final double kArmMotorI = 0;
    public static final double kArmMotorD = 0;
    public static final double kArmMinOutput = 0;
    public static final double kArmMaxOutput = 0;
 
  }

  public static final class WristConstants
  {
    public static final int kWristCANID = 32;
    public static final double kWristMotorP = .2;
    public static final double kWristMotorI = 0;
    public static final double kWristMotorD = 0;
    public static final double kWristMinOutput = 0;
    public static final double kWristMaxOutput = 0;
    public static final int kWristMotorCanId = 0;
    public static final double kWristMotorMaxVelocity = 0.5;
    public static final double kWristMotorMaxAcceleration = 0.2;


    public static final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
      kWristMotorMaxVelocity, kWristMotorMaxAcceleration);
  }

  public static final class PositionConstants
  {
    //HomePosition
    public static final double HomeWristPosition = 0.4467;
    public static final double HomeArmPosition = 0;

    //ConeIntake
    public static final double ConeIntakeWristPosition = 0.2732;
    public static final double ConeIntakeArmPosition = 0;

    //CubeIntake
    public static final double CubeIntakeWristPosition = 0.2470;
    public static final double CubeIntakeArmPosition = 0.01856;

    //StationCone
    public static final double StationConeWristPosition = 0.0380;
    public static final double StationConeArmPosition = 0.227;

    //StationCube
    public static final double StationCubeWristPosition = 0.0975;
    public static final double StationCubeArmPosition = 0.2067;

    //ScoreHigh
    public static final double ScoreHighWristPosition = 0.1561;
    public static final double ScoreHighArmPosition = 0.1979;

    //ScoreMid
    public static final double ScoreMidWristPosition = 0.2400;
    public static final double ScoreMidArmPosition = 0.123;

    //Hold
    public static final double HoldWristPosition = 0.40776;
    public static final double HoldArmPostion = 0.0029;

  }

  public static final class PIDSwerveConstants
  {
    public static final ProfiledPIDController thetaController = new ProfiledPIDController(0.04, 0, 0, AutoConstants.kThetaControllerConstraints);
    public static final ProfiledPIDController m_XController = new ProfiledPIDController(0.04, 0, 0, AutoConstants.kDriveControllerConstraints);
    public static final ProfiledPIDController m_YController = new ProfiledPIDController(0.1, 0, 0, AutoConstants.kDriveControllerConstraints);
  

  }

  public static final class LimelightConstants
  {
    public static final double CAMERA_ELEVATION = 30.75; // TEST BOT
    public static final double TARGET_ELEVATION = 258; // TEST BOT
    public static final double LIMELIGHT_TO_ROBOT_CENTER = 9; // TEST BOT
    // UNIT = degrees
    public static final double CAMERA_ANGLE = 0; // TEST BOT


    // LED MODES /limelight/ledMode
    public static final int LED_MODE_FROM_PIPELINE = 0;
    public static final int LED_MODE_FORCE_OFF = 1;
    public static final int LED_MODE_FORCE_BLINK = 2;
    public static final int LED_MODE_FORCE_ON = 3;

    // pipeline numbers
    public static final int PIPELINE_TELEOP = 0;
    public static final int PIPELINE_GET_POS = 1;
  }

  public static final class PoseConstants
  {
    public static final Pose2d targetPose1 = new Pose2d(14, 1, null);
  }

  

  
}
