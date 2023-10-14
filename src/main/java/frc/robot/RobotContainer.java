// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PositionConstants;
import frc.robot.commands.ArmAndWrist.ArmCommands.ClosedLoopArm.PIDArmCommand;
import frc.robot.commands.ArmAndWrist.ArmCommands.ClosedLoopArm.StopArmPID;
import frc.robot.commands.ArmAndWrist.ArmCommands.OpenLoopArm.ActuateArm;
import frc.robot.commands.ArmAndWrist.WristCommands.WristConsume;
import frc.robot.commands.ArmAndWrist.WristCommands.WristEject;
import frc.robot.commands.ArmAndWrist.WristCommands.ClosedLoopWrist.PIDWristCommand;
import frc.robot.commands.ArmAndWrist.WristCommands.ClosedLoopWrist.StopWristPID;
import frc.robot.commands.ArmAndWrist.WristCommands.OpenLoopWrist.ActuateWrist;
import frc.robot.subsystems.Autos;
import frc.robot.subsystems.Tests;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Swerve.DriveSubsystem;
import frc.robot.subsystems.Wrist.WristIntake;
import frc.robot.subsystems.Wrist.WristSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.io.File;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final WristIntake m_armIntake = new WristIntake();
  private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
  private final WristSubsystem m_WristSubsystem = new WristSubsystem();
  private Autos m_autos;
  private Tests m_tests;

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_OperatorController = new XboxController(OIConstants.kDriverControllerPort2);
  ShuffleboardTab teleOpTab = Shuffleboard.getTab("TeleOp");

  


  
  






  SendableChooser<String> autoChooser = new SendableChooser<>();


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_autos = new Autos(m_robotDrive, m_WristSubsystem, m_ArmSubsystem, m_armIntake);
    m_tests = new Tests(m_robotDrive, m_WristSubsystem, m_ArmSubsystem, m_armIntake);
 // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        
        new RunCommand(
            () -> m_robotDrive.drive(
                MathUtil.applyDeadband(-m_driverController.getLeftY(), 0.15),
                MathUtil.applyDeadband(-m_driverController.getLeftX(), 0.15),
                MathUtil.applyDeadband(-m_driverController.getRightX(), 0.20),
                true),
            m_robotDrive));
    m_ArmSubsystem.setDefaultCommand(new ActuateArm(m_ArmSubsystem, m_OperatorController::getLeftY));
    m_WristSubsystem.setDefaultCommand(new ActuateWrist(m_WristSubsystem, m_OperatorController::getRightY));


            autoChooser.setDefaultOption("Drive Forwards", "Drive Forwards");

          //temporary stand in to make it show up.

          //Add list of paths to shuffleBoard. have to do this since the file directory automation part doesn't work.
          autoChooser.addOption("1AndCubeBalance", "1CubeAndBalance");
          autoChooser.addOption("1ConeAndBalance", "1ConeAndBalance");
          autoChooser.addOption("1ConeMidBalance", "1ConeMidBalance");
          autoChooser.addOption("1ConeMidFarBalance", "1ConeMidFarBalance");
          autoChooser.addOption("2Cube", "2Cube");
          autoChooser.addOption("2CubeRed", "2CubeRed");
          autoChooser.addOption("2CubeBalance", "2CubeBalance");
          autoChooser.addOption("2CubeBalanceRed", "2CubeBalanceRed");
          autoChooser.addOption("HighConeMidCubeBack", "HighConeMidCubeBack");
          autoChooser.addOption("Nothing", "Nothing");
          autoChooser.addOption("2.5CubeRed", "2.5CubeRed");
          autoChooser.addOption("2.5Cube", "2.5Cube");
          autoChooser.addOption("OverAndBack", "OverAndBack");
          autoChooser.addOption("3Piece", "3PieceBLUE");
          autoChooser.addOption("3PieceRED", "3PieceRED");
          autoChooser.addOption("3PieceCableRED", "3PieceCableRED");
          autoChooser.addOption("3PieceREDNew", "3PieceREDNew");  
          autoChooser.addOption("3PieceBLUENew", "3PieceBLUENew"); 
          autoChooser.addOption("3PieceCableBLUENew", "3PieceCableBLUENew"); 
          autoChooser.addOption("OneMobility", "OneMobility"); 
          

          //SmartDashboard.putData("Autonomous routine", autoChooser);
          //autoTab.add(autoChooser);
          teleOpTab.addDouble("Wrist Position", m_WristSubsystem::getAbsoluteEncoderCounts);
          teleOpTab.addDouble("Arm position", m_ArmSubsystem::getAbsoluteEncoderPosition);
          teleOpTab.addDouble("Gyro", m_robotDrive::getHeadingDegrees);
          teleOpTab.addDouble("Position", m_robotDrive::getAverageDistanceMeters);
          teleOpTab.addDouble("frontLeftMotor", m_robotDrive::getFrontLeftRot);
          teleOpTab.addDouble("frontRightMotor", m_robotDrive::getFrontRightRot);
          teleOpTab.addDouble("backLeftMotor", m_robotDrive::getBackLeftRot);
          teleOpTab.addDouble("backRightMotor", m_robotDrive::getBackRightRot);
          teleOpTab.addDouble("pitch", m_robotDrive::getRoll);
          teleOpTab.addDouble("pose estimator pose", m_robotDrive::getPoseHeading);
          teleOpTab.addDouble("estimated X", m_robotDrive::getEstimatedX);
          teleOpTab.addDouble("estimated Y", m_robotDrive::getEstimatedY);
          SmartDashboard.putData(m_ArmSubsystem);
          SmartDashboard.putData(m_WristSubsystem);

          
          
 

         
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */

  public Command getTestingCommand(){
    return m_tests.getPitTesting();
  }
   
  private void configureButtonBindings() {
    //set X mode
    new JoystickButton(m_driverController,  XboxController.Button.kRightBumper.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));


    //reset heading
    new JoystickButton(m_driverController,  XboxController.Button.kStart.value)
    .onTrue(new InstantCommand(
      ()-> m_robotDrive.resetOdometry(new Pose2d(m_robotDrive.getPose().getX(), m_robotDrive.getPose().getY(), new Rotation2d())),
       m_robotDrive).alongWith(new InstantCommand( ()-> m_robotDrive.zeroHeading())));



    //Triggers Begin
    //ConeConsume
    new Trigger(() ->
    {
      if(m_driverController.getLeftTriggerAxis() > 0 || m_driverController.getLeftTriggerAxis() < 0)
        return true;
      else
      {
        return false;
      }
    } ).whileTrue(new WristConsume(m_armIntake, m_driverController::getLeftTriggerAxis));


    //cubeConsume
    new Trigger(() ->
    {
      if(m_driverController.getRightTriggerAxis() > 0 || m_driverController.getRightTriggerAxis() < 0)
        return true;
      else
      {
        return false;
      }
    } 
    ).whileTrue(new WristEject(m_armIntake, m_driverController::getRightTriggerAxis));

    //HomePosition
    new Trigger(()->
    {
      if(m_OperatorController.getLeftBumper())
        return true;
      else
        return false;

    }
    ).and(()->
    {
      if(m_OperatorController.getPOV() == 180)
      return true;
    else
      return false;
    }
    ).onTrue(new PIDWristCommand(m_WristSubsystem, PositionConstants.HomeWristPosition))
    .onTrue(new PIDArmCommand(m_ArmSubsystem, PositionConstants.HomeArmPosition));



    //ConeIntake
    new Trigger(()->
    {
      if(m_OperatorController.getLeftBumper())
        return true;
      else
        return false;

    }
    ).and(()->
    {
      if(m_OperatorController.getBButton())
      return true;
    else
      return false;
    }
    ).onTrue(new PIDWristCommand(m_WristSubsystem, PositionConstants.ConeIntakeWristPosition))
    .onTrue(new PIDArmCommand(m_ArmSubsystem, PositionConstants.ConeIntakeArmPosition));




    //CubeIntake

    new Trigger(()->
    {
      if(m_OperatorController.getLeftBumper())
        return true;
      else
        return false;

    }
    ).and(()->
    {
      if(m_OperatorController.getAButton())
      return true;
    else
      return false;
    }
    ).onTrue(new PIDWristCommand(m_WristSubsystem, PositionConstants.CubeIntakeWristPosition))
    .onTrue(new PIDArmCommand(m_ArmSubsystem, PositionConstants.CubeIntakeArmPosition));



    //StationCone
    new Trigger(()->
    {
      if(m_OperatorController.getLeftBumper())
        return true;
      else
        return false;

    }
    ).and(()->
    {
      if(m_OperatorController.getYButton())
      return true;
    else
      return false;
    }
    ).onTrue(new PIDWristCommand(m_WristSubsystem, PositionConstants.StationConeWristPosition))
    .onTrue(new PIDArmCommand(m_ArmSubsystem, PositionConstants.StationConeArmPosition));




    //StationCube
    new Trigger(()->
    {
      if(m_OperatorController.getLeftBumper())
        return true;
      else
        return false;

    }
    ).and(()->
    {
      if(m_OperatorController.getXButton())
      return true;
    else
      return false;
    }
    ).onTrue(new PIDWristCommand(m_WristSubsystem, PositionConstants.StationCubeWristPosition))
    .onTrue(new PIDArmCommand(m_ArmSubsystem, PositionConstants.StationCubeArmPosition));


    //ScoreHigh
    new Trigger(()->
    {
      if(m_OperatorController.getLeftBumper())
        return true;
      else
        return false;

    }
    ).and(()->
    {
      if(m_OperatorController.getPOV() == 0)
      return true;
    else
      return false;
    }
    ).onTrue(new PIDWristCommand(m_WristSubsystem, PositionConstants.ScoreHighWristPosition))
    .onTrue(new PIDArmCommand(m_ArmSubsystem, PositionConstants.ScoreHighArmPosition));








    //ScoreMid
    new Trigger(()->
    {
      if(m_OperatorController.getLeftBumper())
        return true;
      else
        return false;

    }
    ).and(()->
    {
      if(m_OperatorController.getPOV() == 270)
      return true;
    else
      return false;
    }
    ).onTrue(new PIDWristCommand(m_WristSubsystem, PositionConstants.ScoreMidWristPosition))
    .onTrue(new PIDArmCommand(m_ArmSubsystem, PositionConstants.ScoreMidArmPosition));



    //Hold
    new Trigger(()->
    {
      if(m_OperatorController.getLeftBumper())
        return true;
      else
        return false;

    }
    ).and(()->
    {
      if(m_OperatorController.getPOV() == 90)
      return true;
    else
      return false;
    }
    ).onTrue(new PIDWristCommand(m_WristSubsystem, PositionConstants.HoldWristPosition))
    .onTrue(new PIDArmCommand(m_ArmSubsystem, PositionConstants.HoldArmPostion));


    //Cancel active PID Commands
    new Trigger(()->
    {
      if(m_OperatorController.getLeftBumper())
        return true;
      else
        return false;

    }
    ).and(()->
    {
      if(m_OperatorController.getStartButton())
      return true;
    else
      return false;
    }
    ).onTrue(new StopWristPID(m_WristSubsystem))
    .onTrue(new StopArmPID(m_ArmSubsystem));


    //coneIntake
    new Trigger(() ->
    {
      if(m_OperatorController.getLeftTriggerAxis() > 0 || m_OperatorController.getLeftTriggerAxis() < 0)
        return true;
      else
      {
        return false;
      }
    } ).whileTrue(new WristConsume(m_armIntake, m_OperatorController::getLeftTriggerAxis));

    //cubeIntake
    new Trigger(() ->
    {
      if(m_OperatorController.getRightTriggerAxis() > 0 || m_OperatorController.getRightTriggerAxis() < 0)
        return true;
      else
      {
        return false;
      }
    } 
    ).whileTrue(new WristEject(m_armIntake, m_OperatorController::getRightTriggerAxis));


    //HomePosition
    new Trigger(()->
    {
      if(m_OperatorController.getLeftBumper())
        return true;
      else
        return false;

    }
    ).and(()->
    {
      if(m_OperatorController.getPOV() == 180)
      return true;
    else
      return false;
    }
    ).onTrue(new PIDWristCommand(m_WristSubsystem, PositionConstants.HomeWristPosition))
    .onTrue(new PIDArmCommand(m_ArmSubsystem, PositionConstants.HomeArmPosition));

    //ConeIntake

    new Trigger(()->(m_OperatorController.getLeftBumper()))
           .and(()->m_OperatorController.getBButton())
              .onTrue(new PIDWristCommand(m_WristSubsystem, PositionConstants.ConeIntakeWristPosition))
              .onTrue(new PIDArmCommand(m_ArmSubsystem, PositionConstants.ConeIntakeArmPosition));

    //CubeIntake

    new Trigger(()->
    {
      if(m_OperatorController.getLeftBumper())
        return true;
      else
        return false;

    }
    ).and(()->
    {
      if(m_OperatorController.getAButton())
      return true;
    else
      return false;
    }
    ).onTrue(new PIDWristCommand(m_WristSubsystem, PositionConstants.CubeIntakeWristPosition))
    .onTrue(new PIDArmCommand(m_ArmSubsystem, PositionConstants.CubeIntakeArmPosition));

    //StationCone
    new Trigger(()->
    {
      if(m_OperatorController.getLeftBumper())
        return true;
      else
        return false;

    }
    ).and(()->
    {
      if(m_OperatorController.getYButton())
      return true;
    else
      return false;
    }
    ).onTrue(new PIDWristCommand(m_WristSubsystem, PositionConstants.StationConeWristPosition))
    .onTrue(new PIDArmCommand(m_ArmSubsystem, PositionConstants.StationConeArmPosition));

    //StationCube

    new Trigger(()->
    {
      if(m_OperatorController.getLeftBumper())
        return true;
      else
        return false;

    }
    ).and(()->
    {
      if(m_OperatorController.getXButton())
      return true;
    else
      return false;
    }
    ).onTrue(new PIDWristCommand(m_WristSubsystem, PositionConstants.StationCubeWristPosition))
    .onTrue(new PIDArmCommand(m_ArmSubsystem, PositionConstants.StationCubeArmPosition));


    //ScoreHigh
    new Trigger(()->
    {
      if(m_OperatorController.getLeftBumper())
        return true;
      else
        return false;

    }
    ).and(()->
    {
      if(m_OperatorController.getPOV() == 0)
      return true;
    else
      return false;
    }
    ).onTrue(new PIDWristCommand(m_WristSubsystem, PositionConstants.ScoreHighWristPosition))
    .onTrue(new PIDArmCommand(m_ArmSubsystem, PositionConstants.ScoreHighArmPosition));


    //ScoreMid
    new Trigger(()-> m_OperatorController.getLeftBumper())
      .and(()->
    {
      if(m_OperatorController.getPOV() == 270)
      return true;
    else
      return false;
    }
    ).onTrue(new PIDWristCommand(m_WristSubsystem, PositionConstants.ScoreMidWristPosition))
    .onTrue(new PIDArmCommand(m_ArmSubsystem, PositionConstants.ScoreMidArmPosition));


    //Hold
    new Trigger(()-> m_OperatorController.getLeftBumper())
          .and(()-> m_OperatorController.getPOV() == 90)
                .onTrue(new PIDWristCommand(m_WristSubsystem, PositionConstants.HoldWristPosition))
                .onTrue(new PIDArmCommand(m_ArmSubsystem, PositionConstants.HoldArmPostion));

    //Cancel active PID Commands
    new Trigger(()-> m_OperatorController.getLeftBumper())
           .and(()-> m_OperatorController.getStartButton())
                  .onTrue(new StopWristPID(m_WristSubsystem))
                  .onTrue(new StopArmPID(m_ArmSubsystem));

        //toggle limelight
    new Trigger(()-> m_driverController.getPOV() == 90)
          .onTrue(new InstantCommand(()-> m_robotDrive.toggleLimelight(), m_robotDrive));

    //lock heading
    new Trigger(()-> m_driverController.getAButton())
          .whileTrue(new RunCommand(
                ()-> m_robotDrive.DriveWithVisionLockOn(
                      MathUtil.applyDeadband(-m_driverController.getLeftY(), 0.15),
                      MathUtil.applyDeadband(-m_driverController.getLeftX(), 0.15), 
                      m_ArmSubsystem.getAbsoluteEncoderPosition(), 
                      true), 
                m_robotDrive));
  }

  public double getPoseHeading()
  {
    return m_robotDrive.getPose().getRotation().getDegrees();
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() 
  {
    return m_autos.getAutonomousCommand(); 
  }

  public void addVisionMeasurement()
  {
    m_robotDrive.addMyVisionMeasurment();
  }

}
