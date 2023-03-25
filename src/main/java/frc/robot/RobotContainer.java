// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PositionConstants;
import frc.robot.commands.ArmAndWrist.SetArmWithWristPosition;
import frc.robot.commands.ArmAndWrist.ArmCommands.ClosedLoopArm.PIDArmCommand;
import frc.robot.commands.ArmAndWrist.ArmCommands.ClosedLoopArm.StopArmPID;
import frc.robot.commands.ArmAndWrist.ArmCommands.OpenLoopArm.ActuateArm;
import frc.robot.commands.ArmAndWrist.WristCommands.WristConsume;
import frc.robot.commands.ArmAndWrist.WristCommands.WristEject;
import frc.robot.commands.ArmAndWrist.WristCommands.ClosedLoopWrist.PIDWristCommand;
import frc.robot.commands.ArmAndWrist.WristCommands.ClosedLoopWrist.StopWristPID;
import frc.robot.commands.ArmAndWrist.WristCommands.OpenLoopWrist.ActuateWrist;
import frc.robot.commands.AutoCommands.AutoBalance;
import frc.robot.commands.AutoCommands.AutoScore;
import frc.robot.commands.AutoCommands.WristConsumeWithTime;
import frc.robot.commands.AutoCommands.WristEjectWithTime;
import frc.robot.commands.AutoCommands.SwerveToAndAuto.SwerveWithHighCone;
import frc.robot.commands.AutoCommands.SwerveToMethods.FollowTrajectory;
import frc.robot.commands.AutoCommands.SwerveToNearest.SwerveToNearestPole;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Swerve.DriveSubsystem;
import frc.robot.subsystems.Wrist.WristIntake;
import frc.robot.subsystems.Wrist.WristSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;



import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

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
  // private final OtherPIDWrist m_OtherWrist = new OtherPIDWrist();
  
  

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_OperatorController = new XboxController(OIConstants.kDriverControllerPort2);
  ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");
  ShuffleboardTab teleOpTab = Shuffleboard.getTab("TeleOp");

  


  
  






  SendableChooser<String> autoChooser = new SendableChooser<>();


//Command F points of interest:
/*
 * Top_
 * Configure default commands
 * Add list of paths to shuffleBoard
 * Triggers Begin
 * getAutonomousCommand
 */

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    


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
            try {

              
              // Create a file object
              File f = new File("./src/main/deploy/pathplanner");
      
              // Get all the names of the files present
              // in the given directory
              File[] files = f.listFiles();
              System.out.println("Files are:");
              // Display the names of the files
              for (int i = 0; i < files.length; i++) {
                  String file_name = files[i].getName();
                  String file_extention = file_name.substring(file_name.length() - 5, file_name.length());
                  String path_name = file_name.substring(0, file_name.length() - 5);
                  if (file_extention.equals(".path")){
                    autoChooser.addOption(path_name, path_name);
                  }
              }
          }
          catch (Exception e) {
              System.err.println(e.getMessage());
          }
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
          

          //SmartDashboard.putData("Autonomous routine", autoChooser);
          autoTab.add(autoChooser);
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
      if(m_OperatorController.getLeftTriggerAxis() > 0 || m_OperatorController.getLeftTriggerAxis() < 0)
        return true;
      else
      {
        return false;
      }
    } ).whileTrue(new WristConsume(m_armIntake, m_OperatorController::getLeftTriggerAxis));


    //cubeConsume
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


    //cancel Auto Drive I think
    new Trigger(()->
    {
      if(m_OperatorController.getLeftStickButtonPressed())
        return true;
      else
        return false;
    }
    ).onTrue(new InstantCommand(()->m_robotDrive.drive(0, 0, 0, true)));

        //driveForward
        new Trigger(()->
        {
          if(m_driverController.getLeftBumper())
            return true;
          else
            return false;
    
        }
        ).and(()->
        {
          if(m_driverController.getPOV() == 0)
          return true;
        else
          return false;
        }
        ).whileTrue(new RunCommand(()-> m_robotDrive.drive(0.15, 0, 0, false), m_robotDrive));
    
    
        //toggle limelight
        new Trigger(()->
        {
          if(m_driverController.getPOV() == 90)
            return true;
          else
            return false;
    
        }).onTrue(new InstantCommand(()-> m_robotDrive.toggleLimelight(), m_robotDrive));

//lock heading
    new Trigger(()->
    {
      if(m_driverController.getAButton())
        return true;
      else
        return false;

    }).whileTrue(
      new RunCommand(
        ()-> m_robotDrive.DriveWithVisionLockOn(
          MathUtil.applyDeadband(-m_driverController.getLeftY(), 0.15),
          MathUtil.applyDeadband(-m_driverController.getLeftX(), 0.15), 
          m_ArmSubsystem.getAbsoluteEncoderPosition(), 
          true), 
        m_robotDrive));


//autoBalance for debugging
    // new Trigger(()->
    // {
    //   if(m_driverController.getYButton())
    //     return true;
    //   else
    //     return false;
    // }
    // ).onTrue(new AutoBalance(m_robotDrive));

//swerve to nearest pole debugging
    // new Trigger(()->
    // {
    //   if(m_driverController.getLeftBumper())
    //     return true;
    //   else
    //     return false;

    // }
    // ).and(()->
    // {
    //   if(m_driverController.getPOV() == 0)
    //   return true;
    // else
    //   return false;
    // }
    // ).onTrue(new SwerveToNearestPole(m_robotDrive));

    //autoScore debugging
    // new Trigger(()->
    // {
    //   if(m_driverController.getLeftBumper())
    //     return true;
    //   else
    //     return false;
    // }
    // ).and(()->
    // {
    //   if(m_driverController.getPOV() == 90)
    //   return true;
    // else
    //   return false;
    // }
    // ).onTrue(new AutoScore(m_robotDrive, m_WristSubsystem, m_armIntake, m_ArmSubsystem));


//Everyone is gone binds

    //coneIntake
    new Trigger(() ->
    {
      if(m_driverController.getLeftTriggerAxis() > 0 || m_driverController.getLeftTriggerAxis() < 0)
        return true;
      else
      {
        return false;
      }
    } ).whileTrue(new WristConsume(m_armIntake, m_driverController::getLeftTriggerAxis));

    //cubeIntake
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


    //midScore
    new Trigger(()->
    {
      if(m_driverController.getLeftBumper())
        return true;
      else
        return false;

    }
    ).and(()->
    {
      if(m_driverController.getXButton())
      return true;
    else
      return false;
    }
    ).onTrue(new PIDWristCommand(m_WristSubsystem, PositionConstants.ScoreMidWristPosition))
    .onTrue(new PIDArmCommand(m_ArmSubsystem, PositionConstants.ScoreMidArmPosition));



    //cancel commands
    new Trigger(()->
    {
      if(m_driverController.getLeftBumper())
        return true;
      else
        return false;

    }
    ).and(()->
    {
      if(m_driverController.getRightStickButton())
      return true;
    else
      return false;
    }
    ).onTrue(new StopWristPID(m_WristSubsystem))
    .onTrue(new StopArmPID(m_ArmSubsystem));


    //score high wrist
    new Trigger(()->
    {
      if(m_driverController.getLeftBumper())
        return true;
      else
        return false;

    }
    ).and(()->
    {
      if(m_driverController.getYButton())
      return true;
    else
      return false;
    }
    ).onTrue(new PIDWristCommand(m_WristSubsystem, PositionConstants.ScoreHighWristPosition))
    .onTrue(new PIDArmCommand(m_ArmSubsystem, PositionConstants.ScoreHighArmPosition));


        //CubeIntake

        new Trigger(()->
        {
          if(m_driverController.getLeftBumper())
            return true;
          else
            return false;
    
        }
        ).and(()->
        {
          if(m_driverController.getAButton())
          return true;
        else
          return false;
        }
        ).onTrue(new PIDWristCommand(m_WristSubsystem, PositionConstants.CubeIntakeWristPosition))
        .onTrue(new PIDArmCommand(m_ArmSubsystem, PositionConstants.CubeIntakeArmPosition));
    

            //Hold
    new Trigger(()->
    {
      if(m_driverController.getLeftBumper())
        return true;
      else
        return false;

    }
    ).and(()->
    {
      if(m_driverController.getBButton())
      return true;
    else
      return false;
    }
    ).onTrue(new PIDWristCommand(m_WristSubsystem, PositionConstants.HoldWristPosition))
    .onTrue(new PIDArmCommand(m_ArmSubsystem, PositionConstants.HoldArmPostion));

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
  public Command getAutonomousCommand() {


    // String autoName = autoChooser.getSelected();
    
    // PathPlannerTrajectory examplePath;
    // examplePath = PathPlanner.loadPath(autoName, new PathConstraints(3, 2));
    
    // //If the path you gave is not in the list, drive forward  
    // if (examplePath == null) {
    //   examplePath = PathPlanner.generatePath(
    //     new PathConstraints(4.8, 1.5),  // 4, 3
    //     new PathPoint(new Translation2d(1.0, 3.0), Rotation2d.fromDegrees(0)), // position, heading
    //     new PathPoint(new Translation2d(3.0, 3.0), Rotation2d.fromDegrees(0)) // position, heading
    // );
    // }

    // // Prints for running in simulation, you can comment these our if you want 
    // System.out.print("========== Starting Auto ==========\n");
    // System.out.print("Path: " + autoName + "\n");
    // System.out.print("\n\n");

   // examplePath = PathPlanner.loadPath("Example Path", new PathConstraints(4, 3));

AutoConstants.eventMap.put("Home", new SetArmWithWristPosition(m_WristSubsystem, PositionConstants.HomeWristPosition, m_ArmSubsystem, PositionConstants.HomeArmPosition));
AutoConstants.eventMap.put("CubeGroundIntake", new SetArmWithWristPosition(m_WristSubsystem, PositionConstants.CubeIntakeWristPosition, m_ArmSubsystem, PositionConstants.CubeIntakeArmPosition-0.001));
AutoConstants.eventMap.put("ScoreMid", new SetArmWithWristPosition(m_WristSubsystem, PositionConstants.ScoreMidWristPosition, m_ArmSubsystem, PositionConstants.ScoreMidArmPosition));
AutoConstants.eventMap.put("ConeGroundIntake", new SetArmWithWristPosition(m_WristSubsystem, PositionConstants.ConeIntakeWristPosition, m_ArmSubsystem, PositionConstants.ConeIntakeArmPosition));
AutoConstants.eventMap.put("WristConsumeWithTime", new WristConsumeWithTime(m_armIntake, 2));
AutoConstants.eventMap.put("WristEjectWithTime", new WristEjectWithTime(m_armIntake, 2));
AutoConstants.eventMap.put("ScoreHigh", new SetArmWithWristPosition(m_WristSubsystem, PositionConstants.ScoreHighWristPosition, m_ArmSubsystem, PositionConstants.ScoreHighArmPosition));
AutoConstants.eventMap.put("ScoreBeginning", new PIDArmCommand(m_ArmSubsystem, 0.15).alongWith(new PIDWristCommand(m_WristSubsystem, 0.37)));

  //  eventMap.put("home", new PrintCommand("entering home position"));
  //  eventMap.put("CubeGroundIntake", new PIDWristCommand(m_WristSubsystem, PositionConstants.CubeIntakeWristPosition));
  //  eventMap.put("Thing", new PrintCommand("Thing"));
  //  eventMap.put("CubeIntake", new PrintCommand("intaking cube")); 
  //  eventMap.put("ScoreCubeMid", new PrintCommand("entering mid scoring position"));
  //  eventMap.put("CubeDeposit", new PrintCommand("Cube deposit"));
  //  eventMap.put("Wait5Seconds", new WaitCommand(5));

 

   SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
     m_robotDrive::getPose, 
     m_robotDrive::resetOdometry,
     DriveConstants.kDriveKinematics,
     new PIDConstants(5.0, 0.0 ,0.2), //original p = 5, 1st attempt: p = 5, d = 0.5, 2nd attempt: p= 5, d = 0.5, 3rd attempt: p = 5, d = 3 this caused the wheels to shutter
     new PIDConstants(1.5, 0.0, 0), //5.0, 0, 0.2
     m_robotDrive::setModuleStates,
     AutoConstants.eventMap,
     true,
     m_robotDrive);


     //return autoBuilder.fullAuto(examplePath);
     









     if(autoChooser.getSelected().equals("Nothing"))
     return new PrintCommand("no auto");



     List<PathPlannerTrajectory> auto1Paths =
        PathPlanner.loadPathGroup(autoChooser.getSelected(), 
        1.5, 
        AutoConstants.kMaxAccelerationMetersPerSecondSquared);

     

        m_robotDrive.resetOdometry(auto1Paths.get(0).getInitialPose());
        return autoBuilder.fullAuto(auto1Paths).andThen(new AutoBalance(m_robotDrive));
     
        // Command AutoStuff = new SequentialCommandGroup(
        //   new ParallelCommandGroup(
        //         new SetA
        //   )
        // )
        // Command AutoTest = 
        // new FollowPathWithEvents(
        //       new FollowTrajectory(
        //                 m_robotDrive, 
        //                 examplePath, 
        //                 true), 
        //       examplePath.getMarkers(), 
        //       AutoConstants.eventMap);

        // return AutoTest;





      // Command AutoTest2 = new SequentialCommandGroup(
      //         new SetArmWithWristPosition(m_WristSubsystem, PositionConstants.ScoreHighWristPosition, m_ArmSubsystem, PositionConstants.ScoreHighArmPosition),
      //            new FollowTrajectory(m_robotDrive, auto1Paths.get(0), true),
      //            new PrintCommand("HOLY GUACAMOLE"),
      //            new SetArmWithWristPosition(m_WristSubsystem, PositionConstants.ConeIntakeArmPosition, m_ArmSubsystem, PositionConstants.ConeIntakeArmPosition),
      //            new WaitCommand(3),
      //            new FollowTrajectory(m_robotDrive, auto1Paths.get(1), true)
                 

                
      //  );

              

      //  return AutoTest2;




              
              // new FollowPathWithEvents(
              //         new ProxyCommand(autoBuilder.followPathWithEvents(auto1Paths.get(0))), 
              //                          auto1Paths.get(0).getMarkers(), 
              //                          AutoConstants.eventMap),
              // // new ParallelRaceGroup(new WristEject(m_armIntake, () -> 0.5),
              // //                       new WaitCommand(3)),
              // new FollowPathWithEvents(
              //   new ProxyCommand(autoBuilder.followPathWithEvents(auto1Paths.get(1))), 
              //                     auto1Paths.get(1).getMarkers(), 
              //                     AutoConstants.eventMap),
              // new FollowPathWithEvents(
              //   new ProxyCommand(autoBuilder.followPathWithEvents(auto1Paths.get(2))), 
              //                     auto1Paths.get(2).getMarkers(), 
              //                     AutoConstants.eventMap),
              // new PrintCommand("auto1Paths size: "  + auto1Paths.size()),
              // new WristEjectWithTime(m_armIntake, 3)
              // // new ParallelRaceGroup(new WristEject(m_armIntake, () -> 0.5),
              // //                       new WaitCommand(3)),
              // // new FollowPathWithEvents(
              // //   new ProxyCommand(autoBuilder.followPathWithEvents(auto1Paths.get(3))), 
              // //                   auto1Paths.get(3).getMarkers(), 
              // //                   AutoConstants.eventMap)
              // // new FollowPathWithEvents(
              // //   new ProxyCommand(autoBuilder.followPathWithEvents(auto1Paths.get(4))), 
              // //                   auto1Paths.get(4).getMarkers(), 
              // //                   AutoConstants.eventMap)
       // );
   
   
   
   
   
   
   
        // new SequentialCommandGroup(
        //     new FollowPathWithEvents(
        //       new ProxyCommand(autoBuilder.followPathWithEvents(auto1Paths.get(0))), 
        //                        auto1Paths.get(0).getMarkers(), 
        //                        AutoConstants.eventMap),
        //     new ParallelRaceGroup(new WristConsume(m_armIntake, () -> 0.5),
        //                               new WaitCommand(3)),
        //     new FollowPathWithEvents(
        //                               new ProxyCommand(autoBuilder.followPathWithEvents(auto1Paths.get(1))), 
        //                                                 auto1Paths.get(1).getMarkers(), 
        //                                                 AutoConstants.eventMap)
        // );     

    //    return AutoTest;

   
   
   //  return autoBuilder.fullAuto(examplePath);
    
    

  }

  public void addVisionMeasurement()
  {
    m_robotDrive.addMyVisionMeasurment();
  }

}
