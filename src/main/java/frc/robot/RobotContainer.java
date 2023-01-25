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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ArmEjectSlow;
import frc.robot.commands.ArmIntakeSlow;
import frc.robot.subsystems.ArmIntake;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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
  private final ArmIntake m_armIntake = new ArmIntake();
  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  SendableChooser<String> autoChooser = new SendableChooser<>();




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
                MathUtil.applyDeadband(-m_driverController.getRightX(), 0.15),
                true),
            m_robotDrive));

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
          autoChooser.addOption("ThePath", "ThePath");
          autoChooser.addOption("TheOG", "TheOG");
          autoChooser.addOption("3 meters", "3 meters");

          SmartDashboard.putData("Autonomous routine", autoChooser);
          
         
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
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    new JoystickButton(m_driverController, Button.kTriangle.value)
    .onTrue(new InstantCommand(
      ()-> m_robotDrive.zeroHeading(),
       m_robotDrive));
    // new JoystickButton(m_driverController, Button.kL1.value).whileTrue(new ArmIntakeSlow(m_armIntake));
    // new JoystickButton(m_driverController, Button.kL2.value).whileTrue(new ArmEjectSlow(m_armIntake));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    String autoName = autoChooser.getSelected();
    PathPlannerTrajectory examplePath;
    examplePath = PathPlanner.loadPath(autoName, new PathConstraints(4, 3));
    
    //If the path you gave is not in the list, drive forward  
    if (examplePath == null) {
      examplePath = PathPlanner.generatePath(
        new PathConstraints(4, 3), 
        new PathPoint(new Translation2d(1.0, 3.0), Rotation2d.fromDegrees(0)), // position, heading
        new PathPoint(new Translation2d(3.0, 3.0), Rotation2d.fromDegrees(0)) // position, heading
    );
    }



    // Prints for running in simulation, you can comment these our if you want 
    System.out.print("========== Starting Auto ==========\n");
    System.out.print("Path: " + autoName + "\n");
    System.out.print("\n\n");

   // examplePath = PathPlanner.loadPath("Example Path", new PathConstraints(4, 3));


    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("marker1", new PrintCommand("Passed marker 1"));
    eventMap.put("intakeDown", new PrintCommand("intakeDown"));

    m_robotDrive.resetOdometry(examplePath.getInitialPose());

    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      m_robotDrive::getPose, 
      m_robotDrive::resetOdometry,
      DriveConstants.kDriveKinematics,
      new PIDConstants(5.0, 0.0 ,0.2), //original p = 5, 1st attempt: p = 5, d = 0.5, 2nd attempt: p= 5, d = 0.5, 3rd attempt: p = 5, d = 3 this caused the wheels to shutter
      new PIDConstants(1.2, 0.0, 0),
      m_robotDrive::setModuleStates,
      eventMap,
      true,
      m_robotDrive);

      Command fullAuto = autoBuilder.fullAuto(examplePath);

      return fullAuto.andThen(m_robotDrive::setX);

// This is just an example event map. It would be better to have a constant, global event map
// in your code that will be used by all path following commands.







/* 

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    



    //changed to true for the field odometrey instead of default false. 
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, true));
    */
  }




  // public DriveSubsystem getDriveSubsystem()
  // {
  //   return m_robotDrive;
  // }
}
