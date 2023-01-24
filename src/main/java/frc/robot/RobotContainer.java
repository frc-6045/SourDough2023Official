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
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.WristSubsystemSmartMotion;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final WristSubsystem m_WristSubsystem = new WristSubsystem();
  private final WristSubsystemSmartMotion m_WristSubsystemSmartMotion = new WristSubsystemSmartMotion();
  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

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

    m_WristSubsystem.setDefaultCommand(Commands.runOnce(() -> {
      m_WristSubsystem.setGoal(WristConstants.kWristOffset);
      m_WristSubsystem.enable();
    }, m_WristSubsystem));


    m_WristSubsystemSmartMotion.setDefaultCommand(Commands.runOnce(() -> {
      m_WristSubsystemSmartMotion.setMode(true);
      m_WristSubsystemSmartMotion.setSetpoint(0.0);
    }, m_WristSubsystemSmartMotion));
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
//Wrist Subsystem Commands
/* 
  //A
new JoystickButton(m_driverController, Button.kCross.value).onTrue(Commands.runOnce(() -> {
  m_WristSubsystem.setGoal(WristConstants.kWristState1);
  m_WristSubsystem.enable();
}, m_WristSubsystem));
  //B
new JoystickButton(m_driverController, Button.kCircle.value).onTrue(Commands.runOnce(() -> {
  m_WristSubsystem.setGoal(WristConstants.kWristState2);
  m_WristSubsystem.enable();
}, m_WristSubsystem)); 
//X
new JoystickButton(m_driverController, Button.kSquare.value).onTrue(Commands.runOnce(() -> {
  m_WristSubsystem.setGoal(WristConstants.kWristState3);
  m_WristSubsystem.enable();
}, m_WristSubsystem));
//Y
new JoystickButton(m_driverController, Button.kTriangle.value).onTrue(Commands.runOnce(() -> {
  m_WristSubsystem.setGoal(WristConstants.kWristState4);
  m_WristSubsystem.enable();
}, m_WristSubsystem));


boolean enableVariableOutput = false;
//uggggghhhhhh this will not work for some reason i do not know
//new JoystickButton(m_driverController, Button.kOptions.value).onTrue(Commands.runOnce(() -> { enableVariableOutput = !enableVariableOutput; }, m_WristSubsystem));

new JoystickButton(m_driverController, Button.kR2.value).onTrue(Commands.runOnce(() -> {
  if(enableVariableOutput){
    m_WristSubsystem.goUp();
  }
}, m_WristSubsystem));
new JoystickButton(m_driverController, Button.kL2.value).onTrue(Commands.runOnce(() -> {
  if(enableVariableOutput){
    m_WristSubsystem.goDown();
  }
}, m_WristSubsystem));
*/
  //WristSubsystemSmartMotion Commands (to be chosen)
  //A 
new JoystickButton(m_driverController, Button.kCross.value).onTrue(Commands.runOnce(() -> {
  m_WristSubsystemSmartMotion.setMode(true);
  m_WristSubsystemSmartMotion.setSetpoint(1.0);
}, m_WristSubsystemSmartMotion));
  //B
new JoystickButton(m_driverController, Button.kCircle.value).onTrue(Commands.runOnce(() -> {
  m_WristSubsystemSmartMotion.setMode(true);
  m_WristSubsystemSmartMotion.setSetpoint(2.0);
}, m_WristSubsystemSmartMotion));
//X
new JoystickButton(m_driverController, Button.kSquare.value).onTrue(Commands.runOnce(() -> {
  m_WristSubsystemSmartMotion.setMode(true);
  m_WristSubsystemSmartMotion.setSetpoint(-1.0);
}, m_WristSubsystemSmartMotion));
//Y
new JoystickButton(m_driverController, Button.kTriangle.value).onTrue(Commands.runOnce(() -> {
  m_WristSubsystemSmartMotion.setMode(true);
  m_WristSubsystemSmartMotion.setSetpoint(-2.0);
}, m_WristSubsystemSmartMotion));
//Start
new JoystickButton(m_driverController, Button.kOptions.value).onTrue(Commands.runOnce(() -> {
  m_WristSubsystemSmartMotion.setMode(true);
  m_WristSubsystemSmartMotion.setSetpoint(0.0);
}));

//Triggers control velocity up and down
new JoystickButton(m_driverController, Button.kR2.value).whileTrue(new RunCommand(() -> {
  double Axis = m_driverController.getRightTriggerAxis();
  m_WristSubsystemSmartMotion.setMode(false);
  m_WristSubsystemSmartMotion.setSetpoint(Axis);
}, m_WristSubsystemSmartMotion));
new JoystickButton(m_driverController, Button.kL2.value).whileTrue(new RunCommand(() -> {
  double Axis = m_driverController.getLeftTriggerAxis();
  m_WristSubsystemSmartMotion.setMode(false);
  m_WristSubsystemSmartMotion.setSetpoint(-Axis);
}, m_WristSubsystemSmartMotion));
new JoystickButton(m_driverController, Button.kL2.value).onFalse(Commands.runOnce(() -> {
  m_WristSubsystemSmartMotion.setMode(true);
  m_WristSubsystemSmartMotion.setSetpoint(0.0);
}, m_WristSubsystemSmartMotion));
new JoystickButton(m_driverController, Button.kR2.value).onFalse(Commands.runOnce(() -> {
  m_WristSubsystemSmartMotion.setMode(true);
  m_WristSubsystemSmartMotion.setSetpoint(0.0);
}, m_WristSubsystemSmartMotion));



}
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
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
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }
  //may need 
  /* 
  public double changeWristGoal(double amountChange, ){    
    return amountChange += amountChange;
  }
  */
  public DriveSubsystem getDriveSubsystem()
  {
    return m_robotDrive;
  }
}
