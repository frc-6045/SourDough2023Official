// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Swerve.DriveSubsystem;
import frc.robot.subsystems.Wrist.WristIntake;
import frc.robot.subsystems.Wrist.WristSubsystem;
import frc.robot.Constants.PositionConstants;
import frc.robot.commands.ArmAndWrist.SetArmWithWristPosition;
import frc.robot.commands.ArmAndWrist.WristCommands.WristConsume;;
/** Subsystem that wraps the PitTesting command. 
 *  Automates pit testing during compitition.
 *
 */
public class Tests {
    private final DriveSubsystem m_drivetrainSubsystem;
    private final WristSubsystem m_WristSubsystem;
    private final ArmSubsystem m_ArmSubsystem;
    private final WristIntake m_WristIntake;
    private SequentialCommandGroup PitTesting;
    
    public Tests(DriveSubsystem drivetrainSubsystem, WristSubsystem wristSubsystem, ArmSubsystem armSubsystem, WristIntake wristIntake) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_WristSubsystem = wristSubsystem;
        m_ArmSubsystem = armSubsystem;
        m_WristIntake = wristIntake;
        PitTesting = new SequentialCommandGroup(
            new RunCommand(() -> m_drivetrainSubsystem.drive(.6,0,0,false), m_drivetrainSubsystem).withTimeout(5.0), 
            new RunCommand(() -> m_drivetrainSubsystem.drive(0, .6,0,false), m_drivetrainSubsystem).withTimeout(5.0),
            new SetArmWithWristPosition(m_WristSubsystem, PositionConstants.HomeWristPosition, m_ArmSubsystem, PositionConstants.HomeArmPosition)
            .andThen(new WristConsume(m_WristIntake, () -> .7)).withTimeout(3.0));         
    }
    public Command getPitTesting(){
        return PitTesting;
    }





    
}
