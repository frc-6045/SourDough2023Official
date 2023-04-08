// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ArmAndWrist.WristCommands.WristConsume;
import frc.robot.subsystems.Wrist.WristIntake;
import frc.robot.subsystems.Wrist.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class WristConsumeWithTime extends ParallelRaceGroup {
  /** Creates a new WristIntakeWithTime. */
  public WristConsumeWithTime(WristIntake m_WristSubsystem, double time, double speed) {
      
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
      new WristConsume(m_WristSubsystem, () -> speed),
      new WaitCommand(time)
    );
  }
}
