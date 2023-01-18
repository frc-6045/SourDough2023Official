// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;

public class WristWithButtons extends CommandBase {
  /** Creates a new WristWithButtons. */
  private final WristSubsystem wristSubsystem;
  private final Supplier<Boolean> buttonUp;
  private final Supplier<Boolean> buttonDown;
  public WristWithButtons(WristSubsystem wristSubsystem, Supplier<Boolean> buttonUp, Supplier<Boolean> buttonDown) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.wristSubsystem = wristSubsystem;
    this.buttonUp = buttonUp;
    this.buttonDown = buttonDown;

    addRequirements(wristSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wristSubsystem.enable();
    wristSubsystem.setGoal(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(buttonUp.get()){
      wristSubsystem.setGoal(2); 
    } else if(buttonDown.get()){
      wristSubsystem.setGoal(1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wristSubsystem.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
