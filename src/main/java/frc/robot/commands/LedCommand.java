// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDS;

public class LedCommand extends CommandBase {
  /** Creates a new LedCommand. */
  private LEDS ledStrip;

  public LedCommand() {
    ledStrip = new LEDS();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ledStrip.startLEDS();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(ledStrip.getSensorDistance() <= 3.5){
      ledStrip.SetLEDsPurple();
    } else {
      ledStrip.SetLEDsRed();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
