// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmIntake;

public class ArmEject extends CommandBase {
  public final ArmIntake armIntake;
  public Supplier speedSupplier;

  /** Creates a new ArmEjectSlow. */
  public ArmEject(ArmIntake armIntake, Supplier speedSupplier) {
    this.armIntake = armIntake;
    addRequirements(armIntake);
    this.speedSupplier = speedSupplier;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double speed = (double)speedSupplier.get();
    armIntake.setMotorSpeeds(-speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    armIntake.setMotorSpeeds(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
