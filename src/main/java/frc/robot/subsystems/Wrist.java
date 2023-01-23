// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.WristConstants;

public class Wrist extends ProfiledPIDSubsystem {
  
  private final CANSparkMax wristMotor;
  /** Creates a new Wrist. */
  public Wrist() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            0.1,
            0,
            0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(0.4, .2)));
    wristMotor = new CANSparkMax(WristConstants.WristMotorCANID, MotorType.kBrushless);
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) 
  {
    // Use the output (and optionally the setpoint) here
    if(atSetpoint() == false)
    wristMotor.set(output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return 0;
  }

  public boolean atSetpoint() 
  {
    return m_controller.atSetpoint();
  }
}
