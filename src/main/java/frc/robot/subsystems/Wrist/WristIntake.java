// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Wrist;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;

public class WristIntake extends SubsystemBase {
  private final CANSparkMax intakeMotor1;
  private final CANSparkMax intakeMotor2;
  /** Creates a new ArmIntake. */




  public WristIntake()
  {
    intakeMotor1 = new CANSparkMax(IntakeConstants.leftIntakeMotorCanId, MotorType.kBrushless);
    intakeMotor2 = new CANSparkMax(IntakeConstants.rightIntakeMotorCanId, MotorType.kBrushless);

    intakeMotor1.restoreFactoryDefaults();
    intakeMotor2.restoreFactoryDefaults();
  }


  public void setMotorSpeeds(double speed)
  {
    intakeMotor1.set(speed);
    intakeMotor2.set(speed);
  }

  public void setMotorVoltage(double voltage)
  {
    intakeMotor1.setVoltage(voltage);
    intakeMotor2.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
