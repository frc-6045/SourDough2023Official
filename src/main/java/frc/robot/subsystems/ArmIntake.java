// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.net.CacheRequest;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.ArmIntakeConstants;

public class ArmIntake extends SubsystemBase {
  private final CANSparkMax intakeMotor1;
  private final CANSparkMax intakeMotor2;
  private final RelativeEncoder intakeEncoder;
  /** Creates a new ArmIntake. */
  public ArmIntake()
  {
    intakeMotor1 = new CANSparkMax(ArmIntakeConstants.leftIntakeMotorCanId, MotorType.kBrushless);
    intakeMotor2 = new CANSparkMax(ArmIntakeConstants.rightIntakeMotorCanId, MotorType.kBrushless);

    intakeEncoder = intakeMotor1.getEncoder();

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

  public void periodic() {
    // This method will be called once per scheduler run
  }
}
