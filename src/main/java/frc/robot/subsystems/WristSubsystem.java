// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.WristConstants;

public class WristSubsystem extends ProfiledPIDSubsystem {
  /** Creates a new WristSubsystem. */

  private final CANSparkMax m_WristMotor;
  private final SparkMaxAbsoluteEncoder m_WristAbsoluteEncoder;
  private final ArmFeedforward m_WristFeedforward;


  public WristSubsystem() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            WristConstants.kWristMotorP,
            WristConstants.kWristMotorI,
            WristConstants.kWristMotorD,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(WristConstants.kWristMotorMaxVelocity, WristConstants.kWristMotorMaxAcceleration)));
            m_WristMotor = new CANSparkMax(WristConstants.kWristMotorCanId, MotorType.kBrushless);
            m_WristMotor.restoreFactoryDefaults();
            m_WristAbsoluteEncoder = m_WristMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
            m_WristFeedforward = new ArmFeedforward(0, 0, 0);
            m_WristAbsoluteEncoder.setPositionConversionFactor(WristConstants.kWristEncoderPositionFactor);
            m_WristAbsoluteEncoder.setVelocityConversionFactor(WristConstants.kWristEncoderVelocityFactor);
            
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    double feedforward = m_WristFeedforward.calculate(setpoint.position + WristConstants.kWristOffset, setpoint.velocity);
    m_WristMotor.setVoltage(output + feedforward);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return m_WristAbsoluteEncoder.getPosition() + WristConstants.kWristOffset;
  }
  // public void goUp(){
  //   disable();
  //   m_WristMotor.set(0.25);
  //   setGoal(m_WristAbsoluteEncoder.getPosition());
  //   enable();
  // }
  // public void goDown(){
  //   disable();
  //   m_WristMotor.set(-0.25);
  // }
}
