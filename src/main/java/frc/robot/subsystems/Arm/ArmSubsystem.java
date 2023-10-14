// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.Arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new WristSubsystem. */
  private final CANSparkMax armMotor;
  private final AbsoluteEncoder m_AbsoluteEncoder;
  public ArmSubsystem() 
  {
    armMotor = new CANSparkMax(ArmConstants.kArmCANID, MotorType.kBrushless);
    armMotor.restoreFactoryDefaults();
    m_AbsoluteEncoder = armMotor.getAbsoluteEncoder(Type.kDutyCycle);
    m_AbsoluteEncoder.setInverted(true);
    // armMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    // armMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    // armMotor.setSoftLimit(SoftLimitDirection.kForward, (float)0.25);
    // armMotor.setSoftLimit(SoftLimitDirection.kReverse, (float)0.95);
  }
  

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm", getAbsoluteEncoderPosition());

    
  }




  public void setSpeed(double speed)
  {
    armMotor.set(speed);
  }

  public AbsoluteEncoder getAbsoluteEncoder()
  {
    return m_AbsoluteEncoder;
  }

  public void stop()
  {
    armMotor.stopMotor();
  }

  public double getAbsoluteEncoderPosition()
  {
    return m_AbsoluteEncoder.getPosition();
  }



}