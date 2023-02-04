// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import frc.robot.Constants.ArmConstants;
import frc.robot.commands.ActuateArmDown;
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
  }
  

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run


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



}