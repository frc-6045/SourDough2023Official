// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Wrist;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.WristConstants;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Direction;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSubsystem extends SubsystemBase {
  /** Creates a new WristSubsystem. */
  private final CANSparkMax wristMotor;
 private final AbsoluteEncoder m_AbsoluteEncoder;
 private Relay m_Lights = new Relay(0); 
 private boolean LightsOn = false;


  public WristSubsystem() 
  {
    wristMotor = new CANSparkMax(WristConstants.kWristCANID, MotorType.kBrushless);
    wristMotor.restoreFactoryDefaults();
    m_AbsoluteEncoder = wristMotor.getAbsoluteEncoder(Type.kDutyCycle);
  
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Wrist", getAbsoluteEncoderCounts());

    
    // This method will be called once per scheduler run
  }

  public void setSpeed(double speed)
  {
    wristMotor.set(speed);
  }

  public void stop()
  {
    wristMotor.stopMotor();
  }

  public double getAbsoluteEncoderCounts()
  {
    return m_AbsoluteEncoder.getPosition();
  }

  public void runLights()
  {
    if(LightsOn)
    {
      m_Lights.set(Value.kReverse);
      LightsOn = false;
    }
    else
    {
      m_Lights.set(Value.kForward);
      LightsOn = true;
    }
  }


  

}
