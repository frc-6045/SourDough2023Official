// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;
//testing wrist with smart motion support (see: https://github.com/REVrobotics/SPARK-MAX-Examples/tree/master/Java)
public class WristSubsystemSmartMotion extends SubsystemBase {
  /** Creates a new WristSubsystem2. */
  private final CANSparkMax m_WristMotor;
  private final SparkMaxPIDController m_WristController;
  private final SparkMaxAbsoluteEncoder m_WristEncoder; //why
  private double setpoint = 0.0;
  private boolean mode = true; 
  

  public WristSubsystemSmartMotion() {
    m_WristMotor = new CANSparkMax(WristConstants.kWristMotorCanId, MotorType.kBrushless);
    m_WristMotor.restoreFactoryDefaults();

    m_WristEncoder = m_WristMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

    m_WristController = m_WristMotor.getPIDController();
    m_WristController.setP(WristConstants.kWristMotorP);
    m_WristController.setI(WristConstants.kWristMotorI);
    m_WristController.setD(WristConstants.kWristMotorD);
    m_WristController.setIZone(0);
    m_WristController.setFF(0); //feedfoward constant??
    m_WristController.setOutputRange(0, 0); //max and min in rpm

    int smartMotionSlot = 0;
    m_WristController.setSmartMotionMaxVelocity(0, smartMotionSlot);
    m_WristController.setSmartMotionMinOutputVelocity(0, smartMotionSlot);
    m_WristController.setSmartMotionMaxAccel(0, smartMotionSlot);
    m_WristController.setSmartMotionAllowedClosedLoopError(0, smartMotionSlot);



  }

  @Override
  public void periodic() {
    if(mode){
      m_WristController.setReference(setpoint, CANSparkMax.ControlType.kSmartMotion);
      //may want to add proccess variable and output to dashboard as seen in smart motion example later
    } else {
      m_WristController.setReference(setpoint, CANSparkMax.ControlType.kVelocity);
    }
  }

  public double getSetpoint(){
    return setpoint;
  } 
  public void setSetpoint(Double setpoint){
    this.setpoint = setpoint;
  }

  public boolean getMode(){
    return mode;
  }
  public void setMode(boolean mode){
    this.mode = mode;
  }
}
