// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants.NeoMotorConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;
//testing wrist with smart motion support (see: https://github.com/REVrobotics/SPARK-MAX-Examples/tree/master/Java/Smart%20Motion%20Example)
public class WristSubsystemSmartMotion extends SubsystemBase {
  /** Creates a new WristSubsystem2. */
  private final CANSparkMax m_WristMotor;
  private final SparkMaxPIDController m_WristController;
  private final SparkMaxAbsoluteEncoder m_WristEncoder;
  private double setpoint = 0.0;
  private boolean mode = true; 
  private double output;
  private double processVar = 0;

  //using values from the example
  private double maxRPM = 2000; 
  private double maxVel = 500;
  private double minVel = 100;
  private double maxAccel = 500;
  public WristSubsystemSmartMotion() {
    m_WristMotor = new CANSparkMax(WristConstants.kWristMotorCanId, MotorType.kBrushless);
    m_WristMotor.restoreFactoryDefaults();

    m_WristEncoder = m_WristMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

    m_WristController = m_WristMotor.getPIDController();
    m_WristController.setP(WristConstants.kWristMotorP);
    m_WristController.setI(WristConstants.kWristMotorI);
    m_WristController.setD(WristConstants.kWristMotorD);
    m_WristController.setIZone(0);
    m_WristController.setFF(.000156); //feedfoward constant??
    m_WristController.setOutputRange(WristConstants.kWristMinOutput, WristConstants.kWristMaxOutput); 
    
    int smartMotionSlot = 0;
    m_WristController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    m_WristController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    m_WristController.setSmartMotionMaxAccel(maxAccel, smartMotionSlot);
    m_WristController.setSmartMotionAllowedClosedLoopError(0, smartMotionSlot);
    m_WristController.setReference(setpoint, CANSparkMax.ControlType.kSmartMotion);

  //smart motion is rotations (rad)      
  //velocity should be RPM
  }

  @Override
  public void periodic() {
    if(mode){
      m_WristController.setReference(setpoint, CANSparkMax.ControlType.kSmartMotion);
      processVar = m_WristEncoder.getPosition();
    } else {
      m_WristController.setReference(setpoint, CANSparkMax.ControlType.kVelocity);
      processVar = m_WristEncoder.getVelocity();
    }
    output = m_WristMotor.getAppliedOutput();
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
  public double getMinValue(){
    return WristConstants.kWristMinOutput;
  }
  public double getMaxVelocity(){
    return maxVel;
  }
  public double getProccessVar(){
    return processVar;
  }
}
