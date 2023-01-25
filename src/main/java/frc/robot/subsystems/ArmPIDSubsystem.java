// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkMaxAbsoluteEncoder;
// import com.revrobotics.SparkMaxPIDController;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.ArmConstants;
// //testing Arm with smart motion support (see: https://github.com/REVrobotics/SPARK-MAX-Examples/tree/master/Java/Smart%20Motion%20Example)
// public class ArmPIDSubsystem extends SubsystemBase {
  
//   /** Creates a new ArmSubsystem2. */
//   private final CANSparkMax m_ArmMotor;
//   private final SparkMaxPIDController m_ArmController;
//   private final RelativeEncoder m_ArmEncoder;
//   private double setpoint = 0.0;
//   private boolean mode = true; 
//   private double processVar = 0; 

//   //using values from the example
//   private double maxRPM = 5700;
//   private double maxVel = 2000;
//   private double minVel = 0;
//   private double maxAccel = 1500;
//   public ArmPIDSubsystem() {
//     m_ArmMotor = new CANSparkMax(ArmConstants.kArmMotorCanId, MotorType.kBrushless);
//     m_ArmMotor.restoreFactoryDefaults();

//     m_ArmEncoder = m_ArmMotor.getEncoder();

//     m_ArmController = m_ArmMotor.getPIDController();
//     m_ArmController.setP(ArmConstants.kArmMotorP);
//     m_ArmController.setI(ArmConstants.kArmMotorI);
//     m_ArmController.setD(ArmConstants.kArmMotorD);
//     m_ArmController.setIZone(0);
//     m_ArmController.setFF(0); //feedfoward constant??
//     m_ArmController.setOutputRange(ArmConstants.kArmMinOutput, ArmConstants.kArmMaxOutput); //max and min in rpm
    
//     int smartMotionSlot = 0;
//     m_ArmController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
//     m_ArmController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
//     m_ArmController.setSmartMotionMaxAccel(maxAccel, smartMotionSlot);
//     m_ArmController.setSmartMotionAllowedClosedLoopError(0, smartMotionSlot);



//   }

//   @Override
//   public void periodic() {
//     if(mode){
//       m_ArmController.setReference(setpoint, CANSparkMax.ControlType.kSmartMotion);
//       processVar = m_ArmEncoder.getPosition();
//       //may want to add proccess variable and output to dashboard as seen in smart motion example later
//     } else {
//       m_ArmController.setReference(setpoint, CANSparkMax.ControlType.kVelocity);
//       processVar = m_ArmEncoder.getVelocity();
//     }
//   }

//   public double getSetpoint(){
//     return setpoint;
//   } 
//   public void setSetpoint(Double setpoint){
//     this.setpoint = setpoint;
//   }

//   public boolean getMode(){
//     return mode;
//   }
//   public void setMode(boolean mode){
//     this.mode = mode;
//   }
//   public double getMinValue(){
//     return ArmConstants.kArmMinOutput;
//   }
//   public double getMaxVelocity(){
//     return maxVel;
//   }
//   public double getProccessVar(){
//     return processVar;
//   }
// }
