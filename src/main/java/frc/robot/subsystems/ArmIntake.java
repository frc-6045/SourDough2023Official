// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmIntake extends SubsystemBase {

  //Intake motors + the group
  public final CANSparkMax intakeMotor;
  public final CANSparkMax intakeMotor2;
  public final MotorControllerGroup intakeMotorGroup;

  RelativeEncoder intakeEncoder;
  RelativeEncoder intakeEncoder2;

  /** Creates a new ArmIntake. */
  public ArmIntake(CANSparkMax intakeMotor, CANSparkMax intakeMotor2, MotorControllerGroup intakeMotorGroup) 
  {
    this.intakeMotor = intakeMotor;
    this.intakeMotor2 = intakeMotor2;
    this.intakeMotorGroup = intakeMotorGroup;
    this.intakeEncoder = intakeMotor.getEncoder();
    this.intakeEncoder2 = intakeMotor2.getEncoder();
  }



  @Override
  public void periodic() {
   
    // This method will be called once per scheduler run



  }
}
