// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmIntake extends SubsystemBase {

  //Intake motors + the group
  public final CANSparkMax leftIntakeMotor;
  public final CANSparkMax rightIntakeMotor;
  public final MotorControllerGroup intakeMotorGroup;

  RelativeEncoder leftIntakeEncoder;
  RelativeEncoder rightIntakeEncoder;

  /** Creates a new ArmIntake. */
  public ArmIntake() 
  {
    leftIntakeMotor = new CANSparkMax(ArmConstants.leftIntakeMotorCANID, MotorType.kBrushless);
    rightIntakeMotor = new CANSparkMax(ArmConstants.rightIntakeMotorCANID, MotorType.kBrushless);
    leftIntakeMotor.restoreFactoryDefaults();
    rightIntakeMotor.restoreFactoryDefaults();
    rightIntakeMotor.setInverted(true);


    intakeMotorGroup = new MotorControllerGroup(leftIntakeMotor, rightIntakeMotor);

    leftIntakeEncoder = leftIntakeMotor.getEncoder();
    rightIntakeEncoder = rightIntakeMotor.getEncoder();

    leftIntakeEncoder.setPosition(0);
    rightIntakeEncoder.setPosition(0);
    rightIntakeEncoder.setInverted(true);

  }

  

  @Override
  public void periodic() {
   
    // This method will be called once per scheduler run



  }
}
