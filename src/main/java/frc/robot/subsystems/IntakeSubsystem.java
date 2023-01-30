// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;


public class IntakeSubsystem extends SubsystemBase{
 
  public final CANSparkMax leftIntakeMotor;
  public final CANSparkMax rightIntakeMotor;
  public final MotorControllerGroup intakeMotorGroup;

 RelativeEncoder leftIntakeEncoder;
 RelativeEncoder rightIntakeEncoder;
private int position;
private double speed;
public IntakeSubsystem()
{
    leftIntakeMotor = new CANSparkMax(ArmConstants.leftIntakeMotorCANID, MotorType.kBrushless);
    rightIntakeMotor = new CANSparkMax(ArmConstants.rightIntakeMotorCANID, MotorType.kBrushless);
   leftIntakeMotor.restoreFactoryDefaults();
   rightIntakeMotor.restoreFactoryDefaults();
   
   
   
   
  
leftIntakeMotor.setInverted(true);
   rightIntakeMotor.setInverted(true);

intakeMotorGroup = new MotorControllerGroup(leftIntakeMotor, rightIntakeMotor);


 leftIntakeEncoder = leftIntakeMotor.getEncoder();
     rightIntakeEncoder = rightIntakeMotor.getEncoder();


leftIntakeEncoder.setPosition(0);
rightIntakeEncoder.setPosition(0);
leftIntakeMotor.setInverted(true);
} 


    public void setSpeed(double speed)
    {
        leftIntakeMotor.set(speed);
        rightIntakeMotor.set(speed);
    }


    @Override 
    public void periodic() {




    }

}
