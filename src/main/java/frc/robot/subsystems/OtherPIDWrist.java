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
import frc.robot.Constants.Wristconstants;

public class OtherPIDWrist extends ProfiledPIDSubsystem {
  /** Creates a new WristSubsystem. */

  private final CANSparkMax m_WristMotor;
  private final SparkMaxAbsoluteEncoder m_WristAbsoluteEncoder;
  private final ArmFeedforward m_WristFeedforward;


  public OtherPIDWrist() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            Wristconstants.kWristMotorP,
            Wristconstants.kWristMotorI,
            Wristconstants.kWristMotorD,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(Wristconstants.kWristMotorMaxVelocity, Wristconstants.kWristMotorMaxAcceleration)));
            m_WristMotor = new CANSparkMax(Wristconstants.kWristMotorCanId, MotorType.kBrushless);
            m_WristMotor.restoreFactoryDefaults();
            m_WristAbsoluteEncoder = m_WristMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
            m_WristFeedforward = new ArmFeedforward(0, 0, 0);
          
        
            m_WristAbsoluteEncoder.setPositionConversionFactor(Wristconstants.kWristEncoderPositionFactor);
            m_WristAbsoluteEncoder.setVelocityConversionFactor(Wristconstants.kWristEncoderVelocityFactor);
            
            
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    //double feedforward = m_WristFeedforward.calculate(setpoint.position + Wristconstants.kWristOffset, setpoint.velocity);
    
    
    if(getMeasurement() > 0.33 * (Math.PI * 2) || getMeasurement() > 0.7 * Math.PI * 2)
    {
          m_WristMotor.stopMotor();
    }

    else {
      //double feedforward = (output > 0) ? 0.03 : -0.03;
      double feedforward = 0;
      output*=16;
      m_WristMotor.set(output + feedforward);
      System.out.println("feedForwrd: " + feedforward);
      System.out.println("output: " + output);
      System.out.println("Position: " + this.m_WristAbsoluteEncoder.getPosition() );
    }

    
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return m_WristAbsoluteEncoder.getPosition() + Wristconstants.kWristOffset;
  }
}
