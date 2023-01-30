// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BadLimelightFactFinding extends SubsystemBase {
  /** Creates a new BadLimelightFactFinding. */
  private NetworkTable table;
  private NetworkTableEntry tx; // x pos of target
  private NetworkTableEntry ty; // y pos of target
  private NetworkTableEntry ta; // area of target
  private NetworkTableEntry tv; // 0 or 1. target or no target
  private double limelightMountAngle, limelightLensHeight, goalHeight, targetOffset;
  private double angleToGoalDeg, angleToGoalRad, goalDistance;
  public BadLimelightFactFinding() {

    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    limelightMountAngle = 0.0;
    limelightLensHeight = 12.0;
    goalHeight = 2.0;
  
    
  }
  @Override
  public void periodic() {
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);   
    targetOffset = ty.getDouble(0.0);
    angleToGoalDeg = limelightMountAngle + targetOffset;
    angleToGoalRad = angleToGoalDeg * (Math.PI / 180.0);
    goalDistance = (goalHeight - limelightLensHeight)/Math.tan(angleToGoalRad);

    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("Distance2Goal", goalDistance);
  }
  
  public double getTx(){
    return tx.getDouble(0.0);
  }
  public double getTy(){
    return ty.getDouble(0.0);
  }
  public double getTv(){
    return tv.getDouble(0.0);
  }
  public double getDistance(){
    return goalDistance;
  }
}
