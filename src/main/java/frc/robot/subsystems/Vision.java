/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase {
  private double tv, tx, ta, ty, tid, fieldX, fieldY, fieldZ, fieldRoll, fieldPitch, fieldYaw;
  private ArrayList<Double> m_targetList;
  private final int MAX_ENTRIES = 50;
  private int idIndex = 0;

  /**
   * Creates a new Vision.
   */
  public Vision() {
  }

  @Override
  public void periodic() {
    LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("");
    LimelightHelpers.LimelightTarget_Fiducial[] llArr = llresults.targetingResults.targets_Fiducials;


    fieldX = llArr[0].getRobotPose_FieldSpace().getX();
    fieldY = llArr[0].getRobotPose_FieldSpace().getY();
    fieldZ = llArr[0].getRobotPose_FieldSpace().getZ();
    fieldRoll = llArr[0].getRobotPose_FieldSpace().getX();
    fieldPitch = llArr[0].getRobotPose_FieldSpace().getY();
    fieldYaw = llArr[0].getRobotPose_FieldSpace().getZ();

    if(llArr.length > 0) {
        SmartDashboard.putNumber("Current ID", tid);
        SmartDashboard.putNumber("Distance", getDistance());
        SmartDashboard.putNumber("X", fieldX);
        SmartDashboard.putNumber("Y", fieldY);
        SmartDashboard.putNumber("Z", fieldZ);
        SmartDashboard.putNumber("Roll", fieldRoll);
        SmartDashboard.putNumber("Pitch", fieldPitch);
        SmartDashboard.putNumber("Yaw", fieldYaw);
    }
  }

  public double getFieldX() {
    return fieldX;
  }

  public double getFieldY() {
    return fieldY;
  }

  public double getFieldZ() {
    return fieldZ;
  }

  public double getFieldRoll() {
    return fieldRoll;
  }

  public double getFieldPitch() {
    return fieldPitch;
  }

  public double getFieldYaw() {
    return fieldYaw;
  }

  public double getTX() {
    return tx;
  }

  public double getTY() {
    return ty;
  }

  public double getTA() {
    double sum = 0;

    for (Double num : m_targetList) {
      sum += num.doubleValue();
    }
    return sum/m_targetList.size();
  }

  public double getDistance() {
    return Constants.OperatorConstants.yDisplace * Math.tan((90 - Constants.OperatorConstants.llAngle - getTY()) * Math.PI/180) - 2;
  }

  public double getTID() {
    return tid;
  }

  public double getIDIndex(){
    return idIndex;
  }

  public boolean isTargetValid() {
    return (tv == 1.0); 
  }

  public void add(){
    idIndex++;
    idIndex %= Constants.OperatorConstants.idsLength;
  }

}