package com.team5449.frc2024.subsystems.drive;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class SwerveModuleIOInputsAutoLogged extends SwerveModuleIO.SwerveModuleIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("DrivePositionRad", drivePositionRad);
    table.put("DrivePositionMeters", drivePositionMeters);
    table.put("DriveVelocityMetersPerSec", driveVelocityMetersPerSec);
    table.put("DriveCurrentDrawAmps", driveCurrentDrawAmps);
    table.put("DriveAppliedVolts", driveAppliedVolts);
    table.put("SteerPositionTicks", steerPositionTicks);
    table.put("SteerPositionRad", steerPositionRad);
    table.put("SteerPositionDeg", steerPositionDeg);
    table.put("SteerVelocityRadPerSec", steerVelocityRadPerSec);
    table.put("SteerCurrentDrawAmps", steerCurrentDrawAmps);
    table.put("SteerAppliedVolts", steerAppliedVolts);
    table.put("SteerAbsolutePositionRad", steerAbsolutePositionRad);
    table.put("TargetDriveVelocityMetersPerSec", targetDriveVelocityMetersPerSec);
    table.put("TargetSteerPositionRad", targetSteerPositionRad);
  }

  @Override
  public void fromLog(LogTable table) {
    drivePositionRad = table.get("DrivePositionRad", drivePositionRad);
    drivePositionMeters = table.get("DrivePositionMeters", drivePositionMeters);
    driveVelocityMetersPerSec = table.get("DriveVelocityMetersPerSec", driveVelocityMetersPerSec);
    driveCurrentDrawAmps = table.get("DriveCurrentDrawAmps", driveCurrentDrawAmps);
    driveAppliedVolts = table.get("DriveAppliedVolts", driveAppliedVolts);
    steerPositionTicks = table.get("SteerPositionTicks", steerPositionTicks);
    steerPositionRad = table.get("SteerPositionRad", steerPositionRad);
    steerPositionDeg = table.get("SteerPositionDeg", steerPositionDeg);
    steerVelocityRadPerSec = table.get("SteerVelocityRadPerSec", steerVelocityRadPerSec);
    steerCurrentDrawAmps = table.get("SteerCurrentDrawAmps", steerCurrentDrawAmps);
    steerAppliedVolts = table.get("SteerAppliedVolts", steerAppliedVolts);
    steerAbsolutePositionRad = table.get("SteerAbsolutePositionRad", steerAbsolutePositionRad);
    targetDriveVelocityMetersPerSec = table.get("TargetDriveVelocityMetersPerSec", targetDriveVelocityMetersPerSec);
    targetSteerPositionRad = table.get("TargetSteerPositionRad", targetSteerPositionRad);
  }

  public SwerveModuleIOInputsAutoLogged clone() {
    SwerveModuleIOInputsAutoLogged copy = new SwerveModuleIOInputsAutoLogged();
    copy.drivePositionRad = this.drivePositionRad;
    copy.drivePositionMeters = this.drivePositionMeters;
    copy.driveVelocityMetersPerSec = this.driveVelocityMetersPerSec;
    copy.driveCurrentDrawAmps = this.driveCurrentDrawAmps;
    copy.driveAppliedVolts = this.driveAppliedVolts;
    copy.steerPositionTicks = this.steerPositionTicks;
    copy.steerPositionRad = this.steerPositionRad;
    copy.steerPositionDeg = this.steerPositionDeg;
    copy.steerVelocityRadPerSec = this.steerVelocityRadPerSec;
    copy.steerCurrentDrawAmps = this.steerCurrentDrawAmps;
    copy.steerAppliedVolts = this.steerAppliedVolts;
    copy.steerAbsolutePositionRad = this.steerAbsolutePositionRad;
    copy.targetDriveVelocityMetersPerSec = this.targetDriveVelocityMetersPerSec;
    copy.targetSteerPositionRad = this.targetSteerPositionRad;
    return copy;
  }
}
