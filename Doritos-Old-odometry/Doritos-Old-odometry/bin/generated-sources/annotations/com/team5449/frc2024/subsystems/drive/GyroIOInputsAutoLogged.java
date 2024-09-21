package com.team5449.frc2024.subsystems.drive;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class GyroIOInputsAutoLogged extends GyroIO.GyroIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("Connected", connected);
    table.put("Yaw", yaw);
    table.put("Pitch", pitch);
    table.put("Roll", roll);
    table.put("AngularVelocity", angularVelocity);
    table.put("AccumGyroX", AccumGyroX);
    table.put("AccumGyroY", AccumGyroY);
    table.put("AccumGyroZ", AccumGyroZ);
  }

  @Override
  public void fromLog(LogTable table) {
    connected = table.get("Connected", connected);
    yaw = table.get("Yaw", yaw);
    pitch = table.get("Pitch", pitch);
    roll = table.get("Roll", roll);
    angularVelocity = table.get("AngularVelocity", angularVelocity);
    AccumGyroX = table.get("AccumGyroX", AccumGyroX);
    AccumGyroY = table.get("AccumGyroY", AccumGyroY);
    AccumGyroZ = table.get("AccumGyroZ", AccumGyroZ);
  }

  public GyroIOInputsAutoLogged clone() {
    GyroIOInputsAutoLogged copy = new GyroIOInputsAutoLogged();
    copy.connected = this.connected;
    copy.yaw = this.yaw;
    copy.pitch = this.pitch;
    copy.roll = this.roll;
    copy.angularVelocity = this.angularVelocity;
    copy.AccumGyroX = this.AccumGyroX;
    copy.AccumGyroY = this.AccumGyroY;
    copy.AccumGyroZ = this.AccumGyroZ;
    return copy;
  }
}
