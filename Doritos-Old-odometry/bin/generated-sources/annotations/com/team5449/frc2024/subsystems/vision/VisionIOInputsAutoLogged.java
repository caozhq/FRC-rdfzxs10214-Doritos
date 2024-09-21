package com.team5449.frc2024.subsystems.vision;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class VisionIOInputsAutoLogged extends VisionIO.VisionIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("X", x);
    table.put("Y", y);
    table.put("Rotation", rotation);
    table.put("Timestamp", timestamp);
    table.put("IsNew", isNew);
    table.put("MaxAmbiguity", maxAmbiguity);
    table.put("MaxDistance", maxDistance);
    table.put("MinDistance", minDistance);
    table.put("HasTarget", hasTarget);
    table.put("SingleIDUsed", singleIDUsed);
    table.put("SingleIDUsedDouble", singleIDUsedDouble);
    table.put("TranslationToTargetX", translationToTargetX);
    table.put("TranslationToTargetY", translationToTargetY);
    table.put("RotationToTargetYaw", rotationToTargetYaw);
  }

  @Override
  public void fromLog(LogTable table) {
    x = table.get("X", x);
    y = table.get("Y", y);
    rotation = table.get("Rotation", rotation);
    timestamp = table.get("Timestamp", timestamp);
    isNew = table.get("IsNew", isNew);
    maxAmbiguity = table.get("MaxAmbiguity", maxAmbiguity);
    maxDistance = table.get("MaxDistance", maxDistance);
    minDistance = table.get("MinDistance", minDistance);
    hasTarget = table.get("HasTarget", hasTarget);
    singleIDUsed = table.get("SingleIDUsed", singleIDUsed);
    singleIDUsedDouble = table.get("SingleIDUsedDouble", singleIDUsedDouble);
    translationToTargetX = table.get("TranslationToTargetX", translationToTargetX);
    translationToTargetY = table.get("TranslationToTargetY", translationToTargetY);
    rotationToTargetYaw = table.get("RotationToTargetYaw", rotationToTargetYaw);
  }

  public VisionIOInputsAutoLogged clone() {
    VisionIOInputsAutoLogged copy = new VisionIOInputsAutoLogged();
    copy.x = this.x;
    copy.y = this.y;
    copy.rotation = this.rotation;
    copy.timestamp = this.timestamp;
    copy.isNew = this.isNew;
    copy.maxAmbiguity = this.maxAmbiguity;
    copy.maxDistance = this.maxDistance;
    copy.minDistance = this.minDistance;
    copy.hasTarget = this.hasTarget;
    copy.singleIDUsed = this.singleIDUsed;
    copy.singleIDUsedDouble = this.singleIDUsedDouble;
    copy.translationToTargetX = this.translationToTargetX;
    copy.translationToTargetY = this.translationToTargetY;
    copy.rotationToTargetYaw = this.rotationToTargetYaw;
    return copy;
  }
}
