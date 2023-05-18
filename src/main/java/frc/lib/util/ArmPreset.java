// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ArmPreset {
    public final double ElbowAngleRadians;
    public final double WristAngleRadians;

    public ArmPreset(double ElbowAngleDegrees, double WristAngleRadians) {
      this.ElbowAngleRadians = Units.degreesToRadians(ElbowAngleDegrees);
      this.WristAngleRadians = Units.degreesToRadians(WristAngleRadians);
    }

    public ArmPreset getInverse() {
      return new ArmPreset(
        Units.radiansToDegrees(-ElbowAngleRadians),
        Units.radiansToDegrees(-WristAngleRadians));
    }
}