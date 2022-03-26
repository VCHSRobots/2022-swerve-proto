// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class Rotation2d_4415 {
    public Rotation2d interpolate(Rotation2d startValue, Rotation2d endValue, double t) {
        return startValue.plus(endValue.minus(startValue).times(MathUtil.clamp(t, 0, 1)));
    }
}
