// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;

/** Add your docs here. */
public class Constants {
    public static final class SwerveModuleOffsetRadians {
        public static final double FRONT_LEFT = Units.degreesToRadians(-42.5);
        public static final double FRONT_RIGHT = Units.degreesToRadians(29.5);
        public static final double BACK_LEFT = Units.degreesToRadians(-16.5);
        public static final double BACK_RIGHT = Units.degreesToRadians(172.0);
    }

    public static final double xboxDeadband = 0.09;
}