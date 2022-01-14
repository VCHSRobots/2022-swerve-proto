// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class Constants {
    public static final class SwerveModuleOffsetRadians {
        public static final double FRONT_LEFT = Units.degreesToRadians(-39);
        public static final double FRONT_RIGHT = Units.degreesToRadians(29);
        public static final double BACK_LEFT = Units.degreesToRadians(-16);
        public static final double BACK_RIGHT = Units.degreesToRadians(173);
    }

    public static final double xboxDeadband = 0.9;
}