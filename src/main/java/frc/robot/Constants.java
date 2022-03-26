// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;

/** Add your docs here. */
public class Constants {

    public static final boolean isPracticeBot = false;

    public static final class SwerveModuleOffsetRadiansPractice {
        // public static final double FRONT_LEFT = Units.degreesToRadians(-8.3);
        // public static final double FRONT_RIGHT = Units.degreesToRadians(209.0);
        // public static final double BACK_LEFT = Units.degreesToRadians(196.8);
        // public static final double BACK_RIGHT = Units.degreesToRadians(165.1);

        public static final double kBACK_RIGHT  = Units.degreesToRadians(2.5) - Math.PI;
        public static final double kBACK_LEFT   = Units.degreesToRadians(167.5) - Math.PI;
        public static final double kFRONT_RIGHT = Units.degreesToRadians(22.9);
        public static final double kFRONT_LEFT = Units.degreesToRadians(145.6) - Math.PI;
    }
    public static final class SwerveModuleOffsetRadiansComp {
        public static final double FRONT_LEFT = Units.degreesToRadians(-40.00);
        public static final double FRONT_RIGHT = Units.degreesToRadians(97.21);
        public static final double BACK_LEFT = Units.degreesToRadians(76.82);
        public static final double BACK_RIGHT = Units.degreesToRadians(15.91);
    }
    public static final double xboxDeadband = 0.09;
}