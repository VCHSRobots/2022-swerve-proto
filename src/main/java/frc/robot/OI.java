// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class OI {
    static xbox4415 xboxDrive = new xbox4415(0);

    static public double getDriveY() {
        return xboxDrive.getLeftY();
    }

    static public double getDriveX() {
        return xboxDrive.getLeftX();
    }

    static public double getDriveRot() {
        return xboxDrive.getRightX();
    }

    static public boolean shouldSetFieldRelative() {
        return xboxDrive.getPOV() == 270;
    }

    static public boolean shouldSetRobotRelative() {
        return xboxDrive.getPOV() == 90;
    }
}
