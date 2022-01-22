// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.annotation.Retention;

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
        return Math.copySign(Math.pow(xboxDrive.getRightX(), 2), xboxDrive.getRightX());
    }

    static public boolean shouldSetFieldRelative() {
        return xboxDrive.getPOV() == 270;
    }

    static public boolean shouldSetRobotRelative() {
        return xboxDrive.getPOV() == 90;
    }

    static public boolean getResetOdometry() {
        return xboxDrive.getStartButton();
    }

    static public boolean getCenterOfRotationFrontLeft() {
        return xboxDrive.getLeftBumper();
    }
    
    static public boolean getIntake() {
        return xboxDrive.getAButton();
    }

    static public boolean togglePneumatic() {
        return xboxDrive.getXButton();
    }

}
