// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. **/
public class OI {
    static xbox4415 xboxDrive = new xbox4415(0);
    static xbox4415 xboxClimb = new xbox4415(1);
    static xbox4415 xboxShooterTesting = new xbox4415(2);
    static xbox4415 xboxIntakeTesting = new xbox4415(3);

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

    static public boolean getCenterOfRotationFrontRight() {
        return xboxDrive.getRightBumper();
    }
    
    static public boolean startIntake() {
        return xboxIntakeTesting.getXButton();
    }

    static public double getRightTriggerAxisForShoot() {
        return xboxShooterTesting.getRightTriggerAxis();
    }
    static public boolean getLeftBumperForTurntable(){
        return xboxShooterTesting.getLeftBumper();
    }

    static public boolean getSolenoidToggle() {
        return xboxClimb.getAButtonPressed();
    }

    static public boolean getArmsUp(){
        return xboxClimb.getRightBumper();
    }

    static public boolean getArmsDown(){
        return xboxClimb.getLeftBumper();
    }
    static public boolean getRightBumperForTurntable(){
        return xboxShooterTesting.getRightBumper();

    }
    static public boolean getXButtonForToggleFeetToDist(){
        return xboxShooterTesting.getXButton();
    }
    static public boolean getYButtonForShootRPM(){
        return xboxShooterTesting.getYButton();
    }
    

}
