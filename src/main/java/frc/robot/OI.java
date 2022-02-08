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
        return xboxDrive.getPOV() == 180;
    }

    static public boolean getCenterOfRotationFrontLeft() {
        return xboxDrive.getLeftBumper();
    }

    static public boolean getCenterOfRotationFrontRight() {
        return xboxDrive.getRightBumper();
    }

    static public boolean startIntake() {
        return xboxDrive.getAButton();
    }

    static public boolean stopIntake() {
        return xboxDrive.getBButton();
    }

    static public boolean getLeftBumperForTurntable(){
        return xboxDrive.getBackButton();
    }

    static public boolean getSolenoidReverse() {
        return xboxClimb.getAButtonPressed();
    }
    static public boolean getSolenoidForward() {
        return xboxClimb.getBButtonPressed();
    }

    static public boolean getArmsUp(){
        return xboxClimb.getRightBumper();
    }

    static public boolean getArmsDown(){
        return xboxClimb.getLeftBumper();
    }
    static public boolean getRightBumperForTurntable(){
        return xboxDrive.getStartButton();

    }
    static public boolean getXButtonForShootDist(){
        return xboxDrive.getXButton();
    }
    static public boolean getYButtonForShootRPM(){
        return xboxDrive.getYButton();
    }

    static public boolean getXorYforShootingReleased() {
        return xboxDrive.getXButtonReleased() || xboxDrive.getYButtonReleased();
    }
    
    static public boolean getZeroOfTurnTableTalon(){
        return xboxDrive.getRightTriggerAxis() > 0.5;
    }
    

}
