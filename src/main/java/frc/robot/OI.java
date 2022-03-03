// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.xml.crypto.dsig.keyinfo.RetrievalMethod;

/** Add your docs here. **/
public class OI {
    static xbox4415 xboxDrive = new xbox4415(0);
    static xbox4415 xboxClimb = new xbox4415(1);
    // static xbox4415 xboxShooterTesting = new xbox4415(2);
    // static xbox4415 xboxIntakeTesting = new xbox4415(3);
    // static xbox4415 xbox_drive_test = new xbox4415(4);

    static public boolean getDriveForward() {
        // return xbox_drive_test.getYButton();
        return false;
    }

    static public boolean getDriveReverse() {
        // return xbox_drive_test.getAButton();
        return false;
    }

    static public boolean getDriveLeft() {
        // return xbox_drive_test.getXButton();
        return false;
    }

    static public boolean getDriveRight() {
        // return xbox_drive_test.getBButton();
        return false;
    }

    static public double getDriveY() {
        return xboxDrive.getLeftY();
    }

    static public boolean getBarf() {
        return xboxDrive.getRightBumper() && xboxDrive.getRightTriggerAxis() > 0.4;
    }

    static public boolean getUnjam() {
        return xboxDrive.getRightBumper() && xboxDrive.getLeftBumper();
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

    static public boolean startIntake() {
        return xboxDrive.getAButton();
    }

    static public boolean stopIntake() {
        return xboxDrive.getBButton();
    }

    static public boolean getLeftTurntable() {
        // return xboxDrive.getBackButton();
        return xboxDrive.getRawButton(8);
    }

    static public boolean getRightTurntable() {
        // return xboxDrive.getStartButton();
        return xboxDrive.getRawButton(7);

    }

    static public boolean getSolenoidReverse() {
        return xboxClimb.getAButtonPressed();
    }

    static public boolean getSolenoidForward() {
        return xboxClimb.getBButtonPressed();
    }

    static public boolean getArmsUp() {
        return xboxClimb.getRightBumper();
    }

    static public boolean getArmsDown() {
        return xboxClimb.getLeftBumper();
    }

    static public boolean getLeftBumperForShootingFar() {
        return xboxDrive.getLeftBumper();
    }

    static public boolean getRightBumperForShootingClose() {
        return xboxDrive.getRightBumper();
    }

    static public boolean getRightTriggerForShootingAuto() {
        return xboxDrive.getRightTriggerAxis() > 0.4;
    }

    static public boolean getXorYforShootingReleased() {
        return xboxDrive.getXButtonReleased() || xboxDrive.getYButtonReleased();
    }

    static public boolean getAimTurret() {
        return xboxDrive.getLeftTriggerAxis() > 0.5;
    }

    static public boolean getVisionBallEngaged() {
        return xboxDrive.getLeftStickButton();
    }

    static public boolean forwardIntake() {
        // return xboxIntakeTesting.getAButton();
        return false;
    }

    static public boolean reverseIntake() {
        // return xboxIntakeTesting.getBButton();
        return false;
    }

    static public boolean fortFiveTurnTable() {
        // return xboxShooterTesting.getAButton();
        return false;
    }

    static public boolean hundredTurnTable() {
        // return xboxShooterTesting.getXButton();
        return false;
    }

    static public boolean negFortFiveTurnTable() {
        // return xboxShooterTesting.getYButton();
        return false;
    }

    static public boolean negHundredTurnTable() {
        // return xboxShooterTesting.getBButton();
        return false;
    }
}

/*
 * FRC button doc
 * X Returns true if X button is pressed, false otherwise.
 * Y Returns true if Y button is pressed, false otherwise.
 * B Returns true if B button is pressed, false otherwise.
 * A Returns true if A button is pressed, false otherwise.
 * 
 * DpadDown Returns true if D-pad Down is pressed, false otherwise.
 * DpadLeft Returns true if D-pad Left is pressed, false otherwise.
 * DpadRight Returns true if D-pad Right is pressed, false otherwise.
 * DpadUp Returns true if D-pad Up is pressed, false otherwise
 * 
 * LeftTrigger Returns range of value 0.0 to 1.0 as right trigger is pressed.
 * RightTrigger Returns range of value 0.0 to 1.0 as right trigger is pressed.
 * LeftBumper Returns true if the left bumper is pressed, false otherwise.
 * RightBumper Returns true if the right bumper is pressed, false otherwise.
 * 
 * Back Returns true if Back button is pressed, false otherwise.
 * Start Returns true if the Start button is pressed, false otherwise.
 * Guide/Mode Returns true if the Guide/Mode button is pressed, false otherwise.
 * 
 * LeftStickX Returns the left-right deflection of the left stick.Negative
 * values represent left deflections and positive right deflections. Range is
 * -1.0 to +1.0.
 * LeftStickY Returns the up-down deflection of the left stick.Negative values
 * represent up deflections and positive values down deflections. Range is -1.0
 * to +1.0.
 * RightStickX Returns the left-right deflection of the right stick.Negative
 * values represent left deflections and positive right deflections. Range is
 * -1.0 to +1.0.
 * RightStickY Returns the up-down deflection of the right stick.Negative values
 * represent up deflections and positive values down deflections. Range is -1.0
 * to +1.0.
 * LeftStickButton Returns true if the left stick button is pressed, false
 * otherwise.
 * RightStickButton Returns true if the right stick button is pressed, false
 * otherwise.
 * 
 * AtRest Returns true if joys sticks and triggers in neutral position, false
 * otherwise.(AtRest is a property you can access in your programming - not
 * actually a "button" on the controller)
 * 
 * link:
 * https://ftcforum.firstinspires.org/forum/first-tech-challenge-community-forum
 * -this-is-an-open-forum/teams-helping-teams-programming/77647-ftc-game-
 * controller-programming-reference
 */