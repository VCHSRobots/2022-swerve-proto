// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class RobotMap {

    /* turning motor gearing:
     * motor--->
     * 8:24
     * 18:72
     * turning wheel
     * 24/8 * 72/18 = 12
     */

    // CANIVORE // 
    public static final String kCANivore_name = "canivore";

    // TALON FX //
    public static final int kDrive_FrontLeftDrive_TalonFX   = 4;
    public static final int kDrive_FrontLeftTurn_TalonFX    = 3;
    public static final int kDrive_FrontRightDrive_TalonFX  = 2;
    public static final int kDrive_FrontRightTurn_TalonFX   = 1;
    public static final int kDrive_BackLeftDrive_TalonFX    = 6;
    public static final int kDrive_BackLeftTurn_TalonFX     = 5;
    public static final int kDrive_BackRightDrive_TalonFX   = 8;
    public static final int kDrive_BackRightTurn_TalonFX    = 7;
    
    // public static final int kDrive_BackRightDrive_TalonFX   = 8;
    // public static final int kDrive_BackRightTurn_TalonFX    = 7;
    // public static final int kDrive_BackLeftDrive_TalonFX  = 2;
    // public static final int kDrive_BackLeftTurn_TalonFX   = 1;
    // public static final int kDrive_FrontRightDrive_TalonFX    = 6;
    // public static final int kDrive_FrontRightTurn_TalonFX     = 5;
    // public static final int kDrive_FrontLeftDrive_TalonFX   = 8;
    // public static final int kDrive_FrontLeftTurn_TalonFX    = 7;

    public static final int kIntake_roller_TalonFX          = 9;
    public static final int kIntake_mover_TalonFX           = 10;
    public static final int kIntake_shooterInput_TalonFX    = 15;

    public static final int kClimb_master_TalonFX          = 21;
    public static final int kClimb_follower_TalonFX         = 22;
    
    public static final int kShoot_BottomMotor_TalonFX     = 31;
    public static final int kShoot_TopMotor_TalonFX        = 32;
    public static final int kTurnTableMotor_TalonFX        = 33;
    // END TALON FX //

    // CANCODER //
    public static final int kDrive_FrontLeftEncoder         = 12;
    public static final int kDrive_FrontRightEncoder        = 11;
    public static final int kDrive_BackLeftEncoder          = 13;
    public static final int kDrive_BackRightEncoder         = 14;
    // public static final int kDrive_BackRightEncoder         = 12;
    // public static final int kDrive_BackLeftEncoder        = 11;
    // public static final int kDrive_FrontRightEncoder          = 13;
    // public static final int kDrive_FrontLeftEncoder         = 14;
    // END CANCODER //

    // SOLENOID //
    // public static final int kClimb_LeftSolenoidForward     = 4;
    // public static final int kClimb_LeftSolenoidReverse     = 5;
    // public static final int kClimb_RightSolenoidForward    = 6;
    // public static final int kClimb_RightSolenoidReverse    = 7;
    public static final int kClimb_SolenoidForward         = 0;
    public static final int kClimb_SolenoidReverse         = 1;

    public static final int kIntake_Pnuematic1              = 2;
    public static final int kIntake_Pnuematic2              = 3;
    // END SOLENOID //
    
    // DIO //
    public static final int kClimber_LeftBottomLimit = 0;
    public static final int kClimber_RightBottomLimit = 1;
    public static final int kIntake_LoadDIO = 3;
    public static final int kIntake_MiddleDIO = 2;
    public static final int kShooter_TurretZeroDIO = 4;
    // END DIO //
}
