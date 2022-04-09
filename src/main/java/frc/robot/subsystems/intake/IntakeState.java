package frc.robot.subsystems.intake;

import frc.robot.Constants;

public enum IntakeState {
    
    A
        (
            0,
            0,
            0,
            false
        ),
    B
        (
            Constants.IntakeMotorSpeeds.kIntakeOut,
            Constants.IntakeMotorSpeeds.kBTOut,
            Constants.IntakeMotorSpeeds.kLoaderOut, 
            true),
    C
        (
            Constants.IntakeMotorSpeeds.kIntakeOut,
            Constants.IntakeMotorSpeeds.kBTOut,
            0, 
            true
        ),
    D
        (
            0,
            Constants.IntakeMotorSpeeds.kBTOut,
            Constants.IntakeMotorSpeeds.kLoaderLoadingSpeed,
            true
        ),
    E
        (
            0,
            Constants.IntakeMotorSpeeds.kBTOut,
            Constants.IntakeMotorSpeeds.kLoaderLoadingSpeed,
            false
        ),
    F
        (
            0,
            Constants.IntakeMotorSpeeds.kBTOut,
            Constants.IntakeMotorSpeeds.kLoaderOut,
            false
        );

    public final double intakeSpeed;
    public final double ballTransportSpeed;
    public final double loaderSpeed;
    public final boolean pneumaticOut;
    public boolean isReversable = false;

    IntakeState(double intakeSpeed, double ballTransportSpeed, double loaderSpeed, boolean pnuematicOut) {
        this.intakeSpeed = intakeSpeed;
        this.ballTransportSpeed = ballTransportSpeed;
        this.loaderSpeed = loaderSpeed;
        this.pneumaticOut = pnuematicOut;
    }

    public void setReversable() {
        isReversable = true;
    }
}