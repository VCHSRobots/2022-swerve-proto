package frc.robot.subsystems.intake;

import frc.robot.Constants;

public enum IntakeState {
    
    A
        (
            "Nothing",
            0,
            0,
            0,
            false
        ),
    B
        (
            "Loading first ball",
            Constants.IntakeMotorSpeeds.kIntakeOut,
            Constants.IntakeMotorSpeeds.kBTOut,
            Constants.IntakeMotorSpeeds.kLoaderOut, 
            true),
    C
        (
            "Loading second ball (pt 1)",
            Constants.IntakeMotorSpeeds.kIntakeOut,
            Constants.IntakeMotorSpeeds.kBTOut,
            0, 
            true
        ),
    CA
        (
            "Moving second ball to middle",
            Constants.IntakeMotorSpeeds.kIntakeOut,
            Constants.IntakeMotorSpeeds.kBTOut,
            0,
            false
        ),
    D
        (
            "Shooting with intake out",
            0,
            Constants.IntakeMotorSpeeds.kBTOut,
            Constants.IntakeMotorSpeeds.kLoaderLoadingSpeed,
            true
        ),
    E
        (
            "Shooting with intake in",
            0,
            Constants.IntakeMotorSpeeds.kBTOut,
            Constants.IntakeMotorSpeeds.kLoaderLoadingSpeed,
            false
        ),
    F
        (
            "Moving Ball To Color Sensor",
            0,
            Constants.IntakeMotorSpeeds.kBTOut,
            Constants.IntakeMotorSpeeds.kLoaderOut,
            false
        );

    public final String name;
    public final double intakeSpeed;
    public final double ballTransportSpeed;
    public final double loaderSpeed;
    public final boolean pneumaticOut;
    public boolean isReversable = false;

    IntakeState(String name, double intakeSpeed, double ballTransportSpeed, double loaderSpeed, boolean pnuematicOut) {
        this.name = name;
        this.intakeSpeed = intakeSpeed;
        this.ballTransportSpeed = ballTransportSpeed;
        this.loaderSpeed = loaderSpeed;
        this.pneumaticOut = pnuematicOut;
    }

    public void setReversable() {
        isReversable = true;
    }
}