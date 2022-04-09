package frc.robot.subsystems.intake.input;

import java.util.*;

import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeState;

public class InputManager {
    
    private HashMap<IntakeState, Input> inputs = new HashMap<>();
    private Intake intake;

    public InputManager(HashMap<IntakeState, Input> inputs, Intake intake) {
        this.inputs = inputs;
        this.intake = intake;
    }

    public void manageInputs(boolean startIntake, boolean stopIntake, boolean ballAtMiddle, boolean ballAtLoad, boolean ballAtColorSensor) {
        Input i = inputs.get(intake.m_state);
        i.startIntake(startIntake);
        i.stopIntake(stopIntake);
        i.isBallAtLoad(ballAtLoad);
        i.isBallOnlyAtLoad(ballAtLoad && !ballAtMiddle);
        i.isBallAtLoadAndMiddle(ballAtMiddle && ballAtLoad);
        i.isBallAtColorSensor(ballAtColorSensor);
        i.isBallAtLoadOrMiddle(ballAtMiddle || ballAtLoad);
    }

}
