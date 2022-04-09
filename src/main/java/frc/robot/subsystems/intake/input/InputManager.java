package frc.robot.subsystems.intake.input;

import java.util.*;

public class InputManager {
    
    public ArrayList<Input> inputs = new ArrayList<>();

    public InputManager(ArrayList<Input> inputs) {
        this.inputs = inputs;
    }

    public void manageInputs(boolean startIntake, boolean stopIntake, boolean ballAtMiddle, boolean ballAtLoad) {
        for(Input i : inputs) {
            i.startIntake(startIntake);
            i.stopIntake(stopIntake);
            i.isBallAtLoad(ballAtLoad);
            i.isBallOnlyAtLoad(ballAtLoad && !ballAtMiddle);;
            i.isBallAtLoadAndMiddle(ballAtMiddle && ballAtLoad);
            i.isBallAtLoadOrMiddle(ballAtMiddle || ballAtLoad);
        }
    }

}
