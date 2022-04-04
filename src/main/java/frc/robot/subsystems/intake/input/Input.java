package frc.robot.subsystems.intake.input;

public class Input {
    
    private InputActions inputActions;

    public Input(InputActions inputActions) {
        this.inputActions = inputActions;
    }

    public void startIntake(boolean value) {
        if(value) {
            inputActions.startIntakeAction();
        }
    }
    public void stopIntake(boolean value) {
        if(value) {
            inputActions.stopIntakeAction();
        }
    }
    public void isBallAtLoad(boolean value) {
        if(value) {
            inputActions.isBallAtLoadAction();
        }
    }
    public void isBallAtLoadAndMiddle(boolean value) {
        if(value) {
            inputActions.isBallAtLoadAndMiddleAction();
        }
    }
    public void isBallAtLoadOrMiddle(boolean value) {
        if(value) {
            inputActions.isBallAtLoadOrMiddleAction();
        }
    }

}
