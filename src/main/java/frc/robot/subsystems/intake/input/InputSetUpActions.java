package frc.robot.subsystems.intake.input;

import java.util.HashMap;

import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeState;

public class InputSetUpActions {
    
    private Intake intake;

    public InputSetUpActions(Intake intake) {
        this.intake = intake;
    }

    public InputManager getNewInputManager() {
        HashMap<IntakeState, Input> inputs = new HashMap<>();

        inputs.put(IntakeState.A, setUpInputA());
        inputs.put(IntakeState.B, setUpInputB());
        inputs.put(IntakeState.C, setUpInputC());
        inputs.put(IntakeState.CA, setUpInputCA());
        inputs.put(IntakeState.D, setUpInputD());
        inputs.put(IntakeState.E, setUpInputE());
        inputs.put(IntakeState.F, setUpInputF());

        return new InputManager(inputs, intake);
    }

    // STATE AAAAAA

    // intake and load off
    // intake up
    private Input setUpInputA() {
        InputActions inputActionsA = new InputActions() {

            @Override
            public void startIntakeAction() {
                if (intake.isBallAtMiddle() && intake.isBallAtLoad()) {
                    intake.setState(IntakeState.A);
                } else {
                    intake.setState(IntakeState.B);
                }
            }

            @Override
            public void isBallAtLoadAndMiddleAction() {
                intake.setState(IntakeState.F);
            }
            
        };
        return new Input(inputActionsA);
    }

    // STATE BBBBBB

    // intake, mover, ON, loader ON
    // intake down
    private Input setUpInputB() {
        InputActions inputActionsB = new InputActions() {

            @Override
            public void stopIntakeAction() {
                intake.setState(IntakeState.A);
            }

            @Override
            public void isBallAtLoadAction() {
                intake.setState(IntakeState.C);
            }

        };
        return new Input(inputActionsB);
    }

    // STATE CCCCCC

    // intake, mover, ON, loader OFF
    // intake down
    // goes to state CA when color sensor tripped
    private Input setUpInputC() {
        InputActions inputActionsC = new InputActions() {

            @Override
            public void stopIntakeAction() {
                intake.setState(IntakeState.A);
            }

            @Override
            public void isBallAtColorSensorAction() {
                intake.setState(IntakeState.CA);
            }
        };
        return new Input(inputActionsC);
    }

    // STATE CCCCCAAAAAA

    // intake, mover, ON, loader OFF
    // intake in
    // goes to state A when two balls successfully loaded
    private Input setUpInputCA() {
        InputActions inputActionsCA = new InputActions() {

            @Override
            public void stopIntakeAction() {
                intake.setState(IntakeState.A);
            }

            @Override
            public void isBallAtLoadAndMiddleAction() {
                intake.setState(IntakeState.A);
            }

        };
        return new Input(inputActionsCA);
    }

    // STATE DDDDDD

    // start loading balls into shooter (load shooter)
    // BUT WITH INTAKE DOWN
    // stops when no shooter buttons are pressed2
    private Input setUpInputD() {
        InputActions inputActionsD = new InputActions() {

            @Override
            public void startIntakeAction() {
                intake.setState(IntakeState.B);
            }

            @Override
            public void stopIntakeAction() {
                intake.setState(IntakeState.A);
            }

        };
        return new Input(inputActionsD);
    }

    // STATE EEEEEEE

    // start loading balls into shooter (load shooter)
    //
    // stops when no more shooter buttons are pressed
    // or changes to state D when ball only at load
    private Input setUpInputE() {
        InputActions inputActionsE = new InputActions() {

            @Override
            public void startIntakeAction() {
                intake.setState(IntakeState.B);
            }

            @Override
            public void stopIntakeAction() {
                intake.setState(IntakeState.A);
            }

            @Override
            public void isBallOnlyAtLoadAction() {
                intake.setState(IntakeState.D);
            }

        };
        return new Input(inputActionsE);
    }

    // STATE FFFFFFF

    // move ball at color sensor location to shooter loader
    // intake up
    private Input setUpInputF() {
        InputActions inputActionsF = new InputActions() {
            @Override
            public void startIntakeAction() {
                intake.setState(IntakeState.B);
            }

            @Override
            public void stopIntakeAction() {
                intake.setState(IntakeState.A);
            }
            
            @Override
            public void isBallAtLoadAction() {
                intake.setState(IntakeState.A);
            }

        };
        return new Input(inputActionsF);
    }
}
