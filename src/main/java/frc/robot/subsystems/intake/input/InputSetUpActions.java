package frc.robot.subsystems.intake.input;

import java.util.ArrayList;

import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeState;

public class InputSetUpActions {
    
    private Intake intake;

    public InputSetUpActions(Intake intake) {
        this.intake = intake;
    }

    public InputManager getNewInputManager() {
        ArrayList<Input> inputs = new ArrayList<>();

        inputs.add(setUpInputA());
        inputs.add(setUpInputB());
        inputs.add(setUpInputC());
        inputs.add(setUpInputD());
        inputs.add(setUpInputE());
        inputs.add(setUpInputF());

        return new InputManager(inputs);
    }

    // STATE AAAAAA

    // intake and load off, intake up
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
            public void stopIntakeAction() {
                
                
            }

            @Override
            public void isBallAtLoadAction() {
                
                
            }

            @Override
            public void isBallAtLoadAndMiddleAction() {
                intake.setState(IntakeState.F);
            }

            @Override
            public void isBallAtLoadOrMiddleAction() {
                
                
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
            public void startIntakeAction() {
                
                
            }

            @Override
            public void isBallAtLoadAction() {
                intake.setState(IntakeState.C);
            }

            @Override
            public void isBallAtLoadAndMiddleAction() {
                
                
            }

            @Override
            public void isBallAtLoadOrMiddleAction() {
                
                
            }

        };
        return new Input(inputActionsB);
    }

    // STATE CCCCCC

    // intake, mover, ON, loader OFF
    // intake down
    private Input setUpInputC() {
        InputActions inputActionsC = new InputActions() {

            @Override
            public void startIntakeAction() {
                
                
            }

            @Override
            public void stopIntakeAction() {
                intake.setState(IntakeState.A);
            }

            @Override
            public void isBallAtLoadAction() {
                intake.setState(IntakeState.B);
            }

            @Override
            public void isBallAtLoadAndMiddleAction() {
                intake.setState(IntakeState.A);
            }

            @Override
            public void isBallAtLoadOrMiddleAction() {
                
                
            }

        };
        return new Input(inputActionsC);
    }

    // STATE DDDDDD

    // previously used state
    private Input setUpInputD() {
        InputActions inputActionsD = new InputActions() {

            @Override
            public void startIntakeAction() {
                
                
            }

            @Override
            public void stopIntakeAction() {
                
                
            }

            @Override
            public void isBallAtLoadAction() {
                
                
            }

            @Override
            public void isBallAtLoadAndMiddleAction() {
                
                
            }

            @Override
            public void isBallAtLoadOrMiddleAction() {
                
                
            }

        };
        return new Input(inputActionsD);
    }

    // STATE EEEEEEE

    // start loading balls into shooter (loadShooter)
    // stops when no more shooter buttons are pressed
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
            public void isBallAtLoadAction() {
                
                
            }

            @Override
            public void isBallAtLoadAndMiddleAction() {
                
                
            }

            @Override
            public void isBallAtLoadOrMiddleAction() {
                
                
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

            @Override
            public void isBallAtLoadAndMiddleAction() {
                
                
            }

            @Override
            public void isBallAtLoadOrMiddleAction() {
                
                
            }
        };
        return new Input(inputActionsF);
    }
}
