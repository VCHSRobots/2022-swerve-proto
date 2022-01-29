// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.XboxController;

/** Add your docs here. */
public class xbox4415 extends XboxController implements Sendable {
    xbox4415(int port) {
        super(port);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("xbox");
        builder.addDoubleProperty("Left X", ()->getLeftX(), null);
        builder.addDoubleProperty("Left Y", ()->getLeftY(), null);
        builder.addDoubleProperty("Right X", ()->getRightX(), null);
        builder.addDoubleProperty("Right Y", ()->getRightY(), null);
        builder.addDoubleProperty("Left Trigger", ()->getLeftTriggerAxis(), null);
        builder.addDoubleProperty("Right Trigger", ()->getRightTriggerAxis(), null);
        builder.addDoubleProperty("POV", ()->getPOV(), null);
        
    }
}
