package org.tahomarobotics.robot.arm;

import com.ctre.phoenix6.hardware.TalonFX;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.AbstractSubsystem;

public class ArmSubsystem extends AbstractSubsystem {
    // Motors
    private final TalonFX armMotor = new TalonFX(RobotMap.ARM_MOTOR);
    private final TalonFX wristMotor = new TalonFX(RobotMap.WRIST_MOTOR);

    // Setters

    public void setArmPosition() {

    }

    public void setWristPosition() {

    }

    // Getters

    public void getArmPosition() {

    }

    public void getWristPosition() {

    }

    // Implement abstract method in AbstractSubsystem
    @Override
    public void subsystemPeriodic() {

    }
}