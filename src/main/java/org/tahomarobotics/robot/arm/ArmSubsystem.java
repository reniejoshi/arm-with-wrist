package org.tahomarobotics.robot.arm;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.Logger;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.AbstractSubsystem;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Degrees;

public class ArmSubsystem extends AbstractSubsystem {
    // Motors
    private final TalonFX armMotor = new TalonFX(RobotMap.ARM_MOTOR);
    private final TalonFX wristMotor = new TalonFX(RobotMap.WRIST_MOTOR);

    // Control requests
    private final PositionVoltage positonControl = new PositionVoltage(0);

    // Status signals
    private final StatusSignal<Angle> armMotorPosition = armMotor.getPosition();
    private final StatusSignal<Angle> wristMotorPosition = wristMotor.getPosition();

    // Setters

    public void setArmPosition(DoubleSupplier rightYSupplier) {
        double y = rightYSupplier.getAsDouble();
        Logger.recordOutput("Arm/Right Y Axis", y);

        Angle targetPosition = armMotorPosition.getValue();
        if (y < 0) {
            targetPosition = Degrees.of(armMotorPosition.getValueAsDouble() + 1);
        } else if (y > 0) {
            targetPosition = Degrees.of(armMotorPosition.getValueAsDouble() - 1);
        }
        armMotor.setControl(positonControl.withPosition(targetPosition));
        Logger.recordOutput("Arm/Target Position", targetPosition);
    }

    public void setWristPosition(Angle position) {
        wristMotor.setControl(positonControl.withPosition(position));
    }

    // Getters

    public Angle getArmMotorPosition() {
        return armMotorPosition.getValue();
    }

    public Angle getWristMotorPosition() {
        return wristMotorPosition.getValue();
    }

    @Override
    public void subsystemPeriodic() {
        armMotorPosition.refresh();
        wristMotorPosition.refresh();

        Logger.recordOutput("Arm/Arm Motor Position", getArmMotorPosition().in(Degrees));
        Logger.recordOutput("Arm/Wrist Motor Position", getWristMotorPosition().in(Degrees));
    }
}