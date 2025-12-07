package org.tahomarobotics.robot.arm;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
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
    private final PositionVoltage positionControl = new PositionVoltage(0);

    // Status signals
    private final StatusSignal<Angle> armMotorPosition = armMotor.getPosition();
    private final StatusSignal<Angle> wristMotorPosition = wristMotor.getPosition();

    // Setters

    public void setArmPosition(DoubleSupplier rightYSupplier) {
        double y = rightYSupplier.getAsDouble();
        Logger.recordOutput("Arm/Right Y Axis", y);

        double targetPositionDouble = armMotorPosition.getValueAsDouble();
        if (y > 0) {
            targetPositionDouble += ArmConstants.ARM_INCREMENT;
        } else if (y < 0) {
            targetPositionDouble -= ArmConstants.ARM_INCREMENT;
        }

        Angle targetPosition = Degrees.of(MathUtil.clamp(targetPositionDouble, ArmConstants.ARM_MIN_POSITION, ArmConstants.ARM_MAX_POSITION));
        armMotor.setControl(positionControl.withPosition(targetPosition));
        Logger.recordOutput("Arm/Target Arm Position", targetPosition);
    }

    public void setWristPositionClockwise() {
        double targetPositionDouble = wristMotorPosition.getValueAsDouble() + ArmConstants.WRIST_INCREMENT;
        Angle targetPosition = Degrees.of(MathUtil.clamp(targetPositionDouble, ArmConstants.WRIST_MIN_POSITION, ArmConstants.WRIST_MAX_POSITION));
        wristMotor.setControl(positionControl.withPosition(targetPosition));
        Logger.recordOutput("Arm/Target Wrist Position", targetPosition);
    }

    public void setWristPositionCounterclockwise() {
        double targetPositionDouble = wristMotorPosition.getValueAsDouble() - ArmConstants.WRIST_INCREMENT;
        Angle targetPosition = Degrees.of(MathUtil.clamp(targetPositionDouble, ArmConstants.WRIST_MIN_POSITION, ArmConstants.WRIST_MAX_POSITION));
        wristMotor.setControl(positionControl.withPosition(targetPosition));
        Logger.recordOutput("Arm/Target Wrist Position", targetPosition);
    }

    @Override
    public void subsystemPeriodic() {
        armMotorPosition.refresh();
        wristMotorPosition.refresh();

        Logger.recordOutput("Arm/Arm Motor Position", armMotorPosition.getValue().in(Degrees));
        Logger.recordOutput("Arm/Wrist Motor Position", wristMotorPosition.getValue().in(Degrees));
    }
}