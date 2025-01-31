package org.tahomarobotics.robot.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tahomarobotics.robot.util.SubsystemIF;
import org.tahomarobotics.robot.util.persistent.CalibrationData;
import org.tahomarobotics.robot.util.sysid.SysIdTests;
import org.tinylog.Logger;

import java.util.List;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static org.tahomarobotics.robot.elevator.ElevatorConstants.*;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class Elevator extends SubsystemIF {
    private static final Elevator INSTANCE = new Elevator();

    // -- Member Variables --

    // Hardware

    private final TalonFX leftMotor, rightMotor;
    private final CANcoder encoder;

    // Status Signals

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Current> current;

    // Control Request(s)

    private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0.0);

    // State

    private double targetHeight;

    private boolean calibrated;
    private double angularOffset;
    private final CalibrationData<Double> calibration;

    // -- Initialization --

    private Elevator() {
        // Create hardware

        leftMotor = new TalonFX(RobotMap.ELEVATOR_LEFT_MOTOR);
        rightMotor = new TalonFX(RobotMap.ELEVATOR_RIGHT_MOTOR);
        encoder = new CANcoder(RobotMap.ELEVATOR_ENCODER);

        // Configure hardware

        RobustConfigurator.tryConfigureTalonFX("Elevator Left Motor", leftMotor, motorConfiguration);

        // Load previous calibration

        calibration = new CalibrationData<>("ElevatorCalibration", 0d);

        angularOffset = calibration.get();
        calibrated = calibration.isCalibrated();

        applyOffset();

        if (!calibrated) {
            Logger.error("Elevator is not calibrated, cannot use until calibration is completed!");
        }

        // Status signals

        position = leftMotor.getPosition();
        velocity = leftMotor.getVelocity();
        current = leftMotor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
            RobotConfiguration.MECHANISM_UPDATE_FREQUENCY,
            position, velocity, current,
            // Used for fusing the motor's position and velocity
            encoder.getPosition(), encoder.getVelocity()
        );

        ParentDevice.optimizeBusUtilizationForAll(leftMotor, rightMotor, encoder);
    }

    public static Elevator getInstance() {
        return INSTANCE;
    }

    @Override
    public SubsystemIF initialize() {
        // Publish calibration command
        SmartDashboard.putData("Calibrate Elevator", ElevatorCommands.createCalibrateElevator(this));

        return this;
    }

    // -- Calibration --

    public void initializeCalibration() {
        RobustConfigurator.trySetCancoderAngularOffset("Elevator Encoder", encoder, 0);
        RobustConfigurator.trySetMotorNeutralMode("Elevator Left Motor", leftMotor, NeutralModeValue.Coast);
    }

    public void finalizeCalibration() {
        angularOffset = -position.refresh().getValueAsDouble() / MAIN_PULLEY_CIRCUMFERENCE;
        applyOffset();

        calibration.set(angularOffset);
        calibrated = true;
    }

    public void applyOffset() {
        RobustConfigurator.trySetCancoderAngularOffset("Elevator Encoder", encoder, angularOffset);
        RobustConfigurator.trySetMotorNeutralMode("Elevator Left Motor", leftMotor, NeutralModeValue.Brake);
    }

    // -- Getters --

    @Logged(name = "elevatorHeight")
    public double getElevatorHeight() {
        return position.getValueAsDouble();
    }

    @Logged(name = "elevatorTarget")
    public double getElevatorTarget() {
        return targetHeight;
    }

    @Logged
    public boolean isAtPosition() {
        return Math.abs(targetHeight - getElevatorHeight()) <= POSITION_TOLERANCE;
    }

    @Logged
    public boolean isMoving() {
        return Math.abs(velocity.refresh().getValueAsDouble()) > VELOCITY_TOLERANCE;
    }

    @Logged
    public boolean isInSecondStage() {
        return getElevatorHeight() < ELEVATOR_LOW_STAGE_MAX;
    }

    @Logged
    public double getSupplyCurrent() {
        return current.getValueAsDouble();
    }

    // -- Control --

    public void setElevatorHeight(double height) {
        if (!calibrated) {
            Logger.error("Cannot move elevator without calibration!");
            return;
        }

        targetHeight = MathUtil.clamp(height, ELEVATOR_MIN_POSE, ELEVATOR_MAX_POSE);
        rightMotor.setControl(
            new Follower(RobotMap.ELEVATOR_LEFT_MOTOR, true)
        );
        leftMotor.setControl(
            positionControl.withPosition(targetHeight)
        );
    }

    public void stop() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

    // -- Periodic --

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(position, velocity, current);
    }

    // -- Overrides --

    @Override
    public void onDisabledInit() {
        stop();
    }

    @Override
    public void onTeleopInit() {
        setElevatorHeight(ELEVATOR_LOW_POSE);
    }

    @Override
    public List<SysIdTests.Test> getSysIdTests() {
        return List.of(
            SysIdTests.characterize(
                "Elevator", this, leftMotor,
                Volts.of(0.25).per(Second), Volts.of(1),
                rightMotor
            )
        );
    }
}
