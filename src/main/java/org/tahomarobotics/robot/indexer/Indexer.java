package org.tahomarobotics.robot.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tahomarobotics.robot.util.SubsystemIF;
import org.tahomarobotics.robot.util.sysid.SysIdTests;

import java.util.List;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static org.tahomarobotics.robot.indexer.IndexerConstants.*;

public class Indexer extends SubsystemIF {
    private static final Indexer INSTANCE = new Indexer();

    // -- Member Variables --

    // Hardware

    private final TalonFX motor;

    // Status Signals

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Current> current;

    // Control Requests

    private final MotionMagicVelocityVoltage velocityControl = new MotionMagicVelocityVoltage(0);
    private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0).withSlot(1);

    // State

    @Logged
    private IndexerConstants.IndexerState state = IndexerConstants.IndexerState.DISABLED;

    // -- Initialization --

    private Indexer() {
        // Create hardware

        motor = new TalonFX(RobotMap.INDEXER_MOTOR);

        // Configure hardware

        RobustConfigurator.tryConfigureTalonFX("Indexer Motor", motor, configuration);

        // Bind status signals

        position = motor.getPosition();
        velocity = motor.getVelocity();
        current = motor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
            RobotConfiguration.MECHANISM_UPDATE_FREQUENCY, position, velocity, current);
        ParentDevice.optimizeBusUtilizationForAll(motor);
    }

    public static Indexer getInstance() {
        return INSTANCE;
    }

    // -- State Machine --

    private void setTargetState(IndexerConstants.IndexerState state) {
        this.state = state;

        switch (state.type) {
            case POSITION -> {
                zeroPosition();
                motor.setControl(positionControl.withPosition(state.value));
            }
            case VELOCITY -> motor.setControl(velocityControl.withVelocity(state.value));
            case NONE -> motor.stopMotor();
        }
    }

    private void stateMachine() {
        // TODO: These states are currently bypassed due to the indexer being unable to serialize
        //       the piece to a known position.
        switch (state) {
            case INDEXING -> {
                if (isCoralInRollers()) {
                    transitionToSerializing();
                }
            }
            case SERIALIZING -> {
                if (isCollected()) {
                    transitionToCollected();
                }
            }
            default -> {}
        }
    }

    // Transitions

    public void transitionToDisabled() {
        if (state == IndexerState.INDEXING || state == IndexerState.EJECTING) {
            setTargetState(IndexerConstants.IndexerState.DISABLED);
        }
    }

    public void transitionToIndexing() {
        if (state != IndexerConstants.IndexerState.DISABLED) {
            return;
        }

        setTargetState(IndexerConstants.IndexerState.INDEXING);
    }

    private void transitionToSerializing() {
        setTargetState(IndexerConstants.IndexerState.SERIALIZING);
    }

    private void transitionToCollected() {
        setTargetState(IndexerConstants.IndexerState.COLLECTED);
    }

    public void transitionToEjecting() {
        setTargetState(IndexerConstants.IndexerState.EJECTING);
    }

    // -- Getter(s) --

    @Logged
    public double getPosition() {
        return position.getValueAsDouble();
    }

    @Logged
    public double getVelocity() {
        return velocity.getValueAsDouble();
    }

    @Logged
    public double getSupplyCurrent() {
        return current.getValueAsDouble();
    }

    // -- Triggers --

    private boolean isCoralInRollers() {
        // TODO: Add beam break on comp bot.
        return false;
    }

    private boolean isCollected() {
        if (state.type != IndexerState.MotionType.POSITION) { return false; }
        return Math.abs(state.value - getPosition()) < POSITION_THRESHOLD;
    }

    // -- Setter(s) --

    private void zeroPosition() {
        motor.setPosition(0);
    }

    // -- Periodic --

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(position, velocity, current);

        stateMachine();
    }

    // -- SysId --

    @Override
    public List<SysIdTests.Test> getSysIdTests() {
        return List.of(
            SysIdTests.characterize(
                "Indexer SysId Test",
                this,
                motor,
                Volts.of(1).per(Second),
                Volts.of(3)
            ));
    }
}


