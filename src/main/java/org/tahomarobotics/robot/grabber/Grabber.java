package org.tahomarobotics.robot.grabber;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.indexer.Indexer;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tahomarobotics.robot.util.SubsystemIF;
import org.tahomarobotics.robot.util.signals.LoggedStatusSignal;
import org.tahomarobotics.robot.util.sysid.SysIdTests;

import java.util.List;

import static edu.wpi.first.units.Units.*;
import static org.tahomarobotics.robot.grabber.GrabberConstants.*;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class Grabber extends SubsystemIF {
    private final static Grabber INSTANCE = new Grabber();

    // -- Member Variables --

    // Hardware

    private final TalonFX motor;

    // Status Signals

    private final StatusSignal<AngularVelocity> grabberVelocity;
    private final StatusSignal<Current> current;

    @Logged
    private final LoggedStatusSignal.List statusSignals;

    // Control requests

    private final MotionMagicVelocityVoltage velocityControl = new MotionMagicVelocityVoltage(0).withEnableFOC(
        RobotConfiguration.RIO_PHOENIX_PRO);
    private final VoltageOut voltageControl = new VoltageOut(0);

    // State

    @Logged
    private GrabberState state = GrabberState.DISABLED;

    // -- Initialization --

    private Grabber() {
        // Create hardware

        motor = new TalonFX(RobotMap.END_EFFECTOR_MOTOR);

        // Configure hardware

        RobustConfigurator.tryConfigureTalonFX("Grabber Motor", motor, motorConfig);

        // Bind status signals

        grabberVelocity = motor.getVelocity();
        current = motor.getSupplyCurrent();

        statusSignals = new LoggedStatusSignal.List(List.of(
            new LoggedStatusSignal("Grabber Velocity", grabberVelocity),
            new LoggedStatusSignal("Grabber Current", current)
        ));

        statusSignals.setUpdateFrequencyForAll(RobotConfiguration.MECHANISM_UPDATE_FREQUENCY);
        ParentDevice.optimizeBusUtilizationForAll(motor);
    }

    public static Grabber getInstance() {
        return INSTANCE;
    }

    private static final Indexer indexer = Indexer.getInstance();

    // -- State Machine --

    public void setTargetState(GrabberState state) {
        this.state = state;

        switch (state.type) {
            case NONE -> motor.stopMotor();
            case VELOCITY -> motor.setControl(velocityControl.withVelocity(state.value));
            case VOLTAGE -> motor.setControl(voltageControl.withOutput(state.value));
        }
    }

    private void stateMachine() {
        switch(state) {
            case COLLECTING -> {
                if (indexer.isCollected()) {
                    transitionToHolding();
                }
            }
        }
    }

    // Transitions

    public void transitionToDisabled() {
        if (isHolding()) return;
        setTargetState(GrabberState.DISABLED);
    }

    public void transitionToHolding() {
        setTargetState(GrabberState.HOLDING);
    }

    public void transitionToCollecting() {
        if (!isArmAtPassing() || isHolding()) return;
        setTargetState(GrabberState.COLLECTING);
    }

    public void transitionToEjecting() {
        setTargetState(GrabberState.EJECTING);
    }

    public void transitionToScoring() {
        setTargetState(GrabberState.SCORING);
    }

    // -- Getters --

    public boolean isArmAtPassing() {
        // TODO: Check if the arm is in the collecting position
        return SmartDashboard.getBoolean("arm at position", true);
    }

    @Logged
    public boolean isDisabled() {
        return state == GrabberState.DISABLED;
    }

    @Logged
    public boolean isHolding() {
        return state == GrabberState.HOLDING;
    }

    @Logged
    public boolean isCollecting() {
        return state == GrabberState.COLLECTING;
    }

    @Logged
    public boolean isEjecting() {
        return state == GrabberState.EJECTING;
    }

    // -- Periodic --

    @Override
    public void periodic() {
        statusSignals.refreshAll();
        stateMachine();
    }

    // -- SysId --

    @Override
    public List<SysIdTests.Test> getSysIdTests() {
        return List.of(
            SysIdTests.characterize(
                "Grabber",
                this,
                motor,
                Volts.of(0.25).per(Second),
                Volts.of(1)
            )
        );
    }
}
