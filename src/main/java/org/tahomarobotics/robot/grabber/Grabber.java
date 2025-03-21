/*
 * Copyright 2025 Tahoma Robotics
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

package org.tahomarobotics.robot.grabber;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.AutoLogOutput;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.collector.Collector;
import org.tahomarobotics.robot.indexer.Indexer;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tahomarobotics.robot.util.SubsystemIF;
import org.tahomarobotics.robot.util.game.GamePiece;
import org.tahomarobotics.robot.util.signals.LoggedStatusSignal;
import org.tahomarobotics.robot.util.sysid.SysIdTests;
import org.tahomarobotics.robot.windmill.Windmill;
import org.tahomarobotics.robot.windmill.WindmillConstants;
import org.tahomarobotics.robot.windmill.commands.WindmillMoveCommand;
import org.tahomarobotics.robot.windmill.commands.WindmillTrajectories;

import java.util.List;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static org.tahomarobotics.robot.grabber.GrabberConstants.*;
import static org.tahomarobotics.robot.windmill.WindmillConstants.LARGE_PULLBACK;

public class Grabber extends SubsystemIF {
    private final static Grabber INSTANCE = new Grabber();

    // -- Member Variables --

    // Hardware

    private final TalonFX motor;

    // Status Signals

    private final StatusSignal<Current> current;

    private final LoggedStatusSignal[] statusSignals;

    // Control requests

    private final MotionMagicVelocityVoltage velocityControl = new MotionMagicVelocityVoltage(0).withEnableFOC(
        RobotConfiguration.RIO_PHOENIX_PRO);
    private final VoltageOut voltageControl = new VoltageOut(0);

    // State

    @AutoLogOutput(key = "Grabber/State")
    private GrabberState state = GrabberState.DISABLED;

    final Timer coralCollectionTimer = new Timer();
    final Timer algaeCollectionTimer = new Timer();
    private final Timer belowTimer = new Timer();

    // -- Initialization --

    private Grabber() {
        // Create hardware

        motor = new TalonFX(RobotMap.END_EFFECTOR_MOTOR);

        // Configure hardware

        RobustConfigurator.tryConfigureTalonFX("Grabber Motor", motor, motorConfig);

        // Bind status signals

        current = motor.getSupplyCurrent();

        statusSignals = new LoggedStatusSignal[]{
            new LoggedStatusSignal("Grabber Velocity", motor.getVelocity()),
            new LoggedStatusSignal("Grabber Current", current)
        };

        LoggedStatusSignal.setUpdateFrequencyForAll(statusSignals, RobotConfiguration.MECHANISM_UPDATE_FREQUENCY);
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
        if (state == GrabberState.CORAL_COLLECTING) {
            boolean isTripped = indexer.isBeanBakeTripped();

            if (isTripped && Collector.getInstance().getCollectionMode() != GamePiece.ALGAE) {
                coralCollectionTimer.start();

                belowTimer.reset();
                belowTimer.stop();
            } else if (belowTimer.hasElapsed(0.1)) {
                coralCollectionTimer.stop();
                coralCollectionTimer.reset();
            } else {
                belowTimer.restart();
            }
        }
        
        if (state == GrabberState.ALGAE_COLLECTING) {
            boolean isTripped = getCurrent() > ALGAE_COLLECTION_CURRENT_THRESHOLD;

            if (isTripped && Collector.getInstance().getCollectionMode() != GamePiece.CORAL) {
                algaeCollectionTimer.start();

                belowTimer.reset();
                belowTimer.stop();
            } else if (belowTimer.hasElapsed(0.05)) {
                algaeCollectionTimer.stop();
                algaeCollectionTimer.reset();
            } else {
                belowTimer.restart();
            }
        }

        if (algaeCollectionTimer.hasElapsed(ALGAE_COLLECTION_DELAY)) {
            transitionToAlgaeHolding();

            Windmill.getInstance().createTransitionCommand(((WindmillConstants.TrajectoryState.STOW))).schedule();

            algaeCollectionTimer.stop();
            algaeCollectionTimer.reset();
        }

        if (coralCollectionTimer.hasElapsed(CORAL_COLLECTION_DELAY)) {
            transitionToCoralHolding();
            indexer.transitionToDisabled();

            coralCollectionTimer.stop();
            coralCollectionTimer.reset();
        }
    }

    // Transitions

    public void transitionToDisabled() {
        if (isHoldingCoral()) { return; }
        if (algaeCollectionTimer.isRunning() || coralCollectionTimer.isRunning()) { return; }
        setTargetState(GrabberState.DISABLED);
    }

    public void transitionToCoralHolding() {
        setTargetState(GrabberState.CORAL_HOLDING);
    }

    public void transitionToAlgaeHolding() {
        setTargetState(GrabberState.ALGAE_HOLDING);
    }

    public void transitionToAlgaeCollecting() {
        if (!isArmAtPassing() || isHoldingCoral()) { return; }
        setTargetState(GrabberState.ALGAE_COLLECTING);
    }

    public void transitionToCoralCollecting() {
        if (!isArmAtPassing() || isHoldingCoral()) { return; }
        setTargetState(GrabberState.CORAL_COLLECTING);
    }
    public void transitionToScoring() {
        setTargetState(GrabberState.SCORING);
        coralCollectionTimer.stop();
        coralCollectionTimer.reset();
        algaeCollectionTimer.stop();
        algaeCollectionTimer.reset();
    }

    public void transitionToScoringL1() {
        setTargetState(GrabberState.L1_SCORING);
    }

    // -- Getters --

    public boolean isArmAtPassing() {
        return SmartDashboard.getBoolean("arm at position", true);
    }

    public boolean isHoldingCoral() {
        return state == GrabberState.CORAL_HOLDING;
    }

    public boolean isHoldingAlgae() {
        return state == GrabberState.ALGAE_HOLDING;
    }

    public double getCurrent() {
        return current.getValueAsDouble();
    }

    // -- Periodic --

    @Override
    public void periodic() {
        LoggedStatusSignal.refreshAll(statusSignals);
        LoggedStatusSignal.log("Grabber/", statusSignals);

        stateMachine();
    }

    @Override
    public void onDisabledInit() {
        transitionToDisabled();
    }

    // -- Autonomous --

    @Override
    public void onAutonomousInit() {
        transitionToCoralHolding();
    }

    // -- SysId --

    @Override
    public List<SysIdTests.Test> getSysIdTests() {
        return List.of(
            SysIdTests.characterize(
                "Grabber",
                this,
                motor,
                Volts.of(1).per(Second),
                Volts.of(3)
            )
        );
    }
}
