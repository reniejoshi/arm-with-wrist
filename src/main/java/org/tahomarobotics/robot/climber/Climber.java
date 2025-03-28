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

package org.tahomarobotics.robot.climber;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.AutoLogOutput;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.climber.commands.ClimberCommands;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tahomarobotics.robot.util.SubsystemIF;
import org.tahomarobotics.robot.util.signals.LoggedStatusSignal;
import org.tahomarobotics.robot.util.sysid.SysIdTests;
import org.tinylog.Logger;

import java.util.List;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static org.tahomarobotics.robot.climber.ClimberConstants.climberMotorConfig;

public class Climber extends SubsystemIF {
    private final static Climber INSTANCE = new Climber();

    // -- Member Variables --

    // Hardware

    private final TalonFX climberMotor = new TalonFX(RobotMap.CLIMBER_MOTOR);
    private final TalonFX climberFollower = new TalonFX(RobotMap.CLIMBER_FOLLOWER);
    private final TalonFX climberRoller = new TalonFX(RobotMap.CLIMBER_ROLLER);
    private final VictorSPX ratchetSolenoid = new VictorSPX(RobotMap.CLIMBER_RATCHET_SOLENOID);

    private final DigitalInput limitSwitch = new DigitalInput(RobotMap.CLIMBER_LIMIT_SWITCH);

    private final boolean USE_LIMIT_SWITCH = false;

    // Status Signals

    private final LoggedStatusSignal[] statusSignals;

    private final StatusSignal<Current> climberMotorCurrent;
    private final StatusSignal<Current> climberFollowerCurrent;
    private final StatusSignal<Current> climberRollerCurrent;
    private final StatusSignal<Angle> climberMotorPosition;

    // Control Requests

    private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0);

    // State

    private boolean coasting = false;

    @AutoLogOutput(key = "Climber/State")
    private ClimberState climbState = ClimberState.ZEROED;
    @AutoLogOutput(key = "Climber/Target Position")
    private double targetPosition;
    @AutoLogOutput(key = "Climber/Solenoid Voltage")
    private double solenoidVoltage;

    // -- Initialization --

    private Climber() {
        RobustConfigurator.tryConfigureTalonFX("Climber Motor", climberMotor, climberMotorConfig);
        climberFollower.setControl(new Follower(RobotMap.CLIMBER_MOTOR, true));

        climberMotorCurrent = climberMotor.getSupplyCurrent();
        climberFollowerCurrent = climberFollower.getSupplyCurrent();
        climberRollerCurrent = climberRoller.getSupplyCurrent();

        climberMotorPosition = climberMotor.getPosition();

        statusSignals = new LoggedStatusSignal[]{
            new LoggedStatusSignal("Climber Motor Position", climberMotorPosition),
            new LoggedStatusSignal("Climber Follower Position", climberFollower.getPosition()),
            new LoggedStatusSignal("Climber Motor Current", climberMotorCurrent),
            new LoggedStatusSignal("Climber Follower Current", climberFollowerCurrent),
            new LoggedStatusSignal("Climber Roller Current", climberRollerCurrent),
            new LoggedStatusSignal("Climber Motor Voltage", climberMotor.getMotorVoltage()),
            new LoggedStatusSignal("Climber Follower Voltage", climberFollower.getMotorVoltage()),
            new LoggedStatusSignal("Climber Roller Voltage", climberRoller.getMotorVoltage())
        };

        LoggedStatusSignal.setUpdateFrequencyForAll(statusSignals, RobotConfiguration.MECHANISM_UPDATE_FREQUENCY);
        ParentDevice.optimizeBusUtilizationForAll(climberMotor, climberFollower, climberRoller);
    }

    public static Climber getInstance() {
        return INSTANCE;
    }

    @Override
    public Climber initialize() {
        new Trigger(() -> RobotState.isEnabled() && !RobotState.isTest())
            .onTrue(
                runOnce(this::zeroPosition)
                    .andThen(ClimberCommands.getClimberCommand())
                    .onlyIf(() -> climbState.equals(ClimberState.ZEROED))
            );

        // Climb on limit switch
        new Trigger(() -> USE_LIMIT_SWITCH && isLimitSwitchPressed() && RobotState.isTeleop() && climbState == ClimberState.DEPLOYED)
            .onTrue(Commands.waitSeconds(0.5).andThen(
                ClimberCommands.getClimberCommand()
            ));

        // Debug
        SmartDashboard.putData("Disengage Solenoid", runOnce(this::disengageSolenoid));
        SmartDashboard.putData("Engage Solenoid", runOnce(this::engageSolenoid));
        SmartDashboard.putData("Toggle Coast Climber", runOnce(this::toggleCoast).ignoringDisable(true));

        return this;
    }

    // -- Getters --

    public ClimberState getClimbState() {
        return climbState;
    }

    @AutoLogOutput(key = "Climber/Is at Target Position?")
    public boolean isAtTargetPosition() {
        return (Math.abs(climberMotorPosition.getValueAsDouble() - targetPosition) < ClimberConstants.CLIMB_POSITION_TOLERANCE);
    }

    // -- Climb Control --

    public void wiggle() {
        targetPosition -= 0.01;

        Logger.info("WIGGLING");
        climberMotor.setControl(positionControl.withPosition(targetPosition).withSlot(0));
    }

    public void stow() {
        targetPosition = ClimberConstants.STOW_POSITION;

        Logger.info("Climber going to STOW");
        climberMotor.setControl(positionControl.withPosition(targetPosition).withSlot(0));

        climbState = ClimberState.STOWED;
    }

    public void deploy() {
        targetPosition = ClimberConstants.DEPLOY_POSITION;

        Logger.info("Climber going to DEPLOY");
//        climberMotor.setControl(positionControl.withPosition(targetPosition).withSlot(0));

        climbState = ClimberState.DEPLOYED;
    }

    public void climb() {
        targetPosition = ClimberConstants.CLIMB_POSITION;

        Logger.info("Climber going to CLIMB");
        climberMotor.setControl(positionControl.withPosition(targetPosition).withSlot(1));

        climbState = ClimberState.CLIMBED;
    }

    public void disengageSolenoid() {
        Logger.info("Disengaged solenoid...");
        ratchetSolenoid.set(VictorSPXControlMode.PercentOutput, ClimberConstants.RATCHET_SOLENOID_DEPLOY_PERCENTAGE);
    }

    public void engageSolenoid() {
        Logger.info("Engaged solenoid...");
        ratchetSolenoid.set(VictorSPXControlMode.PercentOutput, 0);
    }

    public void runRollers() {
        Logger.info("Running rollers...");
        climberRoller.set(-Math.PI / 10d);
    }

    public void disableRollers() {
        Logger.info("Disabling rollers...");
        climberRoller.set(0);
    }

    public void disableClimberMotors() {
        climberMotor.stopMotor();
    }

    public void zeroPosition() {
        climberMotor.setPosition(ClimberConstants.ZERO_POSITION);
        climberFollower.setPosition(ClimberConstants.ZERO_POSITION);
    }

    public void toggleCoast() {
        if (coasting) {
            climberMotor.setNeutralMode(NeutralModeValue.Brake);
            climberFollower.setNeutralMode(NeutralModeValue.Brake);
        } else {
            climberMotor.setNeutralMode(NeutralModeValue.Coast);
            climberFollower.setNeutralMode(NeutralModeValue.Coast);
        }
        coasting = !coasting;
    }

    // -- Getters --

    public double getLeadCurrent() {
        return climberMotorCurrent.getValueAsDouble();
    }

    public double getFollowerCurrent() {
        return climberFollowerCurrent.getValueAsDouble();
    }

    public double getRollersCurrent() {
        return climberRollerCurrent.getValueAsDouble();
    }

    @AutoLogOutput(key = "Climber/Is Limit Switch Pressed?")
    public boolean isLimitSwitchPressed() {
        return !limitSwitch.get();
    }

    // -- Periodic --

    @Override
    public void periodic() {
        LoggedStatusSignal.refreshAll(statusSignals);
        LoggedStatusSignal.log("Climber/", statusSignals);

        solenoidVoltage = ratchetSolenoid.getMotorOutputVoltage();
    }

    // -- States --

    public enum ClimberState {
        STOWED,
        DEPLOYED,
        CLIMBED,
        ZEROED
    }

    // -- SysId --

    @Override
    public List<SysIdTests.Test> getSysIdTests() {
        return List.of(SysIdTests.characterize(
            "Climber",
            this,
            climberMotor,
            Volts.of(0.25).per(Second),
            Volts.of(1),
            climberFollower,
            true
        ));
    }
}