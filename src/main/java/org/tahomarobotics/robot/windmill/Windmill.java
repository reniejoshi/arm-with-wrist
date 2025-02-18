package org.tahomarobotics.robot.windmill;

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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.RobustConfigurator;
import org.tahomarobotics.robot.util.SubsystemIF;
import org.tahomarobotics.robot.util.persistent.CalibrationData;
import org.tahomarobotics.robot.util.sysid.SysIdTests;
import org.tahomarobotics.robot.windmill.commands.WindmillCommands;
import org.tahomarobotics.robot.windmill.commands.WindmillMoveCommand;
import org.tinylog.Logger;

import java.util.List;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static org.tahomarobotics.robot.windmill.WindmillConstants.*;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class Windmill extends SubsystemIF {
    private static final Windmill INSTANCE = new Windmill();

    // -- Member Variables --

    // Hardware

    private final TalonFX elevatorLeftMotor;
    private final TalonFX elevatorRightMotor;
    private final TalonFX armMotor;
    private final CANcoder elevatorEncoder;
    private final CANcoder armEncoder;

    // Status Signals

    private final StatusSignal<Angle> elevatorPosition, armPosition;
    private final StatusSignal<AngularVelocity> elevatorVelocity, armVelocity;
    private final StatusSignal<Current> elevatorCurrent, armCurrent;

    // Control Requests

    private final MotionMagicVoltage elevatorPositionControl = new MotionMagicVoltage(0);
    private final MotionMagicVoltage armPositionControl = new MotionMagicVoltage(0);

    // State

    private double targetHeight;
    private double targetAngle;
    private TrajectoryState targetTrajectoryState = TrajectoryState.START;

    private boolean elevatorCalibrated = true, armCalibrated = true;

    private CalibrationData<Boolean> elevatorCalibration = null;
    private CalibrationData<Boolean> armCalibration = null;

    // Trajectory

    public final Field2d field = new Field2d();

    // -- Initialization --

    private Windmill() {
        // Create Hardware

        elevatorLeftMotor = new TalonFX(RobotMap.ELEVATOR_LEFT_MOTOR);
        elevatorRightMotor = new TalonFX(RobotMap.ELEVATOR_RIGHT_MOTOR);
        elevatorEncoder = new CANcoder(RobotMap.ELEVATOR_ENCODER);

        armMotor = new TalonFX(RobotMap.ARM_MOTOR);
        armEncoder = new CANcoder(RobotMap.ARM_ENCODER);

        // Configure hardware

        RobustConfigurator.tryConfigureCANcoder("Arm Encoder", armEncoder, armEncoderConfiguration);
        RobustConfigurator.tryConfigureTalonFX("Elevator Left Motor", elevatorLeftMotor, elevatorMotorConfiguration);
        elevatorRightMotor.setControl(new Follower(RobotMap.ELEVATOR_LEFT_MOTOR, true));
        RobustConfigurator.tryConfigureTalonFX("Arm Motor", armMotor, armMotorConfiguration);

        // Load previous calibrations

        if (RobotBase.isReal()) {
            elevatorCalibration = new CalibrationData<>("ElevatorCalibration", false);
            armCalibration = new CalibrationData<>("ArmCalibration", false);

            elevatorCalibrated = elevatorCalibration.get();
            armCalibrated = armCalibration.isCalibrated();

            applyArmOffset();

            if (!elevatorCalibrated) {
                Logger.error("Elevator is not calibrated, cannot use until calibration is completed!");
            }
            if (!armCalibrated) {
                Logger.error("Arm is not calibrated, cannot use until calibration is completed!");
            }
        }

        // Status Signals

        elevatorPosition = elevatorLeftMotor.getPosition();
        elevatorVelocity = elevatorLeftMotor.getVelocity();
        elevatorCurrent = elevatorLeftMotor.getSupplyCurrent();

        armPosition = armMotor.getPosition();
        armVelocity = armMotor.getVelocity();
        armCurrent = armMotor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
            RobotConfiguration.MECHANISM_UPDATE_FREQUENCY,
            elevatorPosition, elevatorVelocity, elevatorCurrent,
            armPosition, armVelocity, armCurrent,
            elevatorEncoder.getPosition(), elevatorEncoder.getVelocity(),
            armEncoder.getPosition(), armEncoder.getVelocity(),
            elevatorLeftMotor.getMotorVoltage(), armMotor.getMotorVoltage()
        );

        ParentDevice.optimizeBusUtilizationForAll(
            elevatorLeftMotor, elevatorRightMotor, elevatorEncoder, armMotor, armEncoder);
    }

    public static Windmill getInstance() {
        return INSTANCE;
    }

    @Override
    public SubsystemIF initialize() {
        // Publish calibration commands
        SmartDashboard.putData("Calibrate Elevator", WindmillCommands.createCalibrateElevatorCommand(this));
        SmartDashboard.putData("Calibrate Arm", WindmillCommands.createCalibrateArmCommand(this));

        return this;
    }

    // -- Calibration --

    public void initializeElevatorCalibration() {
        RobustConfigurator.trySetMotorNeutralMode("Elevator Left Motor", elevatorLeftMotor, NeutralModeValue.Coast);
    }

    public void initializeArmCalibration() {
        RobustConfigurator.trySetMotorNeutralMode("Arm Left Motor", armMotor, NeutralModeValue.Coast);
    }

    public void finalizeElevatorCalibration() {
        elevatorEncoder.setPosition(0);

        elevatorCalibration.set(true);
        elevatorCalibrated = true;

        RobustConfigurator.trySetMotorNeutralMode("Elevator Motor", elevatorLeftMotor, NeutralModeValue.Brake);
    }

    public void finalizeArmCalibration() {
        // Divide by sensor coefficient to set accurate position
        armEncoder.setPosition(ARM_UPRIGHT_POSE / ARM_BELT_REDUCTION);

        armCalibration.set(true);
        armCalibrated = true;

        RobustConfigurator.trySetMotorNeutralMode("Arm Motor", armMotor, NeutralModeValue.Brake);
    }

    public void applyElevatorOffset() {
        RobustConfigurator.trySetMotorNeutralMode("Elevator Left Motor", elevatorLeftMotor, NeutralModeValue.Brake);
    }

    public void applyArmOffset() {
        RobustConfigurator.trySetMotorNeutralMode("Arm Motor", armMotor, NeutralModeValue.Brake);
    }

    // -- Getters --

    @Logged(name = "windmillPosition")
    public Translation2d getWindmillPosition() {
        double armAngleRotations = MathUtil.inputModulus(getArmPosition(), 0, 1);

        return WindmillKinematics.forwardKinematics(
            elevatorPosition.getValueAsDouble(),
            Units.rotationsToRadians(armAngleRotations),
            true
        );
    }

    @Logged(name = "windmillX")
    public double getWindmillPositionX() {
        return getWindmillPosition().getX();
    }

    @Logged(name = "windmillY")
    public double getWindmillPositionY() {
        return getWindmillPosition().getY();
    }

    public List<BaseStatusSignal> getStatusSignals() {
        return List.of(elevatorPosition, armPosition, elevatorVelocity, armVelocity, elevatorCurrent, armCurrent);
    }

    public WindmillState getCurrentState() {
        WindmillState.ElevatorState elevatorState = new WindmillState.ElevatorState(
            elevatorPosition.getValueAsDouble(),
            elevatorVelocity.getValueAsDouble(),
            elevatorLeftMotor.getAcceleration().refresh().getValueAsDouble()
        );

        WindmillState.ArmState armState = new WindmillState.ArmState(
            armPosition.getValueAsDouble(),
            armVelocity.getValueAsDouble(),
            armMotor.getAcceleration().refresh().getValueAsDouble()
        );

        return new WindmillState(0, elevatorState, armState);
    }

    @Logged(name = "Target Trajectory State")
    public TrajectoryState getTargetTrajectoryState() {
        return targetTrajectoryState;
    }

    // Elevator

    @Logged(name = "elevatorHeight")
    public double getElevatorHeight() {
        return elevatorPosition.getValueAsDouble();
    }

    @Logged(name = "elevatorTarget")
    public double getElevatorTarget() {
        return targetHeight;
    }

    @Logged
    public boolean isElevatorAtPosition() {
        return Math.abs(targetHeight - getElevatorHeight()) <= ELEVATOR_POSITION_TOLERANCE;
    }

    @Logged
    public boolean isElevatorMoving() {
        return Math.abs(elevatorVelocity.refresh().getValueAsDouble()) >= ELEVATOR_VELOCITY_TOLERANCE;
    }

    @Logged
    public boolean isInSecondStage() {
        return getElevatorHeight() > ELEVATOR_LOW_STAGE_MAX;
    }

    // Arm

    @Logged(name = "armPosition")
    public double getArmPosition() {
        return armPosition.getValueAsDouble();
    }

    @Logged(name = "armTarget")
    public double getArmTarget() {
        return targetAngle;
    }

    @Logged(name = "armVelocity")
    public double getArmVelocity() {
        return armVelocity.getValueAsDouble();
    }

    @Logged(name = "armCurrent")
    public double getArmCurrent() {
        return armCurrent.getValueAsDouble();
    }

    @Logged
    public boolean isArmAtPosition() {
        return Math.abs(getArmPosition() - targetAngle) < ARM_POSITION_TOLERANCE;
    }

    @Logged
    public boolean isArmMoving() {
        return Math.abs(armVelocity.refresh().getValueAsDouble()) >= ARM_VELOCITY_TOLERANCE;
    }

    @Logged
    public double distanceToTargetState() {
        return targetTrajectoryState.t2d.getDistance(getWindmillPosition());
    }

    @Logged
    public boolean isAtTargetState() {
        return distanceToTargetState() < 0.03;
    }

    // -- Control --

    public void setTargetState(TrajectoryState targetState) {
        this.targetTrajectoryState = targetState;
    }

    public void setState(WindmillState state) {
        setElevatorHeight(state.elevatorState().heightMeters());
        setArmPosition(Units.radiansToRotations(state.armState().angleRadians()));
    }

    public void setElevatorHeight(double height) {
        if (!elevatorCalibrated) {
            Logger.error("Cannot move elevator without calibration!");
            return;
        }

        targetHeight = MathUtil.clamp(height, ELEVATOR_MIN_POSE, ELEVATOR_MAX_POSE);
        elevatorRightMotor.setControl(
            new Follower(RobotMap.ELEVATOR_LEFT_MOTOR, true)
        );
        elevatorLeftMotor.setControl(
            elevatorPositionControl.withPosition(targetHeight)
        );

        Logger.info("Set elevator height: " + targetHeight);
    }

    public void setArmPosition(double position) {
        if (!armCalibrated) {
            Logger.error("Cannot move arm without calibration!");
            return;
        }

        targetAngle = position;
        armMotor.setControl(armPositionControl.withPosition(targetAngle));

        Logger.info("Set arm position: " + targetAngle);
    }

    public Command createTransitionCommand(TrajectoryState to) {
        return Commands.deferredProxy(() -> WindmillMoveCommand.fromTo(targetTrajectoryState, to).orElseGet(Commands::none));
    }

    public Command createTransitionToggleCommand(TrajectoryState onTrue, TrajectoryState onFalse) {
        return Commands.deferredProxy(() -> {
            if (targetTrajectoryState == onFalse) {
                Logger.info("Toggling between {} and {}", onFalse, onTrue);
                return createTransitionCommand(onTrue);
            } else if (targetTrajectoryState == onTrue) {
                Logger.info("Toggling between {} and {}", onTrue, onFalse);
                return createTransitionCommand(onFalse);
            } else {
                return Commands.runOnce(() -> Logger.error("Cannot toggle from an untoggleable state."));
            }
        });
    }

    public void stopElevator() {
        elevatorLeftMotor.stopMotor();
    }

    public void stopArm() {
        armMotor.stopMotor();
    }

    // -- Periodic --

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(elevatorPosition, elevatorVelocity, elevatorCurrent, armPosition, armVelocity, armCurrent);
    }

    // -- Overrides --


    @Override
    public void onTeleopInit() {
        setElevatorHeight(ELEVATOR_LOW_POSE);
        setArmPosition(0.25);
        setTargetState(TrajectoryState.STOW);
    }

    @Override
    public void onDisabledInit() {
        stopElevator();
        stopArm();
    }

    @Override
    public List<SysIdTests.Test> getSysIdTests() {
        return List.of(
            // Elevator test
            SysIdTests.characterize(
                "Elevator", this, elevatorLeftMotor,
                Volts.of(0.25).per(Second), Volts.of(1),
                elevatorRightMotor, true
            ),
            // Arm test
            SysIdTests.characterize(
                "Arm", this, armMotor,
                Volts.of(0.25).per(Second), Volts.of(1)
            )
        );
    }
}
