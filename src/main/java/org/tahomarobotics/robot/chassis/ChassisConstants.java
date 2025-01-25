package org.tahomarobotics.robot.chassis;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import org.tahomarobotics.robot.RobotConfiguration;

@SuppressWarnings("SuspiciousNameCombination")
public class ChassisConstants {
    // Physical

    /** Width-wise distance between wheel centers in <strong>meters</strong> */
    private static final double TRACK_WIDTH = 0.5816;
    /** Length-wise distance between wheel centers in <strong>meters</strong> */
    private static final double WHEELBASE = 0.5816;

    private static final double HALF_TRACK_WIDTH = TRACK_WIDTH / 2;
    private static final double HALF_WHEELBASE = WHEELBASE / 2;

    /** Approximate mass of the robot in <strong>kilograms</strong> */
    public static final double MASS = 52.1631;

    /** Approximate radius of the wheel in <strong>meters</strong> */
    private static final double WHEEL_RADIUS = 0.0508;
    /** Approximate circumference of the wheel in <strong>meters</strong> */
    private static final double WHEEL_CIRCUMFERENCE = 2 * Math.PI * WHEEL_RADIUS;
    /** Approximate Moment of Inertia based on CAD in <strong>kilogram meter squared</strong>. */
    // https://www.swervedrivespecialties.com/collections/mk4i-parts/products/billet-wheel-4d-x-1-5w-bearing-bore
    public static final double WHEEL_MOI = 0.00031307;

    public static final Translation2d FRONT_LEFT_OFFSET = new Translation2d(HALF_WHEELBASE, HALF_TRACK_WIDTH);
    public static final Translation2d FRONT_RIGHT_OFFSET = new Translation2d(HALF_WHEELBASE, -HALF_TRACK_WIDTH);
    public static final Translation2d BACK_LEFT_OFFSET = new Translation2d(-HALF_WHEELBASE, HALF_TRACK_WIDTH);
    public static final Translation2d BACK_RIGHT_OFFSET = new Translation2d(-HALF_WHEELBASE, -HALF_TRACK_WIDTH);

    // Gearing

    /** Reduction between the drive motor and the wheel. */
    public static final double DRIVE_REDUCTION = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
    /** Reduction between the steer motor and the wheel. */
    public static final double STEER_REDUCTION = (14.0 / 50.0) * (10.0 / 60.0);
    /** Ratio between angular position and position in <strong>meters</strong> for the wheel. */
    public static final double DRIVE_POSITION_COEFFICIENT = WHEEL_CIRCUMFERENCE * DRIVE_REDUCTION;

    /**
     * Load inertia for each drive motor in <strong>kilogram meter squared</strong>.
     * <p>
     * {@code J_load = M / N * (r / G)^2} where M is the mass of the robot,
     * N is the number of modules, r is the radius of the wheel, and G is the gear ratio.
     */
    public static final double DRIVE_MOTOR_MOI = MASS / 4 * Math.pow(WHEEL_RADIUS * DRIVE_REDUCTION, 2);

    // Simulation

    /** A semi-arbitrary scaling factor for the motor model MOIs. */
    public static final double MOI_SCALING_FACTOR = 10;

    // Motion

    public static final DCMotor DRIVE_MOTOR = DCMotor.getKrakenX60Foc(1);
    /** Theoretical max velocity of the robot in <strong>meters per second</strong>. */
    public static final double MAX_VELOCITY = DRIVE_MOTOR.freeSpeedRadPerSec * DRIVE_REDUCTION * WHEEL_RADIUS;
    /** Theoretical max angular velocity of the robot in <strong>meters per second</strong>. */
    public static final double MAX_ANGULAR_VELOCITY = MAX_VELOCITY / Math.hypot(HALF_TRACK_WIDTH, HALF_WHEELBASE);
    /** Max average acceleration across all modules in <strong>meters per second squared</strong>. */
    public static final double ACCELERATION_LIMIT = 6.0;

    // Control Loops

    @SuppressWarnings("HungarianNotationConstants")
    public static final double kV_DRIVE = (2 * Math.PI) / DRIVE_MOTOR.KvRadPerSecPerVolt;

    // Current Limits

    private static final double DRIVE_SUPPLY_CURRENT_LIMIT = 50;
    private static final double DRIVE_STATOR_CURRENT_LIMIT = 100;
    private static final double STEER_SUPPLY_CURRENT_LIMIT = 20;
    private static final double STEER_STATOR_CURRENT_LIMIT = 30;

    // Configuration

    public static final TalonFXConfiguration driveMotorConfiguration = new TalonFXConfiguration()
        .withSlot0(new Slot0Configs()
                       .withKP(0.15)
                       .withKV(kV_DRIVE))
        .withMotorOutput(new MotorOutputConfigs()
                             .withNeutralMode(NeutralModeValue.Brake)
                             .withInverted(InvertedValue.Clockwise_Positive))
        .withMotionMagic(new MotionMagicConfigs()
                             .withMotionMagicAcceleration(120)
                             .withMotionMagicJerk(360)
        )
        .withCurrentLimits(new CurrentLimitsConfigs()
                               .withStatorCurrentLimit(DRIVE_STATOR_CURRENT_LIMIT)
                               .withSupplyCurrentLimit(DRIVE_SUPPLY_CURRENT_LIMIT))
        .withAudio(new AudioConfigs()
                       .withBeepOnBoot(true)
                       .withBeepOnConfig(true));

    public static final TalonFXConfiguration steerMotorConfiguration = new TalonFXConfiguration()
        .withSlot0(new Slot0Configs()
                       .withKP(8.0)
                       .withKI(0.01)
                       .withKD(0.16)
        )
        .withMotorOutput(new MotorOutputConfigs()
                             .withNeutralMode(NeutralModeValue.Brake)
                             .withInverted(InvertedValue.Clockwise_Positive))
        .withMotionMagic(new MotionMagicConfigs()
                             .withMotionMagicAcceleration(25)
                             .withMotionMagicJerk(100)
        )
        .withClosedLoopGeneral(new ClosedLoopGeneralConfigs() {{
            ContinuousWrap = true;
        }})
        .withFeedback(new FeedbackConfigs() {{
                          if (RobotConfiguration.CANIVORE_PHOENIX_PRO) {
                              FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
                              RotorToSensorRatio = 1 / STEER_REDUCTION;
                          } else {
                              FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
                          }
                      }}
        )
        .withCurrentLimits(new CurrentLimitsConfigs()
                               .withStatorCurrentLimit(STEER_STATOR_CURRENT_LIMIT)
                               .withSupplyCurrentLimit(STEER_SUPPLY_CURRENT_LIMIT))
        .withAudio(new AudioConfigs()
                       .withBeepOnBoot(true)
                       .withBeepOnConfig(true));

    public static final CANcoderConfiguration encoderConfiguration = new CANcoderConfiguration()
        .withMagnetSensor(new MagnetSensorConfigs()
                              .withAbsoluteSensorDiscontinuityPoint(1)
                              .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
        );
}
