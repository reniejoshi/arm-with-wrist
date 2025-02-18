package org.tahomarobotics.robot.grabber;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class GrabberConstants {
    public static final double COLLECT_VELOCITY = -10;
    public static final double EJECT_VELOCITY = 15;
    public static final double HOLD_VOLTAGE = -0.25;

    public static final double HOLDING_CURRENT_THRESHOLD = 20;

    public static final double GEAR_REDUCTION = (10d / 26d);

    // -- States --

    public enum GrabberState {
        DISABLED(MotionType.NONE, 0),
        HOLDING(MotionType.VOLTAGE, HOLD_VOLTAGE),
        COLLECTING(MotionType.VELOCITY, COLLECT_VELOCITY),
        EJECTING(MotionType.VELOCITY, EJECT_VELOCITY),
        SCORING(MotionType.VELOCITY, EJECT_VELOCITY);

        public final MotionType type;
        public final double value;

        GrabberState(MotionType type, double value) {
            this.type = type;
            this.value = value;
        }

        public enum MotionType {
            VELOCITY, VOLTAGE, NONE
        }
    }

    // -- Configuration --

    public static final TalonFXConfiguration motorConfig = new TalonFXConfiguration()
        // SysId'd 02/11
        .withSlot0(new Slot0Configs()
                       .withKP(0.5757)
                       .withKV(0.41844)
                       .withKA(0.8071))
        .withMotorOutput(new MotorOutputConfigs()
                             .withNeutralMode(NeutralModeValue.Brake)
                             .withInverted(InvertedValue.CounterClockwise_Positive))
        .withMotionMagic(new MotionMagicConfigs()
                             .withMotionMagicCruiseVelocity(5)
                             .withMotionMagicAcceleration(15)
                             .withMotionMagicJerk(50))
        .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1 / GEAR_REDUCTION))
        .withAudio(new AudioConfigs().withBeepOnBoot(true).withBeepOnConfig(true));
}
