package org.tahomarobotics.robot.indexer;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IndexerConstants {
    // Motion Magic Constraints

    private static final double MAX_VELOCITY = 22;
    private static final double MAX_ACCELERATION = MAX_VELOCITY * 2;
    private static final double MAX_JERK = MAX_ACCELERATION * 4;

    // Current Limits

    // Tolerances

    public static final double POSITION_THRESHOLD = 0.01;

    // States

    private static final double SERIALIZATION_DISTANCE = 4;

    public enum IndexerState {
        COLLECTED(MotionType.NONE, 0),
        DISABLED(MotionType.NONE, 0),
        INDEXING(MotionType.VELOCITY, MAX_VELOCITY),
        EJECTING(MotionType.VELOCITY, -MAX_VELOCITY),
        // TODO: Trigger with a beam break once installed;
        //       Skip for now (pass to arm directly). Additionally,
        //       serialization might not be feasible due to the
        //       velocity of the coral.
        SERIALIZING(MotionType.POSITION, SERIALIZATION_DISTANCE);

        public final MotionType type;
        public final double value;

        IndexerState(MotionType type, double value) {
            this.type = type;
            this.value = value;
        }

        public enum MotionType {
            POSITION, VELOCITY, NONE
        }
    }

    // Configuration

    public static final TalonFXConfiguration configuration = new TalonFXConfiguration()
        .withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
        ).withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(MAX_VELOCITY)
                .withMotionMagicAcceleration(MAX_ACCELERATION)
                .withMotionMagicJerk(MAX_JERK)
        ).withSlot0(
            new Slot0Configs()
                .withKP(0.083374)
                .withKS(0.17818)
                .withKV(0.12464)
                .withKA(0.0039997)
        ).withSlot1(
            new Slot1Configs()
                .withKP(15.944)
                .withKD(0.27961)
                .withKS(0.17818)
                .withKV(0.12464)
                .withKA(0.0039997)
        ).withAudio(
            new AudioConfigs()
                .withBeepOnBoot(true)
                .withBeepOnConfig(true)
        );

}
