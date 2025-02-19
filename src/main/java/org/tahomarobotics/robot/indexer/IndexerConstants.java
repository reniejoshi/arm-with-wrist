package org.tahomarobotics.robot.indexer;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IndexerConstants {
    // Motion Magic Constraints

    private static final double MAX_VELOCITY = 16; // Rotations per second
    private static final double MAX_ACCELERATION = MAX_VELOCITY * 4;
    private static final double MAX_JERK = MAX_ACCELERATION * 4;

    // States

    public enum IndexerState {
        DISABLED(MotionType.NONE, 0),
        COLLECTED(MotionType.NONE, 0),
        HOLDING(MotionType.NONE, 0),
        COLLECTING(MotionType.VELOCITY, MAX_VELOCITY),
        EJECTING(MotionType.VELOCITY, -MAX_VELOCITY),
        PASSING(MotionType.VELOCITY, MAX_VELOCITY);

        public final MotionType type;
        public final double value;

        IndexerState(MotionType type, double value) {
            this.type = type;
            this.value = value;
        }

        public enum MotionType {
            VELOCITY, NONE
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
            new Slot0Configs() //Velocity Constants
                .withKP(0.083374)
                .withKS(0.17818)
                .withKV(0.12464)
                .withKA(0.0039997)
        ).withSlot1(
            new Slot1Configs() //Position Constants
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
