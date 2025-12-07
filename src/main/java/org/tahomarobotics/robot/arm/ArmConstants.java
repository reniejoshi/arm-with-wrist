package org.tahomarobotics.robot.arm;

import edu.wpi.first.units.measure.Angle;

import static edu.wpi.first.units.Units.Degrees;

public class ArmConstants {
    // Arm limits
    public static final double ARM_MIN_POSITION = 0;
    public static final double ARM_MAX_POSITION = 180;

    // Wrist limits
    public static final double WRIST_MIN_POSITION = 0;
    public static final double WRIST_MAX_POSITION = 300;

    // Increment values
    public static final double ARM_INCREMENT = 1;
    public static final double WRIST_INCREMENT = 1;
}