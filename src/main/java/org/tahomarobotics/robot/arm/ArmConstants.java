package org.tahomarobotics.robot.arm;

import edu.wpi.first.math.util.Units;

public class ArmConstants {
    // Arm limits
    public static final double ARM_MIN_POSITION = Units.degreesToRadians(0);
    public static final double ARM_MAX_POSITION = Units.degreesToRadians(180);

    // Wrist limits
    public static final double WRIST_MIN_POSITION = Units.degreesToRadians(0);
    public static final double WRIST_MAX_POSITION = Units.degreesToRadians(300);

    // Increment values
    public static final double ARM_INCREMENT = 1;
    public static final double WRIST_INCREMENT = 1;
}