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

package org.tahomarobotics.robot;

import edu.wpi.first.math.geometry.Translation2d;

import static org.tahomarobotics.robot.chassis.ChassisConstants.*;

public final class RobotMap {
    public final static int PIGEON = 0;

    public final static int BEAM_BREAK = 0;

    public final static int CLIMBER_ENABLED_JUMPER = 2;
    public final static SwerveModuleDescriptor FRONT_LEFT_MOD = new SwerveModuleDescriptor(
        "Front Left", FRONT_LEFT_OFFSET, 1, 11, 21);
    public final static SwerveModuleDescriptor FRONT_RIGHT_MOD = new SwerveModuleDescriptor(
        "Front Right", FRONT_RIGHT_OFFSET, 2, 12, 22);
    public final static SwerveModuleDescriptor BACK_LEFT_MOD = new SwerveModuleDescriptor(
        "Back Left", BACK_LEFT_OFFSET, 3, 13, 23);
    public final static SwerveModuleDescriptor BACK_RIGHT_MOD = new SwerveModuleDescriptor(
        "Back Right", BACK_RIGHT_OFFSET, 4, 14, 24);

    public static final int ELEVATOR_LEFT_MOTOR = 5;
    public static final int ELEVATOR_RIGHT_MOTOR = 6;
    public static final int ELEVATOR_ENCODER = 7;

    public static final int COLLECTOR_LEFT_MOTOR = 8;
    public static final int COLLECTOR_RIGHT_MOTOR = 9;
    public static final int COLLECTOR_COLLECT_MOTOR = 10;

    public static final int ARM_MOTOR = 15;
    public static final int ARM_ENCODER = 16;

    public static final int END_EFFECTOR_MOTOR = 17;
    public static final int INDEXER_MOTOR = 18;

    public static final int CLIMBER_MOTOR = 19;
    public static final int CLIMBER_FOLLOWER = 20;
    public static final int CLIMBER_RATCHET_SOLENOID = 25;
    public static final int CLIMBER_ROLLER = 27;
    public static final int CLIMBER_LIMIT_SWITCH = 1;

    public static final int LED = 26;

    public record SwerveModuleDescriptor(String moduleName, Translation2d offset, int driveId, int steerId,
                                         int encoderId) {}
}
