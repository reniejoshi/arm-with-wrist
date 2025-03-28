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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.File;

public class RobotConfiguration {
    // -- Staging Feature Flags --

    private static final DigitalInput isClimberEnabled = new DigitalInput(RobotMap.CLIMBER_ENABLED_JUMPER);

    public static boolean isClimberEnabled() {
        return isClimberEnabled.get();
    }

    public static final boolean FEATURE_CORAL_DETECTION = true;
    public static final boolean AEE_FEATURE = false;

    // -- Deploy Directory --
    public static final File DEPLOY_DIR = Filesystem.getDeployDirectory();

    // -- Devices --
    public static final String CANBUS_NAME = "CANivore";

    // -- Phoenix Pro Toggles --
    public static final boolean CANIVORE_PHOENIX_PRO = true;
    public static final boolean WINDMILL_PHOENIX_PRO = true;
    public static final boolean RIO_PHOENIX_PRO = false;

    // -- Update Frequencies --
    public static final double ODOMETRY_UPDATE_FREQUENCY = 250;
    public static final double MECHANISM_UPDATE_FREQUENCY = 100;
}
