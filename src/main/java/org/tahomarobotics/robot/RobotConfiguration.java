package org.tahomarobotics.robot;

import edu.wpi.first.wpilibj.Filesystem;

import java.io.File;

public class RobotConfiguration {
    public static final File DEPLOY_DIR = Filesystem.getDeployDirectory();

    public static final String CANBUS_NAME = "CANivore";
    public static final boolean CANIVORE_PHOENIX_PRO = false;
    public static final boolean RIO_PHOENIX_PRO = false;
    public static final double ODOMETRY_UPDATE_FREQUENCY = 250;
    public static final double MECHANISM_UPDATE_FREQUENCY = 100;
}
