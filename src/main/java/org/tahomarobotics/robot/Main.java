package org.tahomarobotics.robot;

import edu.wpi.first.wpilibj.RobotBase;
import org.tahomarobotics.robot.whoami.Identity;
import org.tinylog.Logger;

public final class Main {
    private Main() {}

    public static void main(String... args) {
        int idx = 0;
        if (args.length == 0) {
            Logger.warn("No identity index passed in! Defaulting to {}", Identity.values()[0]);
        } else {
            try {
                idx = Integer.parseInt(args[0]);
            } catch (NumberFormatException e) {
                Logger.warn("Invalid identity index passed in! Defaulting to {}", Identity.values()[0]);
            }
        }

        Identity identity = Identity.values()[idx];
        Logger.info("Identity: {}", identity);

        RobotBase.startRobot(Robot::new);
    }
}
