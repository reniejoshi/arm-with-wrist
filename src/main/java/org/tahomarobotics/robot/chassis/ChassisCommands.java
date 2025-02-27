package org.tahomarobotics.robot.chassis;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.function.DoubleSupplier;

public class ChassisCommands {
    public static Command createAlignSwerveCommand(Chassis chassis) {
        String FINALIZE_KEY = "Finalize";

        Command cmd = (
            Commands.waitUntil(() -> SmartDashboard.getBoolean(FINALIZE_KEY, false))
                    .beforeStarting(() -> {
                        SmartDashboard.putBoolean(FINALIZE_KEY, false);
                        chassis.initializeCalibration();
                    })
                    .finallyDo(interrupted -> {
                        if (interrupted) {
                            chassis.cancelCalibration();
                        } else {
                            chassis.finalizeCalibration();
                        }
                    })
        ).onlyWhile(RobotState::isDisabled).ignoringDisable(true);
        cmd.addRequirements(chassis);

        return cmd;
    }

    public static Command createTeleOpDriveCommand(
        Chassis chassis,
        DoubleSupplier x, DoubleSupplier y, DoubleSupplier omega
    ) {
        double maxVelocity = ChassisConstants.MAX_VELOCITY;
        double maxAngularVelocity = ChassisConstants.MAX_ANGULAR_VELOCITY;

        return chassis.runEnd(
            () -> {
                double direction = DriverStation.getAlliance().orElse(null) == DriverStation.Alliance.Red ? -1.0 : 1.0;

                chassis.drive(new ChassisSpeeds(
                    x.getAsDouble() * maxVelocity * direction,
                    y.getAsDouble() * maxVelocity * direction,
                    omega.getAsDouble() * maxAngularVelocity
                ));
            },
            () -> chassis.drive(new ChassisSpeeds())
        );
    }
}
