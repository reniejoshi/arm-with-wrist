package org.tahomarobotics.robot.auto;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import org.tahomarobotics.robot.chassis.Chassis;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class GoToPoseCommand extends Command {
    private final Chassis chassis = Chassis.getInstance();

    // Constants

    private static final double SETTLE_TIME = 0.5;

    // State

    private double lastTimestamp = Timer.getFPGATimestamp();

    private final Pose2d targetPose;
    private final ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    // PID Controllers

    private final PIDController rotationController = new PIDController(10.0, 0.0, 0.5);
    private final PIDController xController = new PIDController(7.5, 0, 0.75);
    private final PIDController yController = new PIDController(7.5, 0, 0.75);

    private double rotationTimer, xTimer, yTimer;

    // Initialization

    public GoToPoseCommand(Pose2d targetPose) {
        this.targetPose = targetPose;

        rotationController.setTolerance(Math.PI / 90);
        xController.setTolerance(0.03);
        yController.setTolerance(0.03);

        addRequirements(chassis);
    }

    @Override
    public void initialize() {
        SmartDashboard.putData("Rotation Controller", rotationController);
        SmartDashboard.putData("X Controller", xController);
        SmartDashboard.putData("Y Controller", yController);

        rotationController.reset();
        xController.reset();
        yController.reset();
    }

    // Execute

    @Override
    public void execute() {
        Pose2d currentPose = chassis.getPose();

        double time = Timer.getFPGATimestamp();
        double dT = time - lastTimestamp;
        lastTimestamp = time;

        chassisSpeeds.vxMetersPerSecond = xController.calculate(currentPose.getX(), targetPose.getX());
        chassisSpeeds.vyMetersPerSecond = yController.calculate(currentPose.getY(), targetPose.getY());
        chassisSpeeds.omegaRadiansPerSecond = rotationController.calculate(
            chassis.getHeading().getRadians(),
            targetPose.getRotation().getRadians()
        );

        if (rotationController.atSetpoint()) {
            rotationTimer += dT;
        } else {
            rotationTimer = 0.0;
        }
        if (xController.atSetpoint()) {
            xTimer += dT;
        } else {
            xTimer = 0.0;
        }
        if (yController.atSetpoint()) {
            yTimer += dT;
        } else {
            yTimer = 0.0;
        }

        chassis.drive(chassisSpeeds, true);
    }

    @Override
    public boolean isFinished() {
        return xTimer >= SETTLE_TIME && yTimer >= SETTLE_TIME && rotationTimer >= SETTLE_TIME;
    }

    @Override
    public void end(boolean interrupted) {
        chassis.drive(new ChassisSpeeds());
    }
}
