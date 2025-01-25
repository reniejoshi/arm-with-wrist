package org.tahomarobotics.robot.util.swerve;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * A modified version of {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator} that allows getting
 * the raw odometry pose in addition to the vision compensated pose.
 */
public class SwerveDrivePoseEstimatorDiff extends PoseEstimator<SwerveModulePosition[]> {
    private final int numModules;
    private final SwerveDriveOdometry odometry;

    /**
     * Constructs a SwerveDrivePoseEstimatorDiff.
     *
     * @param kinematics   A correctly-configured kinematics object for your drivetrain.
     * @param odometry     A correctly-configured odometry object for your drivetrain.
     * @param stateStdDevs Standard deviations of the pose estimate (x position in meters, y position
     *                     in meters, and heading in radians). Increase these numbers to trust your state estimate
     *                     less.
     */
    public SwerveDrivePoseEstimatorDiff(
        SwerveDriveKinematics kinematics,
        SwerveDriveOdometry odometry,
        Matrix<N3, N1> stateStdDevs) {
        super(
            kinematics,
            odometry,
            stateStdDevs,
            VecBuilder.fill(1, 1, 1));

        this.numModules = kinematics.getModules().length;
        this.odometry = odometry;
    }

    // Getters

    /**
     * Gets the raw pose of the pose estimator without the influence of vision measurements.
     *
     * @return The raw pose
     */
    public Pose2d getRawPose() {
        return odometry.getPoseMeters();
    }

    // Overrides

    @Override
    public Pose2d updateWithTime(double currentTimeSeconds, Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions) {
        assert wheelPositions.length == numModules;
        return super.updateWithTime(currentTimeSeconds, gyroAngle, wheelPositions);
    }
}
