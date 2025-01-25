package org.tahomarobotics.robot.vision;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import org.photonvision.PhotonCamera;
import org.photonvision.estimation.TargetModel;
import org.photonvision.estimation.VisionEstimation;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.PnpResult;
import org.tahomarobotics.robot.Robot;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tinylog.Logger;
import org.tinylog.TaggedLogger;

import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;

import static org.tahomarobotics.robot.vision.VisionConstants.*;

// TODO: Put a ChArUco board in the repository.

/**
 * A PhotonVision camera configured to handle AprilTags and
 * estimate the robot's position with CasADi. A benefit to CasADi over MultiTag and
 * unconstrained SolvePNP is that when it's wrong, it's <strong>really</strong> wrong,
 * making it easier to filter out.
 *
 * <p><strong>PhotonVision Configuration</strong><p>
 * <p>
 * The PhotonVision camera instance must correspond the supplied naming in the configuration
 * and be as exact to the transform as possible. Furthermore, the camera should be properly
 * calibrated (either importing a prior calibration from the same camera model or calibrating
 * using a ChArUco board) and be using an 3D AprilTag pipeline (All estimation happens on the RoboRIO and
 * only relies on targets' corner positions).
 *
 * <p><strong>AprilTag Layout</strong><p>
 * <p>
 * Constrained estimation with CasADi requires pitch, height, and yaw (toggleable) to be fixed and any errors
 * with them will result in extremely inaccurate estimations. As such, the
 * AprilTag layout in {@link VisionConstants} and the ones uploaded to the PhotonVision instances should correspond
 * to the physical placement as accurately as possible (which will most likely not be perfect). Prior to matches,
 * AprilTag placement should be evaluated using <a href="https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/wpical/index.html"
 * >WPICal</a> with the resulting layout placed at <code>src/main/deploy</code> and uploaded to PhotonVision.
 *
 * <p><strong>Tuning</strong><p>
 * <p>
 * There are a lot of values that can be tuned to increase accuracy of estimations, but most often, scaling standard
 * deviations will be enough to get an accurate estimate. To do this,
 * <ol>
 *      <li>Place the robot a known distance away from an individual AprilTag (you may isolate a single target in code)</li>
 *      <li>Record the distance to the tag from the center of the robot and the error from the estimation</li>
 *      <li>Repeat for several distances</li>
 *      <li>Curve fit the data to get a scaling function for our standard deviations</li>
 *      <li>If necessary, repeat for scenarios with multiple AprilTags or use a scaled version of the single-tag function,
 *    but more likely than not estimations with multiple AprilTags will be extremely accurate
 *    (if they are not then standard deviations can't fix them).</li>
 * </ol>
 */
@Logged(strategy = Logged.Strategy.OPT_IN)
public class AprilTagCamera implements AutoCloseable {
    private final TaggedLogger logger;

    // Camera

    @Logged
    protected final CameraConfiguration configuration;
    protected final PhotonCamera camera;
    protected final PhotonCameraSim sim;

    private final Matrix<N3, N3> cameraMatrix;
    private final Matrix<N8, N1> distortionCoefficients;

    private final Consumer<EstimatedRobotPose> callback;

    private final Notifier notifier;

    // Diagnostics

    @Logged
    private Pose2d multiTagPose = new Pose2d(), singleTagPose = new Pose2d();

    @Logged(importance = Logged.Importance.DEBUG)
    private int failedUpdates;
    @Logged(importance = Logged.Importance.DEBUG)
    private int singleTagUpdates, multiTagUpdates;
    @Logged(importance = Logged.Importance.DEBUG)
    private double estimationTime, photonvisionLatency, processingTime;

    // Initialization

    /**
     * Creates a new {@link AprilTagCamera} with the supplied configuration and simulation properties.
     *
     * @param configuration Configuration about the camera (name, transform, etc.)
     * @param simProperties Simulation properties representing the real camera as closely as possible
     * @param callback      A callback to consume processed estimated robot poses
     */
    public AprilTagCamera(
        CameraConfiguration configuration,
        SimCameraProperties simProperties,
        Consumer<EstimatedRobotPose> callback
    ) {
        this.configuration = configuration;
        this.callback = callback;
        this.notifier = new Notifier(this::processUnreadVisionUpdates);

        logger = Logger.tag(configuration.name());

        camera = new PhotonCamera(configuration.name());
        sim = new PhotonCameraSim(camera, simProperties);

        if (camera.getCameraMatrix().isPresent()) {
            cameraMatrix = camera.getCameraMatrix().get();
        } else {
            logger.error("Could not retrieve camera matrix! Falling back to simulation properties.");
            cameraMatrix = simProperties.getIntrinsics(); // These should be identical.
        }

        if (camera.getDistCoeffs().isPresent()) {
            distortionCoefficients = camera.getDistCoeffs().get();
        } else {
            logger.error("Could not retrieve distortion coefficients! Falling back to simulation properties.");
            distortionCoefficients = simProperties.getDistCoeffs(); // ^
        }

        notifier.startPeriodic(Robot.kDefaultPeriod);
    }

    // Processing

    /**
     * Processes all unread valid vision updates into estimated robot poses.
     */
    public void processUnreadVisionUpdates() {
        camera.getAllUnreadResults()
              .forEach(result -> {
                  var est = processUpdate(result);
                  est.ifPresent(callback);
              });
    }

    /**
     * Processes a {@link PhotonPipelineResult} into an estimated robot pose using a constrained SolvePNP
     * algorithm with CasADi.
     *
     * @param result The pipeline result
     *
     * @return An estimated pose, if possible
     */
    private Optional<EstimatedRobotPose> processUpdate(PhotonPipelineResult result) {
        double timestamp = result.getTimestampSeconds();
        double now = Timer.getFPGATimestamp();

        // Pre-filtering

        List<PhotonTrackedTarget> targets = result.getTargets().stream().filter(
            t -> t.fiducialId > 0 && t.fiducialId < FIELD_LAYOUT.getTags().size()).toList();

        if (targets.isEmpty()) {
            return Optional.empty();
        }

        // CasADi pose estimation

        Pose3d robotPose3d;
        try {
            robotPose3d = estimateCasADiPose(
                timestamp,
                targets
            );
        } catch (Exception e) {
            failedUpdates++;

            logger.error(e.getMessage());
            return Optional.empty();
        }

        // Filter invalid estimates

        if (!isInField(robotPose3d)) {
            failedUpdates++;
            return Optional.empty();
        }

        // Classify the estimation with corresponding standard deviations

        Pose2d robotPose = robotPose3d.toPose2d();
        double distance = result.getBestTarget().bestCameraToTarget.getTranslation().getDistance(new Translation3d());

        EstimatedRobotPose.Type type;
        Vector<N3> stdDevs;
        if (targets.size() > 1) {
            type = EstimatedRobotPose.Type.MULTI_TAG;
            stdDevs = configuration.stdDevScaling()
                                   .scaleStandardDeviations(BASE_MULTI_TAG_STD_DEV, distance, targets.size());

            multiTagPose = robotPose;
            multiTagUpdates++;
        } else {
            type = EstimatedRobotPose.Type.SINGLE_TAG;
            stdDevs = configuration.stdDevScaling()
                                   .scaleStandardDeviations(BASE_SINGLE_TAG_STD_DEV, distance, targets.size());

            singleTagPose = robotPose;
            singleTagUpdates++;
        }

        // Diagnostics

        estimationTime = Timer.getFPGATimestamp() - now;
        photonvisionLatency = result.metadata.getLatencyMillis() / 1000;
        processingTime = Timer.getFPGATimestamp() - timestamp;

        // ---

        EstimatedRobotPose estimation =
            new EstimatedRobotPose(timestamp, type, robotPose, stdDevs, result.targets);

        return Optional.of(estimation);
    }

    /**
     * Estimates the robot position at a timestamp using the AprilTags in view.
     *
     * @param timestamp The timestamp of the update
     * @param targets   The targets the camera saw at the update
     *
     * @return An estimated robot pose
     *
     * @throws Exception If the estimation fails for any reason, an exception will be thrown with a more descriptive
     *                   error message. These are unrecoverable for this update, but not for the program, so they are
     *                   checked exceptions.
     */
    private Pose3d estimateCasADiPose(double timestamp, List<PhotonTrackedTarget> targets) throws Exception {
        // Get the chassis position at the timestamp
        Pose2d pose = Chassis.getInstance()
                             .getPoseAtTimestamp(timestamp)
                             .orElseThrow(() -> new Exception(
                                 "Could not retrieve robot pose at " + timestamp + "!"));

        // TODO: Do not attempt estimation if the robot has an pitch or roll or is otherwise violating constraints.

        // Estimate the position of the robot
        PnpResult casADiResult = VisionEstimation.estimateRobotPoseConstrainedSolvepnp(
            cameraMatrix,
            distortionCoefficients,
            targets,
            configuration.transform(),
            new Pose3d(pose),
            FIELD_LAYOUT,
            TargetModel.kAprilTag36h11,
            HEADING_FREE,
            pose.getRotation(),
            GYRO_ERROR_SCALING_FACTOR
        ).orElseThrow(() -> new Exception("Failed to estimate robot pose!"));

        // Add the resulting transform to the field origin
        return new Pose3d().plus(casADiResult.best);
    }

    // Helper Methods

    /**
     * Checks if the pose is within field bounds.
     *
     * @param pose The pose
     *
     * @return Whether the pose is in field bounds
     */
    private static boolean isInField(Pose3d pose) {
        return !(
            pose.getX() < 0 || pose.getX() > FIELD_LAYOUT.getFieldLength() ||
            pose.getY() < 0 || pose.getY() > FIELD_LAYOUT.getFieldWidth()
        );
    }

    // Getters

    /** Gets the name of this camera. */
    public String getName() {
        return configuration.name();
    }

    // Diagnostics

    /** Whether the camera is currently connected. */
    @Logged
    public boolean connected() {
        return camera.isConnected();
    }

    // Simulation

    /** Adds this camera to a vision simulation. */
    public void addCameraToSimulation(VisionSystemSim sim) {
        sim.addCamera(this.sim, configuration.transform());
    }

    // Processing Results

    /**
     * A timestamped estimated robot pose with associated trust values (standard deviations)
     * and the targets used for the estimation.
     */
    @Logged
    public static class EstimatedRobotPose {
        public final double timestamp;
        public final Type type;

        public final Pose2d pose;

        @NotLogged
        public final Vector<N3> stdDevs;
        @NotLogged
        public final List<PhotonTrackedTarget> targets;

        public EstimatedRobotPose(
            double timestamp, Type type, Pose2d pose, Vector<N3> stdDevs, List<PhotonTrackedTarget> targets
        ) {
            this.timestamp = timestamp;
            this.type = type;
            this.pose = pose;
            this.stdDevs = stdDevs;
            this.targets = targets;
        }

        @Logged
        public enum Type {
            MULTI_TAG, SINGLE_TAG
        }
    }

    // Auto Closeable

    @Override
    public void close() {
        sim.close();
        camera.close();
    }
}
