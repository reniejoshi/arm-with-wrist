package org.tahomarobotics.robot.vision;

import edu.wpi.first.epilogue.Logged;
import org.photonvision.simulation.VisionSystemSim;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.util.SubsystemIF;

import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.stream.Collectors;
import java.util.stream.Stream;

/**
 * A subsystem to consolidate vision updates from multiple AprilTag cameras. Currently, only PhotonVision cameras
 * are supported but if the Limelight 4 is evaluated to be significantly better, then support will be added.
 */
@Logged(strategy = Logged.Strategy.OPT_IN)
public class Vision extends SubsystemIF implements AutoCloseable {
    private static final Vision INSTANCE = new Vision();

    // State

    private final Consumer<AprilTagCamera.EstimatedRobotPose> estimationCallback =
        Chassis.getInstance()::processVisionUpdate;

    // Cameras

    @Logged
    private final AprilTagCamera elevatorSwerve =
        new AprilTagCamera(VisionConstants.ELEVATOR_SWERVE, VisionConstants.simOV9782Properties,
                           estimationCallback);

    @Logged
    private final AprilTagCamera climberSwerve =
        new AprilTagCamera(VisionConstants.CLIMBER_SWERVE, VisionConstants.simOV9782Properties,
                           estimationCallback);

    private final Map<String, AprilTagCamera> aprilTagCameras =
        Stream.of(elevatorSwerve, climberSwerve)
              .collect(Collectors.toMap(
                  AprilTagCamera::getName,
                  Function.identity()
             ));

    // Diagnostic

    // TODO: Store necessary Subsystem-wide diagnostic values here (updates, etc.).

    // Initialization

    private Vision() {}

    public static Vision getInstance() {
        return INSTANCE;
    }

    // Simulation

    VisionSystemSim visionSim = new VisionSystemSim("main");

    @Override
    public void onSimulationInit() {
        visionSim.addAprilTags(VisionConstants.FIELD_LAYOUT);
        aprilTagCameras.values().forEach(camera -> camera.addCameraToSimulation(visionSim));
    }

    @Override
    public void simulationPeriodic() {
        visionSim.update(Chassis.getInstance().getPose());
    }

    // Auto Closeable

    @Override
    public void close() {
        aprilTagCameras.values().forEach(AprilTagCamera::close);
    }
}
