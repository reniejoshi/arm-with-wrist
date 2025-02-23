package org.tahomarobotics.robot.auto;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.chassis.ChassisConstants;
import org.tahomarobotics.robot.collector.Collector;
import org.tahomarobotics.robot.collector.CollectorCommands;
import org.tahomarobotics.robot.grabber.Grabber;
import org.tahomarobotics.robot.grabber.GrabberCommands;
import org.tahomarobotics.robot.indexer.Indexer;
import org.tahomarobotics.robot.util.SubsystemIF;
import org.tahomarobotics.robot.windmill.Windmill;
import org.tahomarobotics.robot.windmill.WindmillConstants;
import org.tinylog.Logger;

import java.util.*;

public class Autonomous extends SubsystemIF {
    private static final Autonomous INSTANCE = new Autonomous();

    private final Chassis chassis = Chassis.getInstance();
    private final SendableChooser<Command> autoChooser;
    private String currentAutoName = AutoConstants.DEFAULT_AUTO_NAME;

    private Autonomous() {
        registerNamedCommands();

//        SendableChooser<Command> prevChooser = (SendableChooser<Command>) SmartDashboard.getData("Auto Chooser");
        autoChooser = PathPlannerHelper.getAutoChooser(chassis, this::onAutoChange);
//        autoChooser.setDefaultOption(prevChooser.getSelected().getName(), new PathPlannerAuto(prevChooser.getSelected().getName()));

        var inst = NetworkTableInstance.getDefault();
        inst.addListener(
            inst.getBooleanTopic("/FMSInfo/IsRedAlliance").getEntry(true),
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            e -> {
                var command = autoChooser.getSelected();
                if (command != null)
                    this.onAutoChange(command);
                else
                    this.onAutoChange(Commands.none().withName(AutoConstants.DEFAULT_AUTO_NAME));
            }
        );

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public static Autonomous getInstance() {
        return INSTANCE;
    }

    public Command getSelectedAuto() {
        return autoChooser.getSelected();
    }

    public String getSelectedAutoName() {
        return currentAutoName;
    }

    // NAMED COMMANDS

    private void registerNamedCommands() {
        Collector collector = Collector.getInstance();
        Grabber grabber = Grabber.getInstance();
        Windmill windmill = Windmill.getInstance();
        Indexer indexer = Indexer.getInstance();

        Pair<Command,Command> collectorControlCommands = CollectorCommands.createCollectorControlCommands(collector);
        NamedCommands.registerCommand("Toggle Collector Deploy and Collect",
                                      CollectorCommands.createDeploymentControlCommand(collector)
                                                       .andThen(Commands.either(collectorControlCommands.getFirst(),
                                                                                collectorControlCommands.getSecond(),
                                                                                collector::isDeploymentStowed)));

        NamedCommands.registerCommand("Calibrate Windmill", Commands.runOnce(windmill::calibrate)
                                                                    .andThen(Commands.waitUntil(windmill::isZeroed))
                                                                    .andThen(Commands.runOnce(() -> {
                                                                        windmill.setTargetState(WindmillConstants.TrajectoryState.START);
                                                                    })));

        NamedCommands.registerCommand("Windmill to Collect", windmill.createTransitionCommand(WindmillConstants.TrajectoryState.COLLECT).andThen(
            GrabberCommands.createGrabberCommands(grabber).getFirst())
                                                                     .andThen(Commands.waitUntil(grabber::isHolding)));
        Pair<Command, Command> grabberScoringCommands = GrabberCommands.createGrabberScoringCommands(grabber);
        NamedCommands.registerCommand("Grabber Score", grabberScoringCommands.getFirst()
                                                                             .andThen(Commands.waitSeconds(0.25))
                                                                             .andThen(grabberScoringCommands.getSecond()));
        NamedCommands.registerCommand("Windmill to L4",  Commands.runOnce(() -> Logger.info("moving windmill"))
                                                                 .andThen(windmill.createTransitionCommand(WindmillConstants.TrajectoryState.L4)));
        NamedCommands.registerCommand("Windmill to L3", windmill.createTransitionCommand(WindmillConstants.TrajectoryState.L3));
        NamedCommands.registerCommand("Windmill to L2", windmill.createTransitionCommand(WindmillConstants.TrajectoryState.L2));
    }

    // PATH VISUALIZATION

    private static List<PathPlannerTrajectory> getTrajectories(List<PathPlannerPath> autoPaths) {
        List<PathPlannerTrajectory> autoTrajectories = new ArrayList<>();

        ChassisSpeeds lastChassisSpeeds = new ChassisSpeeds();
        Rotation2d lastRotation = new Rotation2d();

        for (PathPlannerPath path : autoPaths) {
            if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)) {
                path = path.flipPath();
            }

            PathPlannerTrajectory pathTrajectory = path.generateTrajectory(lastChassisSpeeds, lastRotation, ChassisConstants.robotConfig);
            autoTrajectories.add(pathTrajectory);

            PathPlannerTrajectoryState endState = pathTrajectory.getEndState();

            lastChassisSpeeds = endState.fieldSpeeds;
            lastRotation = endState.heading;
        }

        return autoTrajectories;
    }

    /*
     * Converts a list of PathPlannerPaths to a Trajectory for display on the field.
     * These trajectories are not functional because they do not have acceleration or curvature.
     */
    private static Trajectory convertToTrajectoryForDisplay(List<PathPlannerPath> selectedAutoPaths) {
        return new Trajectory(
            getTrajectories(selectedAutoPaths).stream().flatMap(
                trajectory ->
                    trajectory
                        .getStates().stream().map
                            (state -> new Trajectory.State(
                                state.timeSeconds,
                                state.linearVelocity,
                                0,
                                state.pose, // This will be slightly inaccurate.
                                0)
                            )
            ).toList());
    }

    private void postAutoTrajectory(Field2d field, String autoName) {
        if (autoName == null || autoName.equals(AutoConstants.DEFAULT_AUTO_NAME)) {
            chassis.resetOdometry(new Pose2d());
            field.getObject("Trajectory").setTrajectory(new Trajectory());
            return;
        }


        try {
            List<PathPlannerPath> autoPaths = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
            if (!autoPaths.isEmpty()) {
                PathPlannerPath firstPath = autoPaths.get(0);
                Optional<Pose2d> startingPose = firstPath.getStartingHolonomicPose();
                chassis.resetOdometry(startingPose.isPresent() ? startingPose.get() : new Pose2d());
            }
            field.getObject("Trajectory").setTrajectory(convertToTrajectoryForDisplay(autoPaths));
        } catch (Exception e) {
            System.out.println("Error loading auto path: " + autoName);
        }
    }

    private void onAutoChange(Command auto) {
        new InstantCommand(() ->
                               postAutoTrajectory(chassis.getField(), auto.getName())).schedule();
    }

    @Override
    public double getEnergyUsed() {
        return 0;
    }

    @Override
    public double getTotalCurrent() {
        return 0;
    }
}