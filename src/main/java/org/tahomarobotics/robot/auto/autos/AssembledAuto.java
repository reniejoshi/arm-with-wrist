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

package org.tahomarobotics.robot.auto.autos;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.tahomarobotics.robot.RobotConfiguration;
import org.tahomarobotics.robot.auto.AutonomousConstants;
import org.tahomarobotics.robot.auto.commands.DriveToPoseV4Command;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.collector.Collector;
import org.tahomarobotics.robot.collector.CollectorCommands;
import org.tahomarobotics.robot.grabber.Grabber;
import org.tahomarobotics.robot.indexer.Indexer;
import org.tahomarobotics.robot.windmill.Windmill;
import org.tahomarobotics.robot.windmill.commands.WindmillMoveCommand;
import org.tinylog.Logger;

import java.util.LinkedHashMap;
import java.util.Set;
import java.util.function.DoubleSupplier;

import static org.tahomarobotics.robot.windmill.WindmillConstants.TrajectoryState.*;

public class AssembledAuto extends SequentialCommandGroup {
    // -- Constants --

    private static final double SCORING_DISTANCE = Units.inchesToMeters(3);
    private static final double ARM_UP_DISTANCE = AutonomousConstants.APPROACH_DISTANCE_BLEND_FACTOR + Units.inchesToMeters(6);
    private static final double ARM_DOWN_DISTANCE = Units.inchesToMeters(24);

    private static final double SCORING_TIME = 0.25;

    // -- Requirements --

    private final Chassis chassis = Chassis.getInstance();
    private final Collector collector = Collector.getInstance();
    private final Grabber grabber = Grabber.getInstance();
    private final Indexer indexer = Indexer.getInstance();
    private final Windmill windmill = Windmill.getInstance();

    // -- Determinants --

    private final DriverStation.Alliance alliance;

    // -- Initialization --

    public AssembledAuto(boolean isLeft, LinkedHashMap<Character, DoubleSupplier> scorePositions, DriverStation.Alliance alliance, String name) {
        setName(name + " " + (isLeft ? "Left" : "Right"));

        this.alliance = alliance;

        Timer timer = new Timer();
        addCommands(
            Commands.runOnce(timer::restart)
        );

        if (!isLeft) {
            Character[] keys = scorePositions.keySet().toArray(new Character[0]);
            DoubleSupplier[] fudgeSuppliers = scorePositions.values().toArray(new DoubleSupplier[0]);
            scorePositions.clear();

            // Flip characters to the right side
            for (int i = 0; i < keys.length; i++) {
                if (keys[i] == 'A') {
                    keys[i] = 'B';
                } else {
                    keys[i] = (char) ('C' + ('L' - keys[i]));
                }
                scorePositions.put(keys[i], fudgeSuppliers[i]);
            }
        }

        // Add first score position (this uses a different command than the rest).
        addCommands(driveToFirstPoleThenScore(scorePositions.keySet().toArray(new Character[0])[0], scorePositions.get(scorePositions.keySet().toArray()[0])));
        
        boolean firstKey = true;
        for (Character character : scorePositions.keySet()) {
            if (!firstKey) {
                addCommands(
                    driveToCoralAndCollect(isLeft),
                    driveToPoleThenScoreWhileCollecting(character, scorePositions.get(character))
                );
            }
            firstKey = false;
        }

        addCommands(
            // Reset the robot upon finishing
            Commands.parallel(
                collector.runOnce(() -> {
                    collector.collectorTransitionToDisabled();
                    collector.deploymentTransitionToStow();
                }),
                indexer.runOnce(indexer::transitionToDisabled),
                Commands.waitSeconds(0.25).andThen(grabber.runOnce(grabber::transitionToDisabled))
            ),
            Commands.runOnce(() -> Logger.info("Five-Piece completed in {} seconds.", timer.get()))
        );
    }

    public Command driveToFirstPoleThenScore(char pole) {
        return driveToFirstPoleThenScore(pole, () -> 0);
    }

    public Command driveToFirstPoleThenScore(char pole, DoubleSupplier fudge) {
        // Drive to the scoring position
        DriveToPoseV4Command dtp = AutonomousConstants.getObjectiveForPole(pole - 'A', alliance).fudgeY(fudge.getAsDouble()).driveToPoseV4Command();

        // Move arm from STOW to L4
        Command stowToL4 = WindmillMoveCommand.fromTo(STOW, L4).orElseThrow();

        // Score grabber
        Command scoreGrabber = grabber.runOnce(grabber::transitionToScoring).andThen(Commands.waitSeconds(SCORING_TIME));

        Timer timer = new Timer();
        return Commands.parallel(
            Commands.runOnce(timer::restart),
            // Calibrate
            CollectorCommands.createZeroCommand(Collector.getInstance()),
            Commands.runOnce(windmill::calibrate),
            // Drive
            dtp,
            dtp.runWhen(() -> dtp.getTargetWaypoint() == 0 && dtp.getDistanceToWaypoint() <= ARM_UP_DISTANCE, stowToL4),
            dtp.runWhen(
                () -> dtp.getTargetWaypoint() == 1 && dtp.getDistanceToWaypoint() <= SCORING_DISTANCE,
                scoreGrabber.andThen(Commands.runOnce(() -> Logger.info("Scored grabber.")).andThen(Commands.runOnce(dtp::cancel)))
            )
        ).andThen(Commands.runOnce(() -> Logger.info("Driving to {} and scoring took {} seconds.", pole, timer.get())));
    }

    public Command driveToPoleThenScoreWhileCollecting(char pole) {
        return driveToPoleThenScoreWhileCollecting(pole, () -> 0);
    }

    public Command driveToPoleThenScoreWhileCollecting(char pole, DoubleSupplier fudge) {
        // Drive to the scoring position
        DriveToPoseV4Command dtp = AutonomousConstants.getObjectiveForPole(pole - 'A', alliance).fudgeY(fudge.getAsDouble()).driveToPoseV4Command();

        // Move arm from STOW to L4
        Command stowToL4 = WindmillMoveCommand.fromTo(STOW, L4).orElseThrow();

        // Score grabber
        Command scoreGrabber = grabber.runOnce(grabber::transitionToScoring).andThen(Commands.waitSeconds(SCORING_TIME));

        Timer timer = new Timer();
        return Commands.race(
            // Run the timer until the commands are done
            Commands.startRun(timer::restart, () -> {}),
            // Drive to the scoring position
            dtp,
            Commands
                .waitUntil(grabber::isHoldingCoral)
                .andThen(Commands.parallel(
                    // Move the arm to stow once collected
                    WindmillMoveCommand
                        .fromTo(CORAL_COLLECT, STOW).orElseThrow()
                        .andThen(
                            // Score the coral if collected
                            dtp.runWhen(() -> dtp.getTargetWaypoint() == 0 && dtp.getDistanceToWaypoint() <= ARM_UP_DISTANCE, stowToL4)
                        ),
                    dtp.runWhen(() -> dtp.getTargetWaypoint() == 1 && dtp.getDistanceToWaypoint() <= SCORING_DISTANCE, scoreGrabber.andThen(Commands.runOnce(dtp::cancel)))
                ).onlyIf(() -> dtp.getDistanceToWaypoint() > ARM_DOWN_DISTANCE)) // Only move the arm and score the coral if it is safe to do so
        ).andThen(Commands.runOnce(() -> Logger.info("Driving to {} and scoring took {} seconds.", pole, timer.get())));
    }

    private Command driveToCoralStationAndCollect(boolean isLeft) {
        Timer timer = new Timer();
        return Commands.defer(
            () -> {
                var dtp = AutonomousConstants.getObjectiveForCoralStation(isLeft, Chassis.getInstance().getPose().getTranslation(), alliance)
                                             .driveToPoseV4Command();

                return Commands.parallel(
                    Commands.runOnce(timer::restart),
                    // Drive to the coral station using the current translation of the chassis
                    dtp,
                    dtp.runWhen(
                           () -> dtp.getDistanceFromStart() > ARM_DOWN_DISTANCE,
                           WindmillMoveCommand.fromTo(L4, CORAL_COLLECT).orElseThrow()
                       )
                       .andThen(grabber.runOnce(grabber::transitionToCoralCollecting)),
                    // Collect from the collector and indexer
                    collector.runOnce(collector::deploymentTransitionToCollect).andThen(collector.runOnce(collector::collectorTransitionToCollecting)),
                    indexer.runOnce(indexer::transitionToCollecting)
                ).andThen(Commands.runOnce(() -> Logger.info("Driving to coral station took {} seconds.", timer.get())));
            },
            Set.of(chassis, windmill, grabber, collector, indexer)
        );
    }

    public Command driveToCoralAndCollect(boolean isLeft) {
        if (!RobotConfiguration.FEATURE_CORAL_DETECTION) {
            return driveToCoralStationAndCollect(isLeft);
        }

        Timer timer = new Timer();
        return Commands.defer(
            () -> {
                var dtp = AutonomousConstants.getObjectiveForCoralStation(isLeft, Chassis.getInstance().getPose().getTranslation(), alliance)
                                             .driveToPoseV5Command();

                return Commands.parallel(
                    Commands.runOnce(timer::restart),
                    // Drive to the coral station using the current translation of the chassis
                    dtp.withTimeout(5).until((RobotConfiguration.FEATURE_BEAN_BAKE) ? grabber::isCoralDetected : indexer::isBeanBakeTripped),
                    dtp.runWhen(
                           () -> dtp.getDistanceFromStart() > ARM_DOWN_DISTANCE,
                           WindmillMoveCommand.fromTo(L4, CORAL_COLLECT).orElseThrow()
                       )
                       .andThen(grabber.runOnce(grabber::transitionToCoralCollecting)),
                    // Collect from the collector and indexer
                    collector.runOnce(collector::deploymentTransitionToCollect).andThen(collector.runOnce(collector::collectorTransitionToCollecting)),
                    indexer.runOnce(indexer::transitionToCollecting)
                ).andThen(Commands.runOnce(() -> Logger.info("Driving to coral took {} seconds.", timer.get())));
            },
            Set.of(chassis, windmill, grabber, collector, indexer)
        );
    }
}
