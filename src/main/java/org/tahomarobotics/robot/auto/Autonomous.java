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

package org.tahomarobotics.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.tahomarobotics.robot.auto.autos.AssembledAuto;
import org.tahomarobotics.robot.auto.autos.Strait;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.util.SubsystemIF;
import org.tinylog.Logger;

import java.util.LinkedHashMap;
import java.util.function.DoubleSupplier;

public class Autonomous extends SubsystemIF {
    private static final Autonomous INSTANCE = new Autonomous();

    private final Command cachedFivePieceCommandLeftBlue, cachedFivePieceCommandRightBlue;
    private final Command cachedFivePieceCommandLeftRed, cachedFivePieceCommandRightRed;
    private final Command cachedJackFivePieceCommandRightRed, cachedJackFivePieceCommandRightBlue;
    private final Command cachedJackFivePieceCommandLeftRed, cachedJackFivePieceCommandLeftBlue;
    private final SendableChooser<Command> autoChooser;

    private Autonomous() {
        autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption("No-Op", Commands.none().withName("No-Op"));

        cachedFivePieceCommandLeftBlue = assembleFivePiece(true, DriverStation.Alliance.Blue);
        cachedFivePieceCommandLeftRed = assembleFivePiece(true, DriverStation.Alliance.Red);

        cachedFivePieceCommandRightBlue = assembleFivePiece(false, DriverStation.Alliance.Blue);
        cachedFivePieceCommandRightRed = assembleFivePiece(false, DriverStation.Alliance.Red);

        cachedJackFivePieceCommandLeftBlue = assembleCompatibleFivePiece(true, DriverStation.Alliance.Blue);
        cachedJackFivePieceCommandLeftRed = assembleCompatibleFivePiece(true, DriverStation.Alliance.Red);

        cachedJackFivePieceCommandRightBlue = assembleCompatibleFivePiece(false, DriverStation.Alliance.Blue);
        cachedJackFivePieceCommandRightRed = assembleCompatibleFivePiece(false, DriverStation.Alliance.Red);

        autoChooser.addOption(
            "5-Piece Left", Commands.deferredProxy(() -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Red) == DriverStation.Alliance.Red ?
                cachedFivePieceCommandLeftRed : cachedFivePieceCommandLeftBlue).withName("5-Piece Left")
        );
        autoChooser.addOption(
            "5-Piece Right", Commands.deferredProxy(() -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Red) == DriverStation.Alliance.Red ?
                cachedFivePieceCommandRightRed : cachedFivePieceCommandRightBlue).withName("5-Piece Right")
        );
        autoChooser.addOption(
            "Compatible 5-Piece Left", Commands.deferredProxy(() -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Red) == DriverStation.Alliance.Red ?
                cachedJackFivePieceCommandLeftRed : cachedJackFivePieceCommandLeftBlue).withName("Compatible 5-Piece Left")
        );
        autoChooser.addOption(
            "Compatible 5-Piece Right", Commands.deferredProxy(() -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Red) == DriverStation.Alliance.Red ?
                cachedJackFivePieceCommandRightRed : cachedJackFivePieceCommandRightBlue).withName("Compatible 5-Piece Right")
        );
        autoChooser.addOption(
            "Strait", Commands.deferredProxy(() -> new Strait(DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue))).withName("Strait")
        );

        // Force-Load the reef positions
        AutonomousConstants.getNearestReefPoleScorePosition(new Pose2d().getTranslation()).approachPose().getX();

        SmartDashboard.putData("Auto Chooser", autoChooser);

        autoChooser.onChange(this::onAutoChange);
    }

    public static Autonomous getInstance() {
        return INSTANCE;
    }

    public Command getSelectedAuto() {
        return autoChooser.getSelected();
    }

    public Command assembleFivePiece(boolean isLeft, DriverStation.Alliance alliance) {
        LinkedHashMap<Character, DoubleSupplier> scorePositions = new LinkedHashMap<>();
        scorePositions.put('J', () -> alliance == DriverStation.Alliance.Red && isLeft ? Units.inchesToMeters(1) : 0);
        scorePositions.put('K', () -> alliance == DriverStation.Alliance.Red && isLeft ? Units.inchesToMeters(2) : 0);
        scorePositions.put('L', () -> alliance == DriverStation.Alliance.Red && isLeft ? Units.inchesToMeters(2) : 0);
        scorePositions.put('A', () -> 0);
        return new AssembledAuto(isLeft, scorePositions, alliance, "Five-Piece");
    }

    public Command assembleCompatibleFivePiece(boolean isLeft, DriverStation.Alliance alliance) {
        LinkedHashMap<Character, DoubleSupplier> scorePositions = new LinkedHashMap<>();
        scorePositions.put('I', () -> 0);
        scorePositions.put('J', () -> alliance == DriverStation.Alliance.Red && isLeft ? Units.inchesToMeters(1) : 0);
        scorePositions.put('K', () -> alliance == DriverStation.Alliance.Red && isLeft ? Units.inchesToMeters(2) : 0);
        scorePositions.put('L', () -> alliance == DriverStation.Alliance.Red && isLeft ? Units.inchesToMeters(2) : 0);
        return new AssembledAuto(isLeft, scorePositions, alliance, "Compatible Five-Piece");
    }

    public void onAutoChange(Command command) {
        Chassis chassis = Chassis.getInstance();
        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Red);
        Pose2d startingPose = switch(command.getName()) {
            case "5-Piece Left" -> (alliance == DriverStation.Alliance.Red) ? AutonomousConstants.FIVE_PIECE_LEFT_RED_START : AutonomousConstants.FIVE_PIECE_LEFT_BLUE_START;
            case "5-Piece Right" -> (alliance == DriverStation.Alliance.Red) ? AutonomousConstants.FIVE_PIECE_RIGHT_RED_START : AutonomousConstants.FIVE_PIECE_RIGHT_BLUE_START;
            case "Strait" -> (alliance == DriverStation.Alliance.Red) ? AutonomousConstants.STRAIGHT_RED_START : AutonomousConstants.STRAIGHT_BLUE_START;
            default -> chassis.getPose();
        };

        chassis.resetOdometry(startingPose);
        Logger.info("Selected auto: " + command.getName());
    }
}