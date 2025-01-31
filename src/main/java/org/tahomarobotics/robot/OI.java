package org.tahomarobotics.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.chassis.ChassisCommands;
import org.tahomarobotics.robot.elevator.Elevator;
import org.tahomarobotics.robot.elevator.ElevatorConstants;
import org.tahomarobotics.robot.indexer.Indexer;
import org.tahomarobotics.robot.indexer.IndexerCommands;
import org.tahomarobotics.robot.util.SubsystemIF;
import org.tahomarobotics.robot.util.sysid.SysIdTests;

import java.util.List;
import java.util.function.Function;

@Logged(strategy = Logged.Strategy.OPT_IN)
public class OI extends SubsystemIF {
    private static final OI INSTANCE = new OI();

    // -- Constants --

    private static final double ROTATIONAL_SENSITIVITY = 2.0;
    private static final double TRANSLATIONAL_SENSITIVITY = 1.3;

    private static final double DEADBAND = 0.09;

    // -- Subsystems --

    private final Indexer indexer = Indexer.getInstance();
    private final Chassis chassis = Chassis.getInstance();
    private final Elevator elevator = Elevator.getInstance();

    private final List<SubsystemIF> subsystems = List.of(indexer, chassis, elevator);

    // -- Controllers --

    private final CommandXboxController controller = new CommandXboxController(0);

    // -- Initialization --

    private OI() {
        CommandScheduler.getInstance().unregisterSubsystem(this);
        DriverStation.silenceJoystickConnectionWarning(true);

        configureBindings();
        setDefaultCommands();
    }

    public static OI getInstance() {
        return INSTANCE;
    }

    // -- Bindings --

    public void configureBindings() {
        // Chassis

        controller.povDown().onTrue(Commands.runOnce(chassis::orientToZeroHeading));

        // Indexer

        Pair<Command, Command> indexerCommands = IndexerCommands.createIndexerCommands(indexer);
        controller.leftTrigger().onTrue(indexerCommands.getFirst()).onFalse(indexerCommands.getSecond());

        Pair<Command, Command> indexerEjectingCommands = IndexerCommands.createIndexerEjectingCommands(indexer);
        controller.povLeft().onTrue(indexerEjectingCommands.getFirst())
                       .onFalse(indexerEjectingCommands.getSecond());

        // Elevator

        // TODO: Temporary Controls

        controller.y().onTrue(Commands.runOnce(
            () -> elevator.setElevatorHeight(ElevatorConstants.ELEVATOR_HIGH_POSE)
        ));

        controller.b().onTrue(Commands.runOnce(
            () -> elevator.setElevatorHeight(ElevatorConstants.ELEVATOR_MID_POSE)
        ));

        controller.a().onTrue(Commands.runOnce(
            () -> elevator.setElevatorHeight(ElevatorConstants.ELEVATOR_LOW_POSE)
        ));
    }

    public void setDefaultCommands() {
        chassis.setDefaultCommand(ChassisCommands.createTeleOpDriveCommand(
            chassis,
            this::getLeftY, this::getLeftX, this::getRightX
        ));
    }

    // -- Inputs --

    @Logged(name = "Controller/LeftX")
    public double getLeftX() {
        return -desensitizePowerBased(controller.getLeftX(), TRANSLATIONAL_SENSITIVITY);
    }

    @Logged(name = "Controller/LeftY")
    public double getLeftY() {
        return -desensitizePowerBased(controller.getLeftY(), TRANSLATIONAL_SENSITIVITY);
    }

    @Logged(name = "Controller/RightX")
    public double getRightX() {
        return -desensitizePowerBased(controller.getRightX(), ROTATIONAL_SENSITIVITY);
    }

    // -- SysID --

    public void initializeSysId() {
        // Clear all bound triggers and default commands

        CommandScheduler scheduler = CommandScheduler.getInstance();

        subsystems.forEach(scheduler::removeDefaultCommand);
        scheduler.getActiveButtonLoop().clear();
        scheduler.cancelAll();

        // Allow for selection of tests

        SendableChooser<SysIdTests.Test> chooser = new SendableChooser<>();
        chooser.setDefaultOption("Nothing", null);

        subsystems.stream().flatMap(s -> s.getSysIdTests().stream())
                  .forEachOrdered(test -> chooser.addOption(test.name(), test));

        SmartDashboard.putData("SysId Test", chooser);

        // Create proxy commands for controller bindings

        Function<Function<SysIdTests.Test, Command>, Command> getCommand =
            (command) -> Commands.deferredProxy(() -> {
                SysIdTests.Test selected = chooser.getSelected();
                if (selected == null) {
                    return Commands.none();
                }
                return command.apply(selected);
            });

        Command quasistaticForward = getCommand.apply(SysIdTests.Test::quasistaticForward);
        Command quasistaticReverse = getCommand.apply(SysIdTests.Test::quasistaticReverse);
        Command dynamicForward = getCommand.apply(SysIdTests.Test::dynamicForward);
        Command dynamicReverse = getCommand.apply(SysIdTests.Test::dynamicReverse);

        // Register commands to the controller

        controller.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
        controller.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));

        controller.povUp().whileTrue(quasistaticForward);
        controller.povDown().whileTrue(quasistaticReverse);
        controller.povLeft().whileTrue(dynamicForward);
        controller.povRight().whileTrue(dynamicReverse);
    }

    public void cleanUpSysId() {
        // Clear all bound triggers
        CommandScheduler.getInstance().getActiveButtonLoop().clear();

        // Rebind proper controls
        configureBindings();
        setDefaultCommands();
    }

    // -- Helper Methods --

    public double desensitizePowerBased(double value, double power) {
        value = MathUtil.applyDeadband(value, DEADBAND);
        value *= Math.pow(Math.abs(value), power - 1);
        return value;
    }
}