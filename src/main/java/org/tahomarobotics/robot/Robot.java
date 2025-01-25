package org.tahomarobotics.robot;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.tahomarobotics.robot.chassis.Chassis;
import org.tahomarobotics.robot.util.FauxWatchdog;
import org.tahomarobotics.robot.util.SubsystemIF;
import org.tahomarobotics.robot.vision.Vision;
import org.tinylog.Logger;

import java.util.List;

@Logged
public class Robot extends TimedRobot {
    // Subsystems

    @Logged(name = "OI")
    private final OI oi = OI.getInstance();
    @Logged(name = "Chassis")
    private final Chassis chassis = Chassis.getInstance();
    @Logged(name = "Vision")
    private final Vision vision = Vision.getInstance();

    @NotLogged
    private final List<SubsystemIF> subsystems = List.of(
        oi.initialize(),
        chassis.initialize(),
        vision.initialize()
    );

    // Robot

    public Robot() {
        Epilogue.configure(configuration -> {
            configuration.backend = configuration.backend.lazy();
            configuration.minimumImportance = Logged.Importance.DEBUG; // TODO
        });
        Epilogue.bind(this);

//        disableWatchdog(this, IterativeRobotBase.class);
//        disableWatchdog(CommandScheduler.getInstance(), CommandScheduler.class);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    // Disabled

    @Override
    public void disabledInit() {
        subsystems.forEach(SubsystemIF::onDisabledInit);
    }

    @Override
    public void disabledPeriodic() {}

    // Autonomous

    @Override
    public void autonomousInit() {
        subsystems.forEach(SubsystemIF::onAutonomousInit);
    }

    @Override
    public void autonomousPeriodic() {}

    // Teleop

    @Override
    public void teleopInit() {
        subsystems.forEach(SubsystemIF::onTeleopInit);
    }

    @Override
    public void teleopPeriodic() {}

    // Test

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    // Simulation

    @Override
    public void simulationInit() {
        subsystems.forEach(SubsystemIF::onSimulationInit);
    }

    @Override
    public void simulationPeriodic() {}

    // Util

    <T> void disableWatchdog(T inst, Class<?> clazz) {
        try {
            var field = clazz.getDeclaredField("m_watchdog");
            field.setAccessible(true);
            field.set(inst, new FauxWatchdog());
            Logger.info("Disabled {}'s watchdog!", inst.getClass());
        } catch (NoSuchFieldException | IllegalAccessException ignored) {}
    }
}
