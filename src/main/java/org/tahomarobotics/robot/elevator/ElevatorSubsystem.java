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

package org.tahomarobotics.robot.elevator;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.Logger;
import org.tahomarobotics.robot.RobotMap;
import org.tahomarobotics.robot.util.AbstractSubsystem;

public class ElevatorSubsystem extends AbstractSubsystem {
    // Motors
    private final TalonFX leftMotor = new TalonFX(RobotMap.ELEVATOR_LEFT_MOTOR);
    private final TalonFX rightMotor = new TalonFX(RobotMap.ELEVATOR_RIGHT_MOTOR);

    // States
    private Mode mode = Mode.DISCRETE;

    ElevatorSubsystem() {
        rightMotor.setControl(new Follower(RobotMap.ELEVATOR_LEFT_MOTOR, true));
    }

    // Setters
    public void moveToMinPosition() {

    }

    public void moveToMaxPosition() {

    }

    public void toggleMode() {
        mode = (mode == Mode.DISCRETE) ? Mode.CONTINUOUS : Mode.DISCRETE;
    }

    @Override
    public void subsystemPeriodic() {
        Logger.recordOutput("Elevator/Mode", mode);
    }

    enum Mode {
        CONTINUOUS,
        DISCRETE
    }
}
