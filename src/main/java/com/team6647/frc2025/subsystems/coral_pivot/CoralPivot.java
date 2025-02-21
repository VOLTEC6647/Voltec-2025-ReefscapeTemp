package com.team6647.frc2025.subsystems.coral_pivot;


import com.team1678.frc2024.subsystems.MotorIO;
import com.team1678.frc2024.subsystems.servo.ServoMotorSubsystem;
import com.team1678.frc2024.subsystems.servo.ServoMotorSubsystem2Config;
import com.team254.lib.util.Util;
import com.team6647.frc2025.subsystems.coral_roller.CoralRoller;
import com.team6647.frc2025.subsystems.coral_roller.CoralRollerIOTalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import java.util.function.DoubleSupplier;

/**
 * The ClimberSubsystem class is responsible for controlling the robot's climber
 * mechanism.
 * It manages motor positions, coordinates with the robot's state, and allows
 * for precise
 * movement control using motion magic and position feedback. This subsystem
 * includes commands
 * for setting motor positions, moving the climber slowly via duty cycle to a
 * certain position,
 * and zeroing the position after a climb.
 */

public class CoralPivot extends ServoMotorSubsystem<MotorInputsAutoLogged, MotorIO> {
    private double positionCmd;
    private static CoralPivot mInstance;

    public static CoralRoller getInstance() {
		if (mInstance == null) {
			mInstance = new CoralPivot(new Constants.ClimberConstants().);
		}
		return mInstance;
	}

    public CoralPivot(ServoMotorSubsystem2Config c, final MotorIO io) {

        super(c, new MotorInputsAutoLogged(), io);
        this.robotState = robotState;
        setCurrentPositionAsZero();
        setDefaultCommand(Commands.runOnce(() -> {
            positionCmd = inputs.unitPosition;
        }).andThen(positionSetpointCommand(() -> positionCmd)));
    }

    //public Command motionMagicSetpointCommandBlocking(double setpoint, double tolerance) {
    //    return motionMagicSetpointCommand(() -> setpoint)
    //            .until(() -> Util.epsilonEquals(getCurrentPosition(), setpoint, tolerance));
    //}

    //public Command jogAndThenZero() {
    //    return new ParallelCommandGroup(
    //            withoutLimitsTemporarily(),
    //            dutyCycleCommand(() -> -0.1)).finallyDo(() -> {
    //                setCurrentPositionAsZero();
    //            });
    //}

    //public Command waitForClimberPosition(DoubleSupplier targetPosition) {
    //    return new WaitUntilCommand(() -> Util.epsilonEquals(inputs.unitPosition, targetPosition.getAsDouble(),
    //            Constants.ClimberConstants.kClimbClimbedPositionToleranceRotations));
    //}
}