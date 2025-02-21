package com.team1678.frc2024.subsystems.servo;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team1678.frc2024.loops.ILooper;
import com.team1678.frc2024.loops.Loop;
import com.team1678.frc2024.subsystems.MotorIO;
import com.team1678.frc2024.subsystems.MotorInputsAutoLogged;
import com.team1678.frc2024.subsystems.Subsystem;
import com.team1678.lib.requests.LambdaRequest;
import com.team1678.lib.requests.Request;
import com.team1678.lib.requests.SequentialRequest;
import com.team254.lib.util.Util;
import com.team6647.frc2025.Constants;

import edu.wpi.first.wpilibj2.command.*;

import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

/**
 * RollerMotorSubsystem
 * 
 * @param <T>
 */
public class ServoMotorSubsystem2<T extends MotorInputsAutoLogged, U extends MotorIO> extends Subsystem {
    protected U io;
    protected T inputs;
    private double positionSetpoint = 0.0;

    protected ServoMotorSubsystem2Config conf;

    public ServoMotorSubsystem2(ServoMotorSubsystem2Config config, T inputs, U io) {
        //super(config.name);
        this.conf = config;
        this.io = io;
        this.inputs = inputs;
    }

    public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {}

			@Override
			public void onLoop(double timestamp) {
				io.readInputs(inputs);
			}

			@Override
			public void onStop(double timestamp) {}
		});
	}

    @Override
	public void outputTelemetry() {
		Logger.processInputs(getName(), inputs);
	}

    protected void setOpenLoopDutyCycleImpl(double dutyCycle) {
        Logger.recordOutput(getName() + "/API/setOpenLoopDutyCycle/dutyCycle", dutyCycle);
        io.setOpenLoopDutyCycle(dutyCycle);
    }

    protected void setPositionSetpointImpl(double units) {
        positionSetpoint = units;
        Logger.recordOutput(getName() + "/API/setPositionSetpointImp/Units", units);
        io.setPositionSetpoint(units);
    }

    protected void setNeutralModeImpl(NeutralModeValue mode) {
        Logger.recordOutput(getName() + "/API/setNeutralModeImpl/Mode", mode);
        io.setNeutralMode(mode);
    }

    protected void setMotionMagicSetpointImpl(double units) {
        positionSetpoint = units;
        Logger.recordOutput(getName() + "/API/setMotionMagicSetpointImp/Units", units);
        io.setMotionMagicSetpoint(units);
    }

    protected void setVelocitySetpointImpl(double unitsPerSecond) {
        Logger.recordOutput(getName() + "/API/setVelocitySetpointImpl/UnitsPerS", unitsPerSecond);
        io.setVelocitySetpoint(unitsPerSecond);
    }

    public double getCurrentPosition() {
        return inputs.unitPosition;
    }

    public double getCurrentVelocity() {
        return inputs.velocityUnitsPerSecond;
    }

    public double getPositionSetpoint() {
        return positionSetpoint;
    }

    //public Request dutyCycleCommand(DoubleSupplier dutyCycle) {
    //    runEnd(() -> {
    //        setOpenLoopDutyCycleImpl(dutyCycle.getAsDouble());
    //    }, () -> {
    //        setOpenLoopDutyCycleImpl(0.0);
    //    }).withName(getName() + " DutyCycleControl");
    //}

    public Request velocitySetpointCommand(DoubleSupplier velocitySupplier) {
        return new LambdaRequest(()->{
            setVelocitySetpointImpl(velocitySupplier.getAsDouble());
        });
    }

    public Request setCoast() {
        return new LambdaRequest(()->{
            setNeutralModeImpl(NeutralModeValue.Brake);
        });
    }

    public Request positionSetpointCommand(DoubleSupplier unitSupplier) {
        return new LambdaRequest(()->{
            setPositionSetpointImpl(unitSupplier.getAsDouble());
        });
    }

    public Request positionSetpointUntilOnTargetCommand(DoubleSupplier unitSupplier, DoubleSupplier epsilon) {
		return new Request() {

			@Override
			public void act() {
				positionSetpointCommand(unitSupplier);
			}

			@Override
			public boolean isFinished() {
				return Util.epsilonEquals(unitSupplier.getAsDouble(), inputs.unitPosition, epsilon.getAsDouble());
			}
		};
	}

    public Request motionMagicSetpointCommand(DoubleSupplier unitSupplier) {
        return new LambdaRequest(()->{
            setMotionMagicSetpointImpl(unitSupplier.getAsDouble());
        });
    }

    protected void setCurrentPositionAsZero() {
        io.setCurrentPositionAsZero();
    }

    protected void setCurrentPosition(double positionUnits) {
        io.setCurrentPosition(positionUnits);
    }

    public Command waitForElevatorPosition(DoubleSupplier targetPosition) {
        return new WaitUntilCommand(() -> Util.epsilonEquals(inputs.unitPosition, targetPosition.getAsDouble(),
                Constants.ElevatorConstants.kTolerance));
    }

    protected Command withoutLimitsTemporarily() {
        var prev = new Object() {
            boolean fwd = false;
            boolean rev = false;
        };
        return Commands.startEnd(() -> {
            prev.fwd = conf.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable;
            prev.rev = conf.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable;
            io.setEnableSoftLimits(false, false);
        }, () -> {
            io.setEnableSoftLimits(prev.fwd, prev.rev);
        });
    }

    public String getName(){
        return conf.name;
    }

}