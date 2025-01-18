package com.team6647.frc2025;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team1678.frc2024.subsystems.servo.ServoMotorSubsystem;
import com.team1678.frc2024.subsystems.servo.ServoMotorSubsystem.ServoMotorSubsystemConstants;
import com.team1678.frc2024.subsystems.servo.ServoMotorSubsystem.TalonFXConstants;
import com.team1678.lib.Conversions;
import com.team254.lib.drivers.CanDeviceId;

public class Constants {
    public static class OperatorConstants {

    }
    public static class DriveConstants {
        public static String swerveCANBus = "6647_CANivore";
        //public static String swerveCANBus = "rio";


    }

    public static class ElevatorConstants {
        public static final ServoMotorSubsystemConstants kElevatorServoConstants = new ServoMotorSubsystemConstants();

		static {
			kElevatorServoConstants.kName = "Elevator";

			kElevatorServoConstants.kMainConstants.counterClockwisePositive = false;
			kElevatorServoConstants.kMainConstants.invert_sensor_phase = false;

			kElevatorServoConstants.kFollowerConstants = new TalonFXConstants[1];
			kElevatorServoConstants.kFollowerConstants[0] = new TalonFXConstants();

			kElevatorServoConstants.kMainConstants.id = Ports.ELEVATOR_MAIN;
			kElevatorServoConstants.kFollowerConstants[0].id = Ports.ELEVATOR_FOLLOWER;

			kElevatorServoConstants.kHomePosition = 0.0; // meters

			kElevatorServoConstants.kMaxUnitsLimit = 0.46;
			kElevatorServoConstants.kMinUnitsLimit = 0.0;

			kElevatorServoConstants.kRotationsPerUnitDistance = (9.0) / (Conversions.inchesToMeters(1.432) * Math.PI);

			kElevatorServoConstants.kKp = 1.0; // Raw output / raw error
			kElevatorServoConstants.kKi = 0.0; // Raw output / sum of raw error
			kElevatorServoConstants.kKd = 0.0; // Raw output / (err - prevErr)
			kElevatorServoConstants.kKa = 0.0; // Raw output / accel in (rots/s) / s
			kElevatorServoConstants.kKg = 0.2;
			kElevatorServoConstants.kDeadband = 0; // rots

			kElevatorServoConstants.kCruiseVelocity = 1.0; // m / s
			kElevatorServoConstants.kAcceleration = 120.0; // m / s^2
			kElevatorServoConstants.kRampRate = 0.0; // s

			kElevatorServoConstants.kMaxForwardOutput = 12.0;
			kElevatorServoConstants.kMaxReverseOutput = -12.0;

			kElevatorServoConstants.kEnableSupplyCurrentLimit = true;
			kElevatorServoConstants.kSupplyCurrentLimit = 40; // amps
			kElevatorServoConstants.kSupplyCurrentThreshold = 40; // amps
			kElevatorServoConstants.kSupplyCurrentTimeout = 0.01; // seconds

			kElevatorServoConstants.kNeutralMode = NeutralModeValue.Brake;
		}

		public static double kHomingZone = 0.1; // meters
		public static double kHomingTimeout = 0.5; // seconds
		public static double kHomingVelocityWindow = 0.1; // "units" / second
		public static double kHomingOutput = -2.0; // volts

		//public static double kHomingZone = 0; // meters
		//public static double kHomingTimeout = 0.5; // seconds
		//public static double kHomingVelocityWindow = 0.1; // "units" / second
		//public static double kHomingOutput = -2.0; // volts
    }

    
    
}
