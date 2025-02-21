package com.team6647.frc2025;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team1678.frc2024.subsystems.servo.ServoMotorSubsystem;
import com.team1678.frc2024.subsystems.servo.ServoMotorSubsystem.ServoMotorSubsystemConstants;
import com.team1678.frc2024.subsystems.servo.ServoMotorSubsystem.TalonFXConstants;
import com.team1678.frc2024.subsystems.servo.ServoMotorSubsystem2Config;
import com.team1678.frc2024.subsystems.servo.ServoMotorSubsystemWithCancoder.AbsoluteEncoderConstants;
import com.team1678.lib.Conversions;
import com.team254.lib.drivers.CanDeviceId;
import com.team254.lib.util.Units;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Constants {
    public static class OperatorConstants {

    }
    public static class DriveConstants {
        public static String swerveCANBus = "6647_Swerve";
		public static String mechanismsCANBus = "6647_CANivore";
        //public static String swerveCANBus = "rio";


    }

	public static final ClosedLoopRampsConfigs makeDefaultClosedLoopRampConfig() {
        return new ClosedLoopRampsConfigs()
                .withDutyCycleClosedLoopRampPeriod(0.02)
                .withTorqueClosedLoopRampPeriod(0.02)
                .withVoltageClosedLoopRampPeriod(0.02);
    }

    public static final OpenLoopRampsConfigs makeDefaultOpenLoopRampConfig() {
        return new OpenLoopRampsConfigs()
                .withDutyCycleOpenLoopRampPeriod(0.02)
                .withTorqueOpenLoopRampPeriod(0.02)
                .withVoltageOpenLoopRampPeriod(0.02);
    }

	public static final class ClimberConstants {
        public static final CanDeviceId kClimberTalonCanID = Ports.CORAL_PIVOT;
        public static final double kClimberP = 1.0;
        public static final double kForwardMaxPositionRotations = 119.0;
        public static final double kHooksUpPositionRotations = kForwardMaxPositionRotations * 0.9;
        public static final double kStageHooksRotations = kForwardMaxPositionRotations * 0.4;
        public static final double kClimbClimbedPositionToleranceRotations = kForwardMaxPositionRotations * 0.1;
        public static final double kPositionToleranceRotations = 2.0;
        public static final double kClimberGearRatio = 1.0 / (10.0);
        public static double kReverseMinPositionRotations = 0.0;
    }

    public static final ServoMotorSubsystem2Config kClimberConfig = new ServoMotorSubsystem2Config();
    static {
        kClimberConfig.name = "Climber";
        kClimberConfig.talonCANID = Ports.CORAL_PIVOT;
        kClimberConfig.kMaxPositionUnits = ClimberConstants.kForwardMaxPositionRotations;
        kClimberConfig.kMinPositionUnits = 0.0;
        kClimberConfig.fxConfig.Slot0.kP = 1.0;
        kClimberConfig.fxConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        kClimberConfig.fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        kClimberConfig.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        kClimberConfig.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        kClimberConfig.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.ClimberConstants.kForwardMaxPositionRotations
                - Constants.ClimberConstants.kPositionToleranceRotations;
        kClimberConfig.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;

        kClimberConfig.fxConfig.Audio.BeepOnBoot = false;
        kClimberConfig.fxConfig.Audio.BeepOnConfig = false;
        kClimberConfig.unitToRotorRatio = 1.0;

        kClimberConfig.fxConfig.CurrentLimits.StatorCurrentLimit = 40.0;
        kClimberConfig.fxConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        kClimberConfig.fxConfig.ClosedLoopRamps = makeDefaultClosedLoopRampConfig();
        kClimberConfig.fxConfig.OpenLoopRamps = makeDefaultOpenLoopRampConfig();
    }

    public static final class ElevatorConstants {
        public static final double ElevatorMinPositionRotations = 0.0;
        public static final double ElevatorMaxPositionRotations = 15.356933;
        public static final double ElevatorMaxHeightInches = 16.5;
        public static final double kElevatorGearRatio = (11.0 / 36.0) * (18. / 15.);
        public static final double kElevatorPositionToleranceRotations = 0.1;
        public static final double kAmpScoringHeightInches = 16.0;
        public static final double kElevatorHomeHeightInches = 0.0;
        public static final double kIntakeFromSourceHeightInches = 14.5;
        public static final double kElevatorPositioningToleranceInches = 0.5;
        public static final double kClimbHeightInches = 16.0;
        public static final double kSpoolDiameter = Units.inches_to_meters(0.940);
		public static double kTolerance = 20; /////
    }

    public static final ServoMotorSubsystem2Config kElevatorConfig = new ServoMotorSubsystem2Config();
    static {
        kElevatorConfig.name = "Elevator";
        kElevatorConfig.talonCANID = Ports.ELEVATOR_MAIN;
        kElevatorConfig.fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        kElevatorConfig.fxConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        kElevatorConfig.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        kElevatorConfig.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        kElevatorConfig.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = (ElevatorConstants.ElevatorMaxHeightInches
                - 0.25) / ElevatorConstants.ElevatorMaxHeightInches * ElevatorConstants.ElevatorMaxPositionRotations;
        kElevatorConfig.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
        kElevatorConfig.fxConfig.Slot0.kG = 0.26;
        kElevatorConfig.fxConfig.Slot0.kS = 0.18;
        kElevatorConfig.fxConfig.Slot0.kV = 0.135;
        kElevatorConfig.fxConfig.Slot0.kA = 0.0001 * 12.0;
        kElevatorConfig.fxConfig.Slot0.kP = 3.0;

        kElevatorConfig.fxConfig.MotionMagic.MotionMagicAcceleration = 800;
        kElevatorConfig.fxConfig.MotionMagic.MotionMagicCruiseVelocity = 80;
        kElevatorConfig.unitToRotorRatio = ElevatorConstants.ElevatorMaxHeightInches
                / (ElevatorConstants.ElevatorMaxPositionRotations - ElevatorConstants.ElevatorMinPositionRotations);
        kElevatorConfig.kMinPositionUnits = 0.0;
        kElevatorConfig.kMaxPositionUnits = ElevatorConstants.ElevatorMaxHeightInches;

        kElevatorConfig.fxConfig.Audio.BeepOnBoot = false;
        kElevatorConfig.fxConfig.Audio.BeepOnConfig = false;

        kElevatorConfig.fxConfig.CurrentLimits.StatorCurrentLimit = 80.0;
        kElevatorConfig.fxConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        //kElevatorConfig.fxConfig.ClosedLoopRamps = makeDefaultClosedLoopRampConfig();
        //kElevatorConfig.fxConfig.OpenLoopRamps = makeDefaultOpenLoopRampConfig();
    }

	public static final class AlgaeRollerConstants {
		public static TalonFXConfiguration RollerFXConfig() {
			TalonFXConfiguration config = new TalonFXConfiguration();

			config.CurrentLimits.SupplyCurrentLimitEnable = true;
			config.CurrentLimits.SupplyCurrentLimit = 40.0;

			config.CurrentLimits.StatorCurrentLimitEnable = true;
			config.CurrentLimits.StatorCurrentLimit = 80.0;

			config.Voltage.PeakForwardVoltage = 4.0;
			config.Voltage.PeakReverseVoltage = -4.0;

			config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
			
			return config;
		}
	}

	public static final class AlgaeHolderConstants {
		public static SparkMaxConfig SparkMaxConfig() {
			SparkMaxConfig config = new SparkMaxConfig();

			config.smartCurrentLimit(30);

			//config.Voltage.PeakForwardVoltage = 12.0;
			//config.Voltage.PeakReverseVoltage = -12.0;

			config.idleMode(IdleMode.kBrake);
			return config;
		}
		public double movementThreshold = 5.0;
	}

	public static final class CoralPivotConstants {

		public static final double kRotorRotationsPerOutputRotation = 1.0; // Rotor to unit distance

		public static final ServoMotorSubsystemConstants kHoodServoConstants = new ServoMotorSubsystemConstants();
		public static final AbsoluteEncoderConstants kHoodEncoderConstants = new AbsoluteEncoderConstants();

		public static final double kHomingVoltage = -2.0;
		public static final double kHomingCurrentThreshold = 10.0;
		public static final double kMinHomingTime = 0.2;
		public static final double kMaxHomingTime = 4.0;

		static {
			kHoodServoConstants.kName = "CoralPivot";

			kHoodServoConstants.kMainConstants.id = Ports.CORAL_PIVOT;
			kHoodServoConstants.kMainConstants.counterClockwisePositive = true;

			kHoodServoConstants.kHomePosition = 0; // Degrees
			kHoodServoConstants.kTolerance = 1; // Degrees
			kHoodServoConstants.kRotationsPerUnitDistance = (1.0 / 360.0) /* (7.16 / 1.0)*/; // Cancoder to unit distance
			kHoodServoConstants.kKp = 30.0;
			kHoodServoConstants.kKi = 0;
			kHoodServoConstants.kKd = 0.0;
			kHoodServoConstants.kKg = 0;
			kHoodServoConstants.kKs = 0.0;
			kHoodServoConstants.kDeadband = 0; // Ticks

			//kHoodServoConstants.kMinUnitsLimit = 15.0;
			//kHoodServoConstants.kMaxUnitsLimit = 62.0;

			kHoodServoConstants.kCruiseVelocity = 200.0; // degrees / s
			kHoodServoConstants.kAcceleration = 20.0; // degrees / s^2

			kHoodServoConstants.kEnableSupplyCurrentLimit = true;
			kHoodServoConstants.kSupplyCurrentLimit = 40;
			kHoodServoConstants.kSupplyCurrentThreshold = 40;

			kHoodServoConstants.kEnableStatorCurrentLimit = true;
			kHoodServoConstants.kStatorCurrentLimit = 40;

			kHoodServoConstants.kMaxForwardOutput = 12.0;
			kHoodServoConstants.kMaxReverseOutput = -12.0;//12

			kHoodServoConstants.kRampRate = 0.0;

			kHoodServoConstants.kNeutralMode = NeutralModeValue.Coast;

			kHoodEncoderConstants.encoder_type = FeedbackSensorSourceValue.FusedCANcoder; //FusedCANcoder
			kHoodEncoderConstants.remote_encoder_port = Ports.CORAL_CANCODER;
			kHoodEncoderConstants.rotor_rotations_per_output = 0.0;
			kHoodEncoderConstants.remote_encoder_offset = 6.644531;
		}
	}

	public static final class CoralPivotConstantsSolo {

		public static final double kRotorRotationsPerOutputRotation = 25.0 / 1.0; // Rotor to unit distance

		public static final ServoMotorSubsystemConstants kHoodServoConstants = new ServoMotorSubsystemConstants();
		public static final AbsoluteEncoderConstants kHoodEncoderConstants = new AbsoluteEncoderConstants();

		public static final double kHomingVoltage = -2.0;
		public static final double kHomingCurrentThreshold = 10.0;
		public static final double kMinHomingTime = 0.2;
		public static final double kMaxHomingTime = 4.0;
		public static final double kHomePosition = 30;

		static {
			kHoodServoConstants.kName = "CoralPivot";

			kHoodServoConstants.kMainConstants.id = Ports.CORAL_PIVOT;
			kHoodServoConstants.kMainConstants.counterClockwisePositive = false;

			kHoodServoConstants.kHomePosition = 50; // Degrees
			kHoodServoConstants.kTolerance = 1; // Degrees
			kHoodServoConstants.kRotationsPerUnitDistance = (1.0 / 360.0) /* (7.16 / 1.0)*/; // Cancoder to unit distance
			kHoodServoConstants.kKp = 30.0;
			kHoodServoConstants.kKi = 0;
			kHoodServoConstants.kKd = 0.0;
			kHoodServoConstants.kKg = 0;
			kHoodServoConstants.kKs = 0.0;
			kHoodServoConstants.kDeadband = 0; // Ticks

			//kHoodServoConstants.kMinUnitsLimit = 15.0;
			//kHoodServoConstants.kMaxUnitsLimit = 62.0;

			kHoodServoConstants.kCruiseVelocity = 200.0; // degrees / s
			kHoodServoConstants.kAcceleration = 60.0; // degrees / s^2

			kHoodServoConstants.kEnableSupplyCurrentLimit = true;
			kHoodServoConstants.kSupplyCurrentLimit = 40;
			kHoodServoConstants.kSupplyCurrentThreshold = 40;

			kHoodServoConstants.kEnableStatorCurrentLimit = true;
			kHoodServoConstants.kStatorCurrentLimit = 40;

			kHoodServoConstants.kMaxForwardOutput = 12.0;
			kHoodServoConstants.kMaxReverseOutput = -12.0;//12

			kHoodServoConstants.kRampRate = 0.0;

			kHoodServoConstants.kNeutralMode = NeutralModeValue.Coast;

			kHoodEncoderConstants.encoder_type = FeedbackSensorSourceValue.FusedCANcoder; //FusedCANcoder
			kHoodEncoderConstants.remote_encoder_port = Ports.CORAL_CANCODER;
			kHoodEncoderConstants.rotor_rotations_per_output = 0.0;
			kHoodEncoderConstants.remote_encoder_offset = 6.644531;

			
		}
	}

    
    
}
