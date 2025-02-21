//modified 6647
package com.team6647.frc2025;

import com.team254.lib.drivers.CanDeviceId;
import com.team6647.frc2025.Constants.DriveConstants;

public class Ports {
	/*
	 * LIST OF CHANNEL AND CAN IDS
	 *
	 * Swerve Modules go:
	 * 0 1
	 * 2 3
	 *
	 * spotless:off
	 */

	/* DRIVETRAIN CAN DEVICE IDS */
	public static final CanDeviceId FR_ROTATION = new CanDeviceId(1, DriveConstants.swerveCANBus);
	public static final CanDeviceId FR_DRIVE = new CanDeviceId(2, DriveConstants.swerveCANBus);
	public static final CanDeviceId FR_CANCODER = new CanDeviceId(3, DriveConstants.swerveCANBus);

	public static final CanDeviceId BR_ROTATION = new CanDeviceId(4, DriveConstants.swerveCANBus);
	public static final CanDeviceId BR_DRIVE = new CanDeviceId(5, DriveConstants.swerveCANBus);
	public static final CanDeviceId BR_CANCODER = new CanDeviceId(6, DriveConstants.swerveCANBus);

	public static final CanDeviceId BL_ROTATION = new CanDeviceId(7, DriveConstants.swerveCANBus);
	public static final CanDeviceId BL_DRIVE = new CanDeviceId(8, DriveConstants.swerveCANBus);
	public static final CanDeviceId BL_CANCODER = new CanDeviceId(9, DriveConstants.swerveCANBus);

	public static final CanDeviceId FL_ROTATION = new CanDeviceId(10, DriveConstants.swerveCANBus);
	public static final CanDeviceId FL_DRIVE = new CanDeviceId(11, DriveConstants.swerveCANBus);
	public static final CanDeviceId FL_CANCODER = new CanDeviceId(12, DriveConstants.swerveCANBus);

	/* SUBSYSTEM CAN DEVICE IDS */
	public static final CanDeviceId ALGAE_ROLLER1 = new CanDeviceId(15, DriveConstants.mechanismsCANBus);
	public static final CanDeviceId ALGAE_ROLLER2 = new CanDeviceId(16, DriveConstants.mechanismsCANBus);

	public static final CanDeviceId ALGAE_HOLDER = new CanDeviceId(17, "rio");

	public static final CanDeviceId CORAL_ROLLER = new CanDeviceId(21, DriveConstants.mechanismsCANBus);

	//public static final CanDeviceId CORAL_PIVOT = new CanDeviceId(18, DriveConstants.mechanismsCANBus);
	public static final CanDeviceId CORAL_PIVOT = new CanDeviceId(19, DriveConstants.mechanismsCANBus);
	public static final CanDeviceId CORAL_CANCODER = new CanDeviceId(20, DriveConstants.mechanismsCANBus);
	public static final int CORAL_ENCODER = 0;

	public static final CanDeviceId TESTING_MOTOR = new CanDeviceId(55, DriveConstants.mechanismsCANBus);

	public static final CanDeviceId ELEVATOR_MAIN = new CanDeviceId(13, DriveConstants.mechanismsCANBus);
	public static final CanDeviceId ELEVATOR_FOLLOWER = new CanDeviceId(14, DriveConstants.mechanismsCANBus);


	public static final int PIGEON = 13;
	
	/* BEAM BREAK DIO CHANNELS*/
	//public static final int SERIALIZER_BREAK = Constants1678.isEpsilon ? 7 : 8;
	public static final int FEEDER_BREAK = 0;
	public static final int AMP_BREAK = 9; 

	/* LINEAR SERVO PWM CHANNELS */
	public static final int CLIMBER_LINEAR_ACTUATOR = 9;
	public static final int ELEVATOR_LINEAR_ACTUATOR = 0;

	// spotless:on
}
