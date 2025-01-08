package com.team1678.frc2024.controlboard;

import com.team1678.frc2024.subsystems.Drive;
import com.team1678.frc2024.subsystems.vision.VisionDeviceManager;
import com.team6647.frc2025.subsystems.Superstructure;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriverControls1678 {

	ControlBoard mControlBoard = ControlBoard.getInstance();

	Superstructure mSuperstructure = Superstructure.getInstance();
	Drive mDrive = Drive.getInstance();

	private boolean top_buttons_clear = true;

	/* ONE CONTROLLER */

	public void oneControllerMode() {
		if (mControlBoard.driver.rightTrigger.isBeingPressed()) {
			mDrive.overrideHeading(true);
		} else {
			mDrive.overrideHeading(false);
		}
	}

	private boolean mClimberJog = false;

	/* TWO CONTROLLERS */

	
	public void twoControllerMode() {
		
	}
	

}