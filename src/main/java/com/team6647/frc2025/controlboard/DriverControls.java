package com.team6647.frc2025.controlboard;

import com.team1678.frc2024.FieldLayout.CoralTarget;
import com.team1678.frc2024.auto.AutoModeBase;
import com.team1678.frc2024.controlboard.ControlBoard;
import com.team1678.frc2024.subsystems.Drive;
import com.team1678.frc2024.subsystems.vision.VisionDeviceManager;
import com.team6647.frc2025.auto.modes.configuredQuals.test1;
import com.team6647.frc2025.subsystems.Superstructure;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriverControls {

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
	private AutoModeBase coralPlacer;

	double angles[] = {CoralTarget.LEFT.angle, CoralTarget.RIGHT.angle, CoralTarget.BOTTOM_LEFT.angle, CoralTarget.BOTTOM_RIGHT.angle, CoralTarget.TOP_LEFT.angle, CoralTarget.TOP_RIGHT.angle};

	/* TWO CONTROLLERS */

	//driver/operator
	public void twoControllerMode() {
		if(mControlBoard.driver.yButton.wasActivated()){
			coralPlacer = new test1();
			coralPlacer.run();
		}else if (mControlBoard.driver.yButton.wasReleased()) {
			coralPlacer.stop();
		}
		double angle = Math.toDegrees(Math.atan2(mControlBoard.operator.getRightX(), mControlBoard.operator.getRightY()));
		if (angle < 0) {
			angle += 360;
		}
		for (int i = 0; i < angles.length; i++) {
			if (Math.abs(angle - angles[i]) < 30) {
				Shuffleboard.getTab("Coral").addPersistent("Position", CoralTarget.values()[i].name());
				break;
			}
		}

	}


}