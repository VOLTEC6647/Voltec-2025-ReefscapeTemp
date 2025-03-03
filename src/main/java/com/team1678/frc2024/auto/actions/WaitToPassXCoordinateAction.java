package com.team1678.frc2024.auto.actions;

import com.team1678.frc2024.subsystems.Drive;
import com.team6647.frc2025.FieldLayout;
import com.team6647.frc2025.Robot;

public class WaitToPassXCoordinateAction implements Action {
	double startingXCoordinate;
	double targetXCoordinate;
	Drive drive;

	public WaitToPassXCoordinateAction(double x) {
		if (Robot.is_red_alliance) {
			targetXCoordinate = FieldLayout.kFieldLength - x;
		} else {
			targetXCoordinate = x;
		}
		drive = Drive.getInstance();
	}

	@Override
	public boolean isFinished() {
		return Math.signum(startingXCoordinate - targetXCoordinate)
				!= Math.signum(drive.getPose().getTranslation().x() - targetXCoordinate);
	}

	@Override
	public void start() {
		startingXCoordinate = drive.getPose().getTranslation().x();
	}

	@Override
	public void update() {}

	@Override
	public void done() {}
}
