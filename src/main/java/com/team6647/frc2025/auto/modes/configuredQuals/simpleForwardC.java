package com.team6647.frc2025.auto.modes.configuredQuals;

import com.team1678.frc2024.auto.AutoModeBase;
import com.team1678.frc2024.auto.AutoModeEndedException;
import com.team1678.frc2024.auto.actions.ChoreoTrajectoryAction;
import com.team1678.frc2024.auto.actions.LambdaAction;
import com.team1678.frc2024.auto.actions.ParallelAction;
import com.team1678.frc2024.auto.actions.SeriesAction;
import com.team1678.frc2024.auto.actions.SwerveTrajectoryAction;
import com.team1678.frc2024.auto.actions.TurnInPlaceAction;
import com.team1678.frc2024.auto.actions.WaitAction;
import com.team1678.frc2024.auto.actions.WaitForSuperstructureAction;
import com.team1678.frc2024.auto.actions.WaitToPassXCoordinateAction;
import com.team1678.frc2024.auto.actions.WaitToPassYCoordinateAction;
import com.team1678.frc2024.subsystems.Drive;
import com.team1678.lib.requests.LambdaRequest;
import com.team254.lib.geometry.Pose2dWithMotion;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.trajectory.Trajectory254;
import com.team254.lib.trajectory.timing.TimedState;
import com.team6647.frc2025.auto.actions.GenAction;
import com.team6647.frc2025.auto.actions.WaitForEnterPathGeneratedAction;
import com.team6647.frc2025.auto.actions.WaitForPathGeneratedAction;
import com.team6647.frc2025.auto.paths.TrajectoryGenerator;
import com.team6647.frc2025.auto.paths.TrajectoryGenerator.TrajectorySet;
import com.team6647.frc2025.subsystems.Superstructure;
import com.team6647.frc2025.subsystems.Superstructure.Levels;
import com.team6647.frc2025.subsystems.coral_roller.CoralRoller;
import com.team6647.frc2025.subsystems.coral_roller.CoralRoller.State;

import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import java.util.List;
import java.util.function.Supplier;

public class simpleForwardC extends AutoModeBase {
	private Drive d = Drive.getInstance();
	private Superstructure s = Superstructure.getInstance();
	public simpleForwardC() {
		
	}

	// spotless:off
	@Override
	protected void routine() throws AutoModeEndedException {
		runAction(new ChoreoTrajectoryAction("SimpleForward"));
		s.request(s.prepareLevel(Levels.LEVEL3));
		CoralRoller.getInstance().setState(CoralRoller.getInstance().OUTAKING);

		System.out.println("Finished auto!");
	}
	// spotless:on
}