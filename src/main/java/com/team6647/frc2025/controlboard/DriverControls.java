package com.team6647.frc2025.controlboard;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.Logger;

import com.team1678.frc2024.auto.AutoModeBase;
import com.team1678.frc2024.auto.AutoModeExecutor;
import com.team1678.frc2024.controlboard.ControlBoard;
import com.team1678.frc2024.subsystems.CoralPivotSolo;
import com.team1678.frc2024.subsystems.Drive;
import com.team1678.frc2024.subsystems.Drive.DriveControlState;
import com.team1678.frc2024.subsystems.vision.VisionDeviceManager;
import com.team6647.frc2025.Constants.CoralPivotConstantsSolo;
import com.team6647.frc2025.FieldLayout.CoralTarget;
import com.team6647.frc2025.auto.actions.AssistModeExecutor;
import com.team6647.frc2025.auto.modes.configuredQuals.goCenter;
import com.team6647.frc2025.auto.modes.configuredQuals.test1;
import com.team6647.frc2025.subsystems.AlgaeHolder;
import com.team6647.frc2025.subsystems.AlgaeRollers;
import com.team6647.frc2025.subsystems.MotorTest;
import com.team6647.frc2025.subsystems.Superstructure;
import com.team6647.frc2025.subsystems.coral_roller.CoralRoller;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriverControls {

	ControlBoard mControlBoard = ControlBoard.getInstance();

	Superstructure mSuperstructure = Superstructure.getInstance();
	Superstructure s = mSuperstructure;

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
	private AssistModeExecutor mAssistedActionsExecutor;
	private AutoModeBase coralPlacer;

	
	
	private MotorTest mMotorTest = MotorTest.getInstance();
	private AlgaeRollers mAlgaeRollers = AlgaeRollers.getInstance();
	private AlgaeHolder mAlgaeHolder = AlgaeHolder.getInstance();
	private CoralRoller mCoralRoller = CoralRoller.getInstance();
	//private Elevator mElevator = Elevator.getInstance();
	private CoralPivotSolo mCoralPivot = CoralPivotSolo.getInstance();



	/* TWO CONTROLLERS */

	//driver/operator
	@SuppressWarnings("unused")
	public void twoControllerMode() {
		
		if(mControlBoard.operator.aButton.wasActivated()){
			mMotorTest.setState(MotorTest.State.FORWARD);
		}
		if (mControlBoard.operator.aButton.wasReleased()) {
			mMotorTest.setState(MotorTest.State.IDLE);
			
		}
		if(mControlBoard.operator.bButton.wasActivated()){
			mMotorTest.setState(MotorTest.State.BACKWARD);
		}
		if (mControlBoard.operator.bButton.wasReleased()) {
			mMotorTest.setState(MotorTest.State.IDLE);
		}
			 
		
		if(mControlBoard.driver.yButton.wasActivated()){
			if(mAssistedActionsExecutor == null){
				mAssistedActionsExecutor = new AssistModeExecutor();
			}
			coralPlacer = new goCenter();
			mAssistedActionsExecutor.setAutoMode(coralPlacer);
			mAssistedActionsExecutor.start();
		}
		if (mControlBoard.driver.yButton.wasReleased()) {
			stopAssist();
		}
		//Angle angle = Angle.ofBaseUnits(Math.toDegrees(Math.atan2(mControlBoard.operator.getRightX(), -mControlBoard.operator.getRightY())),Radians);
		double angle = Angle.ofBaseUnits(Math.atan2(mControlBoard.operator.getRightX(), -mControlBoard.operator.getRightY()),Radians).in(Degrees);
		angle = angle;
		if(angle<0){
			angle = 360+angle;
		}
		if(angle>360){
			angle = angle-360;
		}
		Logger.recordOutput("/Coral/angle", angle);
		//angle = angle.plus(Angle.ofBaseUnits(90, Degree));
		if(Math.abs(mControlBoard.operator.getRightX())+Math.abs(mControlBoard.operator.getRightY())>0.5){
			for (int i = 0; i < s.angles.length; i++) {
				if (Math.abs(s.angles[i].angle-angle) < 30) {
					s.go6(mClimberJog);
					Logger.recordOutput("/Coral/Position", CoralTarget.values()[i].name());
					s.coralId = i;
					Logger.recordOutput("/Coral/id", s.coralId);
					stopAssist();
					if(angle-s.angles[i].angle>0){
						s.subCoralId = 1;
					}else{
						s.subCoralId = 0;
					}
					Logger.recordOutput("/Coral/subId", s.subCoralId);
					s.showAngle();
					break;
				}
			}
		}
		
		if(mControlBoard.operator.POV0.wasActivated()){
			s.level++;
			if (s.level>3){
				s.level = 3;
			}
			//coralPlacer.stop();
			s.showLevel();
		}
		if(mControlBoard.operator.POV180.wasActivated()){
			s.level--;
			if (s.level<0){
				s.level = 0;
			}
			//coralPlacer.stop();
			s.showLevel();
		}
		if(mControlBoard.operator.POV90.wasActivated()){
			s.coralStationPosition++;
			if (s.coralStationPosition>1){
				s.coralStationPosition = 1;
			}
			//coralPlacer.stop();
			s.showSource();
		}
		if(mControlBoard.operator.POV270.wasActivated()){
			s.coralStationPosition--;
			if (s.coralStationPosition<0){
				s.coralStationPosition = 0;
			}
			//coralPlacer.stop();
			s.showSource();
		}
		//if(mControlBoard.operator.leftTrigger.wasActivated()){
		//	mAlgaeRollers.setState(AlgaeRollers.State.INTAKING);
		//}
		//if(mControlBoard.operator.rightTrigger.wasActivated()){
		//	mAlgaeRollers.setState(AlgaeRollers.State.EXHAUST);
		//}
		//if(mControlBoard.operator.leftTrigger.wasReleased()||mControlBoard.operator.rightTrigger.wasReleased()){
		//	mAlgaeRollers.setState(AlgaeRollers.State.IDLE);
		//}

		if(mControlBoard.operator.leftTrigger.wasActivated()){
			mAlgaeHolder.setState(AlgaeHolder.State.RETRACTING);
		}
		if(mControlBoard.operator.rightTrigger.wasActivated()){
			mAlgaeHolder.setState(AlgaeHolder.State.DEPLOYING);
		}
		if(mControlBoard.operator.leftTrigger.wasReleased()||mControlBoard.operator.rightTrigger.wasReleased()){
			mAlgaeHolder.setState(AlgaeHolder.State.IDLE);
		}

		/*
		if(mControlBoard.operator.leftTrigger.wasActivated()){
			mCoralRoller.setState(CoralRoller.State.INTAKING);
		}
		if(mControlBoard.operator.rightTrigger.wasActivated()){
			mCoralRoller.setState(CoralRoller.State.OUTAKING);
		}
		if(mControlBoard.operator.leftTrigger.wasReleased()||mControlBoard.operator.rightTrigger.wasReleased()){
			mCoralRoller.setState(CoralRoller.State.IDLE);
		}
			 */

			 /*
		if(mControlBoard.operator.xButton.wasActivated()){
			//s.request(mElevator.L2Request());
			//mCoralPivot.setSetpointMotionMagic(100);
			//System.out.println(mCoralPivot.getSetpoint());
			mElevator.setSetpointMotionMagic(120);
		}

		if(mControlBoard.operator.yButton.wasActivated()){
			//s.request(mElevator.L2Request());
			//mCoralPivot.setSetpointMotionMagic(CoralPivotConstantsSolo.kHomePosition);
			//System.out.println(mCoralPivot.getSetpoint());
			mElevator.setSetpointMotionMagic(20);
		}
			 */
			

	}

	private void stopAssist(){
		if(mAssistedActionsExecutor != null){
			mAssistedActionsExecutor.stop();
			mAssistedActionsExecutor = null;
			mDrive.setControlState(DriveControlState.OPEN_LOOP);
			coralPlacer = null;
			//coralPlacer = null;
		}
	}
}