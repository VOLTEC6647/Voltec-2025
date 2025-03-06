package com.team6647.frc2025.controlboard;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.Logger;

import com.team1678.frc2024.auto.AutoModeBase;
import com.team1678.frc2024.auto.AutoModeExecutor;
import com.team1678.frc2024.controlboard.ControlBoard;
import com.team1678.frc2024.subsystems.AlgaeT;
import com.team1678.frc2024.subsystems.Climber;
import com.team1678.frc2024.subsystems.CoralPivot;
import com.team1678.frc2024.subsystems.Drive;
import com.team1678.frc2024.subsystems.Drive.DriveControlState;
import com.team1678.frc2024.subsystems.vision.VisionDeviceManager;
import com.team1678.lib.requests.LambdaRequest;
import com.team1678.lib.requests.ParallelRequest;
import com.team1678.lib.requests.SequentialRequest;
import com.team1678.lib.requests.WaitForPrereqRequest;
import com.team1678.lib.requests.WaitRequest;
import com.team1678.lib.util.NearestAngleFinder;
import com.team254.lib.geometry.Rotation2d;
import com.team6647.frc2025.FieldLayout;
import com.team6647.frc2025.FieldLayout.CoralTarget;
import com.team6647.frc2025.auto.actions.AssistModeExecutor;
import com.team6647.frc2025.auto.modes.configuredQuals.goCenter;
import com.team6647.frc2025.auto.modes.configuredQuals.putCoral;
import com.team6647.frc2025.auto.modes.configuredQuals.test1;

import com.team6647.frc2025.subsystems.Elevator;
import com.team6647.frc2025.subsystems.MotorTest;
import com.team6647.frc2025.subsystems.Superstructure;
import com.team6647.frc2025.subsystems.Superstructure.Levels;
import com.team6647.frc2025.subsystems.algae_roller.AlgaeRoller;
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
	private AlgaeRoller mAlgaeRollers = AlgaeRoller.getInstance();
	private AlgaeT mAlgaeHolder = AlgaeT.getInstance();
	private CoralRoller mCoralRoller = CoralRoller.getInstance();
	private Elevator mElevator = Elevator.getInstance();
	private CoralPivot mCoralPivot = CoralPivot.getInstance();
	private Climber mClimber = Climber.getInstance();

	public boolean autoPose = true;




	/* TWO CONTROLLERS */

	//driver/operator
	@SuppressWarnings("unused")
	public void twoControllerMode() {
		
		if(mControlBoard.operator.aButton.wasActivated()){
			mSuperstructure.request(
				new SequentialRequest(
					mSuperstructure.prepareLevel(s.currentLevel),
					new LambdaRequest(
						()->{
							//mCoralRoller.setState(CoralRoller.State.OUTAKING);
						}
					)
				)
			);
		}

		if(mControlBoard.operator.leftBumper.wasActivated()){
			mSuperstructure.request(
				new SequentialRequest(
					mSuperstructure.prepareLevel(Levels.ALGAEING1),
					new LambdaRequest(
						()->{
							mCoralRoller.setState(CoralRoller.State.OUTAKING3);
						}
					)
				)
			);
		}

		if(mControlBoard.operator.rightBumper.wasActivated()){
			mSuperstructure.request(
				new SequentialRequest(
					mSuperstructure.prepareLevel(Levels.ALGAEING2),
					new LambdaRequest(
						()->{
							mCoralRoller.setState(CoralRoller.State.OUTAKING3);
						}
					)
				)
			);
		}

		if(mControlBoard.operator.aButton.wasReleased()){
			mSuperstructure.request(
				mSuperstructure.softHome()
			);
		}
		if(mControlBoard.operator.leftBumper.wasReleased()){
			mSuperstructure.request(
				mSuperstructure.softHome()
			);
			mCoralRoller.setState(CoralRoller.State.IDLE);
		}
		if(mControlBoard.operator.rightBumper.wasReleased()){
			mSuperstructure.request(
				mSuperstructure.softHome()
			);
			mCoralRoller.setState(CoralRoller.State.IDLE);
		}
		if(mControlBoard.operator.bButton.wasActivated()){
			//mElevator.setWantHome(true);
			mSuperstructure.request(
			mCoralPivot.setPivotRequest(CoralPivot.kIntakingAngle)
			);
		}
		
		if (mControlBoard.operator.yButton.wasActivated()) {
			mCoralRoller.setState(CoralRoller.State.INTAKING);
		}
		if (mControlBoard.operator.yButton.wasReleased()) {
			mCoralRoller.setState(CoralRoller.State.IDLE);
		}
		if (mControlBoard.driver.xButton.wasActivated()) {
			mCoralRoller.setState(mCoralRoller.OUTAKING);
		}
		if (mControlBoard.driver.bButton.wasActivated()) {
			mCoralPivot.setSetpointMotionMagic(CoralPivot.kLevel4Angle+15);
		}
		if (mControlBoard.driver.xButton.wasReleased()) {
			mCoralRoller.setState(CoralRoller.State.IDLE);
		}
		if (mControlBoard.operator.xButton.wasActivated()) {
			mCoralRoller.setState(mCoralRoller.OUTAKING);
		}
		if (mControlBoard.operator.xButton.wasReleased()) {
			mCoralRoller.setState(CoralRoller.State.IDLE);
		}
		
		

		if (mControlBoard.driver.backButton.wasActivated()) {
			mDrive.zeroGyro(0);
		}
		/*
		if(mControlBoard.operator.bButton.wasActivated()){
			mMotorTest.setState(MotorTest.State.BACKWARD);
		}
		if (mControlBoard.operator.bButton.wasReleased()) {
			mMotorTest.setState(MotorTest.State.IDLE);
		}
			 */
			 
		
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

		if(mControlBoard.operator.startButton.wasActivated()){
			if(mAssistedActionsExecutor == null){
				mAssistedActionsExecutor = new AssistModeExecutor();
			}
			coralPlacer = new putCoral();
			mAssistedActionsExecutor.setAutoMode(coralPlacer);
			mAssistedActionsExecutor.start();
		}
		if (mControlBoard.operator.startButton.wasReleased()) {
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
		////Logger.recordOutput("/Coral/angle", angle);
		
		//angle = angle.plus(Angle.ofBaseUnits(90, Degree));
		if(Math.abs(mControlBoard.operator.getRightX())+Math.abs(mControlBoard.operator.getRightY())>0.5){
			for (int i = 0; i < s.angles.length; i++) {
				if (Math.abs(s.angles[i].angle-angle) < 30) {
					s.go6(mClimberJog);
					////Logger.recordOutput("/Coral/Position", CoralTarget.values()[i].name());
					s.coralId = i;
					////Logger.recordOutput("/Coral/id", s.coralId);
					stopAssist();
					if(angle-s.angles[i].angle>0){
						s.subCoralId = 1;
					}else{
						s.subCoralId = 0;
					}
					////Logger.recordOutput("/Coral/subId", s.subCoralId);
					s.showAngle();
					break;
				}
			}
		}
		
		if(mControlBoard.operator.POV0.wasActivated()){
			s.setLevel(s.level+1);
			if (s.level>3){
				s.setLevel(4);
			}
			//coralPlacer.stop();
			s.showLevel();
		}
		if(mControlBoard.operator.POV180.wasActivated()){
			s.setLevel(s.level-1);
			if (s.level<1){
				s.setLevel(1);
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

		if(mControlBoard.driver.rightTrigger.wasActivated()){
			mDrive.stabilizeHeading(Rotation2d.fromDegrees(125));
		}
		if(mControlBoard.driver.rightTrigger.wasReleased()){
			mDrive.setControlState(DriveControlState.OPEN_LOOP);
		}
		if(mControlBoard.driver.leftTrigger.wasActivated()){
			mDrive.stabilizeHeading(Rotation2d.fromDegrees(-48.4));
		}
		if(mControlBoard.driver.leftTrigger.wasReleased()){
			mDrive.setControlState(DriveControlState.OPEN_LOOP);
		}
		if(mControlBoard.driver.aButton.wasActivated()){
			//Rotation2d coralRotation = FieldLayout.getCoralTargetPos(s.angles[s.coralId]).algae.getRotation();
			//mDrive.stabilizeHeading(coralRotation);

			Rotation2d coralRotation = Rotation2d.fromDegrees(NearestAngleFinder.findNearestAngle(s.angles, mDrive.getHeading().getDegrees()));
			mDrive.stabilizeHeading(coralRotation);
		}
		if(mControlBoard.driver.aButton.wasReleased()){
			mDrive.setControlState(DriveControlState.OPEN_LOOP);
		}
		/*
		if(mControlBoard.operator.leftTrigger.wasActivated()){
			Rotation2d coralRotation = FieldLayout.getCoralTargetPos(s.angles[s.coralId]).algae.getRotation();
			mDrive.stabilizeHeading(coralRotation);
		}
		if(mControlBoard.operator.leftTrigger.wasReleased()){
			mDrive.setControlState(DriveControlState.OPEN_LOOP);
		}
		*/

		if(mControlBoard.operator.leftTrigger.wasActivated()){
			s.request(
				new SequentialRequest(
				new LambdaRequest(()->{mAlgaeRollers.setState(AlgaeRoller.State.INTAKINGFAST);}),
				mAlgaeHolder.setPositionRequest(AlgaeT.kIntakingAngle),
				new WaitRequest(1),
				new LambdaRequest(()->{mAlgaeRollers.setState(AlgaeRoller.State.INTAKING);}),
				new WaitForPrereqRequest(()->!mControlBoard.operator.leftTrigger.isBeingPressed()),
				new LambdaRequest(()->{mAlgaeRollers.setState(AlgaeRoller.State.IDLE);})

			)
			);
			
		}

		if(mControlBoard.operator.rightTrigger.wasActivated()){
			
			s.request(
			new SequentialRequest(
				new LambdaRequest(()->{mAlgaeRollers.setState(AlgaeRoller.State.IDLE);}),

				mAlgaeHolder.setPositionRequest(AlgaeT.kHoldingAngle),
				new WaitRequest(0.92),
				//new WaitForPrereqRequest(()->mControlBoard.operator.getRightTriggerAxis()<0.5),

				new LambdaRequest(()->{mAlgaeHolder.setSetpointMotionMagic(AlgaeT.kIdleAngle);}),

				new WaitRequest(2),
				new LambdaRequest(()->{mAlgaeRollers.setState(AlgaeRoller.State.OUTAKING);}),

				new WaitRequest(7),

				new LambdaRequest(()->{mAlgaeRollers.setState(AlgaeRoller.State.IDLE);})
			));
		}

		if(mControlBoard.operator.backButton.wasActivated()){
			mClimber.setSetpointMotionMagic(Climber.kPreparing);
		}
		if(mControlBoard.operator.backButton.wasReleased()){
			mClimber.setSetpointMotionMagic(Climber.kVertical);
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