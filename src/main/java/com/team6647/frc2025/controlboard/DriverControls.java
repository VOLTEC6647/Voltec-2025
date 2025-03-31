package com.team6647.frc2025.controlboard;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.frc2025.RobotState;
import org.littletonrobotics.frc2025.subsystems.drive.Drive;
import com.team1678.frc2024.auto.AutoModeBase;
import com.team1678.frc2024.controlboard.ControlBoard;
import com.team1678.frc2024.subsystems.AlgaeT;
import com.team1678.frc2024.subsystems.Climber;
import com.team1678.frc2024.subsystems.CoralPivot;
import com.team1678.lib.requests.LambdaRequest;
import com.team1678.lib.requests.Request;
import com.team1678.lib.requests.SequentialRequest;
import com.team1678.lib.requests.WaitForPrereqRequest;
import com.team1678.lib.requests.WaitRequest;
import com.team1678.lib.util.NearestCoralFinder;
import com.team254.lib.geometry.Rotation2d;
import com.team6647.frc2025.Robot;
import com.team6647.frc2025.auto.actions.AssistModeExecutor;
import com.team6647.frc2025.auto.modes.configuredQuals.intakeAuto;
import com.team6647.frc2025.auto.modes.configuredQuals.PreparePutCoral;
import com.team6647.frc2025.subsystems.Elevator;
import com.team6647.frc2025.subsystems.MotorTest;
import com.team6647.frc2025.subsystems.Superstructure;
import com.team6647.frc2025.subsystems.Superstructure.Levels;
import com.team6647.frc2025.subsystems.algae_roller.AlgaeRoller;
import com.team6647.frc2025.subsystems.coral_roller.CoralRoller;
import com.team6647.frc2025.subsystems.leds.LEDSubsystem;
import com.team6647.frc2025.subsystems.vision.VisionSubsystem;

import edu.wpi.first.units.measure.Angle;

public class DriverControls {

	ControlBoard mControlBoard = ControlBoard.getInstance();

	Superstructure mSuperstructure = Superstructure.getInstance();
	Superstructure s = mSuperstructure;

	Drive mDrive = Drive.getInstance();
	RobotState state = RobotState.getInstance();
	LEDSubsystem mleds = LEDSubsystem.getInstance();
	VisionSubsystem mVision = VisionSubsystem.getInstance();

	/* ONE CONTROLLER */

	public void oneControllerMode() {
		/*
		if (mControlBoard.driver.rightTrigger.isBeingPressed()) {
			mDrive.overrideHeading(true);
		} else {
			mDrive.overrideHeading(false);
		}
			 */
	}

	private boolean mClimberJog = false;
	private AssistModeExecutor mAssistedActionsExecutor;
	
	
	private MotorTest mMotorTest = MotorTest.getInstance();
	private AlgaeRoller mAlgaeRollers = AlgaeRoller.getInstance();
	private AlgaeT mAlgaeHolder = AlgaeT.getInstance();
	private CoralRoller mCoralRoller = CoralRoller.getInstance();
	private Elevator mElevator = Elevator.getInstance();
	private CoralPivot mCoralPivot = CoralPivot.getInstance();
	private Climber mClimber = Climber.getInstance();

	public boolean assisting = false;
	




	/* TWO CONTROLLERS */

	//driver/operator
	@SuppressWarnings("unused")
	public void twoControllerMode() {
		//if(assisting){
		//	startAssist(new putCoral());
		//}
		if(mControlBoard.operator.startButton.wasActivated()){
			assisting = !assisting;
		}
		
		if(mControlBoard.operator.aButton.wasActivated()){
			if(assisting){
				//new PathplannerAlignAction(null);
				startAssist(new PreparePutCoral());

			}else{
				mSuperstructure.request(
				new SequentialRequest(
					mSuperstructure.prepareLevel(s.currentLevel)
				)
			);
			}
			
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
			stopAssist();
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
			if(assisting){
				startAssist(new intakeAuto());

			}else{
				
				mSuperstructure.request(
					new SequentialRequest(
						mCoralPivot.setPivotRequest(CoralPivot.kIntakingAngle),
						new LambdaRequest(()->{
							mleds.solidGreen();
						})
					)	
				);
			}
			
		}

		if(mControlBoard.operator.bButton.wasReleased()){
			stopAssist();
		}
		
		if (mControlBoard.operator.yButton.wasActivated()) {
			s.request(
				new SequentialRequest(
					new LambdaRequest(
						()->{mCoralRoller.setState(CoralRoller.State.INTAKING);}
					),
					new WaitRequest(0.2),
					new WaitForPrereqRequest(()->mCoralRoller.getBeamBreak()),
					new LambdaRequest(
						()->{mCoralRoller.setState(CoralRoller.State.IDLE);}
					),
					new LambdaRequest(()->{
						mleds.solidBlue();
					})
				)
				);
		}
		if (mControlBoard.operator.yButton.wasReleased()) {
			mCoralRoller.setState(CoralRoller.State.IDLE);
		}
		if (mControlBoard.driver.xButton.wasActivated()) {
			mCoralRoller.setState(mCoralRoller.OUTAKING);
		}
		if (mControlBoard.driver.bButton.wasActivated()) {
			mCoralPivot.setSetpointMotionMagic(CoralPivot.kLevel4Angle+30);
		}
		if (mControlBoard.driver.xButton.wasReleased()) {
			mCoralRoller.setState(CoralRoller.State.IDLE);
		}
		if (mControlBoard.operator.xButton.wasActivated()) {
			//mCoralRoller.setState(mCoralRoller.OUTAKING);
			s.addRequestToQueue(s.prepareLevel(s.currentLevel));
			
		}
		if (mControlBoard.operator.xButton.wasReleased()) {
			if(!s.placing_coral){
				mSuperstructure.request(
				mSuperstructure.softHome()
			);
			}
		}
		
		

		if (mControlBoard.driver.backButton.wasActivated()) {
			state.resetGyro(edu.wpi.first.math.geometry.Rotation2d.fromDegrees(0));
		}
		/*
		if(mControlBoard.operator.bButton.wasActivated()){
			mMotorTest.setState(MotorTest.State.BACKWARD);
		}
		if (mControlBoard.operator.bButton.wasReleased()) {
			mMotorTest.setState(MotorTest.State.IDLE);
		}
			 */

		if(mControlBoard.operator.startButton.wasActivated()){
			startAssist(new PreparePutCoral());
		}
		if (mControlBoard.operator.startButton.wasReleased()) {
			stopAssist();
		}
		//Angle angle = Angle.ofBaseUnits(Math.toDegrees(Math.atan2(mControlBoard.operator.getRightX(), -mControlBoard.operator.getRightY())),Radians);
		double angle = Angle.ofBaseUnits(Math.atan2(mControlBoard.operator.getLeftX(), -mControlBoard.operator.getLeftY()),Radians).in(Degrees);
		if(angle<0){
			angle = 360+angle;
		}
		if(angle>360){
			angle = angle-360;
		}
		////Logger.recordOutput("/Coral/angle", angle);
		
		//angle = angle.plus(Angle.ofBaseUnits(90, Degree));
		if(Math.abs(mControlBoard.operator.getLeftX())+Math.abs(mControlBoard.operator.getLeftY())>0.5){
			for (int i = 0; i < s.angles.length; i++) {
				if (Math.abs(s.angles[i].angle-angle) < 30) {
					s.coralId = i;
					stopAssist();
					if(angle-s.angles[i].angle>0){
						//s.subCoralId = 1;
					}else{
						//s.subCoralId = 0;
					}
					//s.showAngle();
					break;
				}
			}
		}

		if(mControlBoard.operator.getRightX()>0.5){
			s.subCoralId = 0;
			//s.showAngle();
		}
		if(mControlBoard.operator.getRightX()<-0.5){
			s.subCoralId = 1;
			//s.showAngle();
		}
		
		if(mControlBoard.operator.POV0.wasActivated()){
			s.setLevel(s.level+1);
			if (s.level>3){
				s.setLevel(4);
			}
			//coralPlacer.stop();
		}
		if(mControlBoard.operator.POV180.wasActivated()){
			s.setLevel(s.level-1);
			if (s.level<1){
				s.setLevel(1);
			}
			//coralPlacer.stop();
		}
		if(mControlBoard.operator.POV90.wasActivated()){
			//s.coralStationPosition++;
			if (s.coralStationPosition>1){
				s.coralStationPosition = 1;
			}
			//coralPlacer.stop();
			s.showSource();
		}
		if(mControlBoard.operator.POV270.wasActivated()){
			//s.coralStationPosition--;
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

		if(mControlBoard.driver.startButton.wasActivated()){
			state.resetGyro(mVision.getBestPose().getRotation());
			mVision.
		}
		if(mControlBoard.driver.startButton.wasReleased()){
			
		}

		
		if(mControlBoard.driver.POV270.wasActivated()){
			s.subCoralId = 1;
			double joystickDifference = 0;
			if(Robot.is_red_alliance){
				joystickDifference = 180;
			}
			s.coralId = NearestCoralFinder.getCoralIdFromTarget( NearestCoralFinder.findNearestCoral(s.angles, state.getHeading().plus(Rotation2d.fromDegrees(joystickDifference).toLegacy()).getDegrees()));
			s.coralId = 4;
			
			startAssist(new PreparePutCoral());
		}
		if(mControlBoard.driver.POV270.wasReleased()){
			stopAssist();
		}

		if(mControlBoard.driver.POV90.wasActivated()){
			s.subCoralId = 0;
			double joystickDifference = 0;
			if(Robot.is_red_alliance){
				joystickDifference = 180;
			}
			s.coralId = NearestCoralFinder.getCoralIdFromTarget( NearestCoralFinder.findNearestCoral(s.angles, state.getHeading().plus(Rotation2d.fromDegrees(joystickDifference).toLegacy()).getDegrees()));
			s.coralId = 4;
			
			startAssist(new PreparePutCoral());
		}
		if(mControlBoard.driver.POV90.wasReleased()){
			stopAssist();
		}
			 

		/*
		if(mControlBoard.driver.POV0.wasActivated()){
			s.sourcePose = s.sourcePose1;
			s.showSource();
			startAssist(new intakePP());
		}
		if(mControlBoard.driver.POV0.wasReleased()){
			mDrive.setControlState(DriveControlState.OPEN_LOOP);
			stopAssist();
		}

		if(mControlBoard.driver.POV180.wasActivated()){
			s.sourcePose = s.sourcePose2;
			s.showSource();
			startAssist(new intakePP());
		}
		if(mControlBoard.driver.POV180.wasReleased()){
			mDrive.setControlState(DriveControlState.OPEN_LOOP);
			stopAssist();
		}

		*/

		if(mControlBoard.driver.leftTrigger.wasActivated()){
		}
		if(mControlBoard.driver.leftTrigger.wasReleased()){
		}
		if(mControlBoard.driver.aButton.wasActivated()){
			//Rotation2d coralRotation = FieldLayout.getCoralTargetPos(s.angles[s.coralId]).algae.getRotation();
			//mDrive.stabilizeHeading(coralRotation);
			if(assisting){
				//new PathplannerAlignAction(null);
				//placing_coral = true;
				startAssist(new PreparePutCoral());
			}
			

			//Rotation2d coralRotation = Rotation2d.fromDegrees(NearestAngleFinder.findNearestAngle(s.angles, mDrive.getHeading().getDegrees()));
			//mDrive.stabilizeHeading(coralRotation);
		}
		if(mControlBoard.driver.aButton.wasReleased()){
			stopAssist();
			s.placing_coral = false;
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
				new WaitForPrereqRequest(()->!mControlBoard.operator.leftTrigger.isBeingPressed()),
				new LambdaRequest(()->{mAlgaeRollers.setState(AlgaeRoller.State.INTAKING);})
				//new WaitForPrereqRequest(()->!mControlBoard.operator.leftTrigger.isBeingPressed()),
				//new LambdaRequest(()->{mAlgaeRollers.setState(AlgaeRoller.State.IDLE);})

			)
			);
			
		}

		if(mControlBoard.operator.rightTrigger.wasActivated()){
			
			s.request(
			new SequentialRequest(
				new LambdaRequest(()->{mAlgaeRollers.setState(AlgaeRoller.State.IDLE);}),

				mAlgaeHolder.setPositionRequest(AlgaeT.kHoldingAngle),
				new WaitForPrereqRequest(()->!mControlBoard.operator.rightTrigger.isBeingPressed()),
				//new WaitForPrereqRequest(()->mControlBoard.operator.getRightTriggerAxis()<0.5),

				new LambdaRequest(()->{mAlgaeHolder.setSetpointMotionMagic(AlgaeT.kIdleAngle);}),

				new WaitRequest(2),
				new LambdaRequest(()->{mAlgaeRollers.setState(AlgaeRoller.State.OUTAKING);}),

				new WaitRequest(4),

				new LambdaRequest(()->{mAlgaeRollers.setState(AlgaeRoller.State.IDLE);})
			));
		}

		if(mControlBoard.operator.backButton.wasActivated()){
			mClimber.setSetpointMotionMagic(Climber.kPreparing);
		}
		if(mControlBoard.operator.backButton.wasReleased()){
			mClimber.setSetpointMotionMagic(Climber.kVertical);
			mCoralPivot.setSetpointMotionMagic(CoralPivot.kLevel2Angle);
			mAlgaeHolder.setSetpointMotionMagic(AlgaeT.kIntakingAngle);
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
			//coralPlacer = null;
			//coralPlacer = null;
		}
	}

	private void startAssist(AutoModeBase mode){
		if(mAssistedActionsExecutor == null){
			mAssistedActionsExecutor = new AssistModeExecutor();
		}
		mAssistedActionsExecutor.setAutoMode(mode);
		mAssistedActionsExecutor.start();
	}

	
	public Request assistRequest(AutoModeBase mode) {
		return new Request() {

			@Override
			public void act() {
				startAssist(mode);
			}

			@Override
			public boolean isFinished() {
				//return Util.epsilonEquals(mDrive.getPose(), mDrive.getMotionPlanner().isDone(), 0.2);
				return false;
			}
			//cleanuppp
		};
	}
		 
}