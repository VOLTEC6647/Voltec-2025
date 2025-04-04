package com.team6647.frc2025.auto.modes.configuredQuals;

import com.team1678.frc2024.auto.AutoModeBase;
import com.team1678.frc2024.auto.AutoModeEndedException;
import com.team1678.frc2024.auto.actions.ChoreoTrajectoryAction;
import com.team1678.frc2024.auto.actions.LambdaAction;
import com.team1678.frc2024.auto.actions.WaitAction;
import com.team1678.frc2024.auto.actions.WaitForPrereqAction;
import com.team1678.frc2024.subsystems.CoralPivot;
import com.team1678.lib.requests.LambdaRequest;
import com.team1678.lib.requests.SequentialRequest;
import com.team1678.lib.requests.WaitForPrereqRequest;
import com.team1678.lib.requests.WaitRequest;
import com.team6647.frc2025.subsystems.Elevator;
import com.team6647.frc2025.subsystems.Superstructure;
import com.team6647.frc2025.subsystems.Superstructure.Levels;
import com.team6647.frc2025.subsystems.coral_roller.CoralRoller;

import edu.wpi.first.wpilibj2.command.WaitCommand;

public class S3Right extends AutoModeBase {
	private Superstructure s = Superstructure.getInstance();
	public S3Right() {
		
	}

	// spotless:off
	@Override
	protected void routine() throws AutoModeEndedException {
		runAction(new WaitAction(0.3));
		s.request(s.prepareLevel(Levels.LEVEL2));
		runAction(new ChoreoTrajectoryAction("S3Right1",true));
		runAction(new WaitAction(0.5));
		CoralRoller.getInstance().setState(CoralRoller.getInstance().OUTAKING);
		runAction(new WaitAction(0.6));
		//Elevator.getInstance().setWantHome(true);

		runAction(new ChoreoTrajectoryAction("S3Right2",false,0.3));
		s.request(s.prepareLevel(Levels.ALGAEING2));
		runAction(new WaitAction(0.7));
		
		Elevator.getInstance().setSetpointMotionMagic(Elevator.kL2Height);
		s.request(new SequentialRequest(
			new WaitRequest(1),
			new LambdaRequest(()->{CoralRoller.getInstance().setState(CoralRoller.State.INTAKING);}),
			new LambdaRequest(()->{CoralPivot.getInstance().setSetpointMotionMagic(CoralPivot.kIntakingAngle);})
			)
		);
		
		runAction(new ChoreoTrajectoryAction("S3Right3"));
		runAction(new WaitForPrereqAction(()->CoralRoller.getInstance().getBeamBreak()));
		CoralRoller.getInstance().setState(CoralRoller.State.IDLE);

		s.currentLevel = Levels.LEVEL3;
		runAction(new ChoreoTrajectoryAction("S3Right4"));
		s.request(s.prepareLevel(Levels.LEVEL3));

		runAction(new WaitAction(1));
		CoralRoller.getInstance().setState(CoralRoller.getInstance().OUTAKING);
		runAction(new WaitAction(0.6));




		System.out.println("Finished auto!");
	}
	// spotless:on
}