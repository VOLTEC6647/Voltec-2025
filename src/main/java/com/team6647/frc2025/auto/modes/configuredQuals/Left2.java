package com.team6647.frc2025.auto.modes.configuredQuals;

import com.team1678.frc2024.auto.AutoModeBase;
import com.team1678.frc2024.auto.AutoModeEndedException;
import com.team1678.frc2024.auto.actions.ChoreoTrajectoryAction;
import com.team1678.frc2024.auto.actions.LambdaAction;
import com.team1678.frc2024.auto.actions.WaitAction;
import com.team1678.frc2024.subsystems.CoralPivot;
import com.team6647.frc2025.subsystems.Elevator;
import com.team6647.frc2025.subsystems.Superstructure;
import com.team6647.frc2025.subsystems.Superstructure.Levels;
import com.team6647.frc2025.subsystems.coral_roller.CoralRoller;

public class Left2 extends AutoModeBase {
	private Superstructure s = Superstructure.getInstance();
	public Left2() {
		
	}

	// spotless:off
	@Override
	protected void routine() throws AutoModeEndedException {
		runAction(new ChoreoTrajectoryAction("S2Left1",true));
		s.request(s.prepareLevel(Levels.LEVEL3));
		runAction(new WaitAction(1.5));
		CoralRoller.getInstance().setState(CoralRoller.getInstance().OUTAKING);
		runAction(new WaitAction(1));
		Elevator.getInstance().setWantHome(true);
		CoralRoller.getInstance().setState(CoralRoller.State.IDLE);
		runAction(new WaitAction(1));

		runAction(new ChoreoTrajectoryAction("S2Left2",false));
		new LambdaAction(()->{
			CoralPivot.getInstance().setSetpointMotionMagic(CoralPivot.kIntakingAngle);
		});
		runAction(new WaitAction(0.5));
		CoralRoller.getInstance().setState(CoralRoller.State.INTAKING);
		runAction(new WaitAction(1.5));
		CoralRoller.getInstance().setState(CoralRoller.State.IDLE);

		runAction(new ChoreoTrajectoryAction("S2Left3",false));
		s.request(s.prepareLevel(Levels.LEVEL3));
		runAction(new WaitAction(1.5));
		CoralRoller.getInstance().setState(CoralRoller.getInstance().OUTAKING);
		runAction(new WaitAction(1));
		Elevator.getInstance().setWantHome(true);
		CoralRoller.getInstance().setState(CoralRoller.State.IDLE);

		System.out.println("Finished auto!");
	}
	// spotless:on
}