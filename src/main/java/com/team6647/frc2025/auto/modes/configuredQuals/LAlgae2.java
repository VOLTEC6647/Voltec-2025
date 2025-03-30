package com.team6647.frc2025.auto.modes.configuredQuals;

import com.team1678.frc2024.auto.AutoModeBase;
import com.team1678.frc2024.auto.AutoModeEndedException;
import com.team1678.frc2024.auto.actions.ChoreoTrajectoryAction;
import com.team1678.frc2024.auto.actions.WaitAction;
import com.team6647.frc2025.subsystems.Elevator;
import com.team6647.frc2025.subsystems.Superstructure;
import com.team6647.frc2025.subsystems.Superstructure.Levels;
import com.team6647.frc2025.subsystems.coral_roller.CoralRoller;

public class LAlgae2 extends AutoModeBase {
	private Superstructure s = Superstructure.getInstance();
	public LAlgae2() {
		
	}

	// spotless:off
	@Override
	protected void routine() throws AutoModeEndedException {
		runAction(new ChoreoTrajectoryAction("S2Left1",true));
		s.request(s.prepareLevel(Levels.LEVEL3));
		runAction(new WaitAction(1.4));
		CoralRoller.getInstance().setState(CoralRoller.getInstance().OUTAKING);
		runAction(new WaitAction(1));
		Elevator.getInstance().setWantHome(true);
		CoralRoller.getInstance().setState(CoralRoller.State.IDLE);
		runAction(new WaitAction(1));

		runAction(new ChoreoTrajectoryAction("LAlgae1",false));
		s.request(s.prepareLevel(Levels.ALGAEING2));
		runAction(new WaitAction(1.7));
		Elevator.getInstance().setWantHome(true);

		runAction(new ChoreoTrajectoryAction("LAlgae2",false));
		s.request(s.prepareLevel(Levels.ALGAEING2));
		runAction(new WaitAction(1.7));
		Elevator.getInstance().setWantHome(true);

		System.out.println("Finished auto!");
	}
	// spotless:on
}