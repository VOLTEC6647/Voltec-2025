package com.team6647.frc2025.auto.modes.configuredQuals;

import com.team1678.frc2024.auto.AutoModeBase;
import com.team1678.frc2024.auto.AutoModeEndedException;
import com.team1678.frc2024.auto.actions.ChoreoTrajectoryAction;
import com.team1678.frc2024.auto.actions.WaitAction;
import com.team6647.frc2025.subsystems.Elevator;
import com.team6647.frc2025.subsystems.Superstructure;
import com.team6647.frc2025.subsystems.Superstructure.Levels;
import com.team6647.frc2025.subsystems.coral_roller.CoralRoller;

public class Left1 extends AutoModeBase {
	private Superstructure s = Superstructure.getInstance();
	public Left1() {
		
	}

	// spotless:off
	@Override
	protected void routine() throws AutoModeEndedException {
		s.request(s.prepareLevel(Levels.LEVEL3));
		runAction(new ChoreoTrajectoryAction("S2Left1",true));
		runAction(new WaitAction(0.5));
		CoralRoller.getInstance().setState(CoralRoller.getInstance().OUTAKING);
		runAction(new WaitAction(1));
		//Elevator.getInstance().setWantHome(true);
		CoralRoller.getInstance().setState(CoralRoller.State.IDLE);


		System.out.println("Finished auto!");
	}
	// spotless:on
}