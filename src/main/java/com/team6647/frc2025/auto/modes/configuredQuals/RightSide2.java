package com.team6647.frc2025.auto.modes.configuredQuals;

import com.team1678.frc2024.auto.AutoModeBase;
import com.team1678.frc2024.auto.AutoModeEndedException;
import com.team1678.frc2024.auto.actions.ChoreoTrajectoryAction;
import com.team1678.frc2024.auto.actions.WaitAction;
import com.team6647.frc2025.subsystems.Elevator;
import com.team6647.frc2025.subsystems.Superstructure;
import com.team6647.frc2025.subsystems.Superstructure.Levels;
import com.team6647.frc2025.subsystems.coral_roller.CoralRoller;

public class RightSide2 extends AutoModeBase {
	private Superstructure s = Superstructure.getInstance();
	public RightSide2() {
		
	}

	// spotless:off
	@Override
	protected void routine() throws AutoModeEndedException {
		//Coral1
		runAction(new ChoreoTrajectoryAction("SimpleForward",true));
		s.request(s.prepareLevel(Levels.LEVEL3));
		CoralRoller.getInstance().setState(CoralRoller.getInstance().OUTAKING);
		new WaitAction(1);
		Elevator.getInstance().setWantHome(true);




		System.out.println("Finished auto!");
	}
	// spotless:on
}