package com.team6647.frc2025.auto.modes.configuredQuals;

import com.team1678.frc2024.auto.AutoModeBase;
import com.team1678.frc2024.auto.AutoModeEndedException;
import com.team1678.frc2024.auto.actions.ChoreoTrajectoryAction;
import com.team1678.frc2024.auto.actions.WaitAction;
import com.team6647.frc2025.subsystems.Elevator;
import com.team6647.frc2025.subsystems.Superstructure;
import com.team6647.frc2025.subsystems.Superstructure.Levels;
import com.team6647.frc2025.subsystems.coral_roller.CoralRoller;

import edu.wpi.first.wpilibj2.command.WaitCommand;

public class CTest extends AutoModeBase {
	private Superstructure s = Superstructure.getInstance();
	public CTest() {
		
	}

	// spotless:off
	@Override
	protected void routine() throws AutoModeEndedException {
		runAction(new ChoreoTrajectoryAction("CTest",true));
		System.out.println("Finished auto!");
	}
	// spotless:on
}