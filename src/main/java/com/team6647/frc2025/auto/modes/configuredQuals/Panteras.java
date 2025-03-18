package com.team6647.frc2025.auto.modes.configuredQuals;

import com.team1678.frc2024.auto.AutoModeBase;
import com.team1678.frc2024.auto.AutoModeEndedException;
import com.team1678.frc2024.auto.actions.ChoreoTrajectoryAction;
import com.team1678.frc2024.auto.actions.LambdaAction;
import com.team1678.frc2024.auto.actions.WaitAction;
import com.team1678.frc2024.auto.actions.WaitForPrereqAction;
import com.team1678.frc2024.subsystems.CoralPivot;
import com.team1678.frc2024.subsystems.Drive;
import com.team1678.lib.requests.LambdaRequest;
import com.team1678.lib.requests.SequentialRequest;
import com.team1678.lib.requests.WaitForPrereqRequest;
import com.team1678.lib.requests.WaitRequest;
import com.team6647.frc2025.subsystems.Elevator;
import com.team6647.frc2025.subsystems.Superstructure;
import com.team6647.frc2025.subsystems.Superstructure.Levels;
import com.team6647.frc2025.subsystems.coral_roller.CoralRoller;

import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Panteras extends AutoModeBase {
	private Drive d = Drive.getInstance();
	private Superstructure s = Superstructure.getInstance();
	public Panteras() {
		
	}

	// spotless:off
	@Override
	protected void routine() throws AutoModeEndedException {
		CoralRoller.getInstance().setState(CoralRoller.State.CONSTANT);
		s.request(s.prepareLevel(Levels.LEVEL2));
		runAction(new WaitAction(6));
		runAction(new ChoreoTrajectoryAction("S4Center1",true));
		s.request(s.prepareLevel(Levels.LEVEL3));
		runAction(new WaitAction(1.5));
		CoralRoller.getInstance().setState(CoralRoller.getInstance().OUTAKING);
		runAction(new WaitAction(3));
		CoralRoller.getInstance().setState(CoralRoller.State.IDLE);

		System.out.println("Finished auto!");
	}
	// spotless:on
}