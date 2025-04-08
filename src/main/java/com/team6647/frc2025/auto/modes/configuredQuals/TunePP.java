package com.team6647.frc2025.auto.modes.configuredQuals;

import org.littletonrobotics.frc2025.subsystems.drive.Drive;

import com.team1678.frc2024.auto.AutoModeBase;
import com.team1678.frc2024.auto.AutoModeEndedException;
import com.team1678.frc2024.auto.actions.ChoreoTrajectoryAction;
import com.team1678.frc2024.auto.actions.LambdaAction;
import com.team1678.frc2024.auto.actions.PathplannerTrajectoryAction;
import com.team1678.frc2024.auto.actions.ResetOdometryAction;
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

public class TunePP extends AutoModeBase {
	//private Drive d = Drive.getInstance();
	private Superstructure s = Superstructure.getInstance();
	public TunePP() {
		
	}

	// spotless:off
	@Override
	protected void routine() throws AutoModeEndedException {
		runAction(new ResetOdometryAction("Tuning"));
		runAction(new WaitAction(2));
		runAction(new PathplannerTrajectoryAction("Tuning"));


		System.out.println("Finished auto!");
	}
	// spotless:on
}