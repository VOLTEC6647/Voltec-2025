package com.team6647.frc2025.auto.modes.configuredQuals;

import org.littletonrobotics.frc2025.commands.DriveToPose;

import com.team1678.frc2024.auto.AutoModeBase;
import com.team1678.frc2024.auto.AutoModeEndedException;
import com.team1678.frc2024.auto.actions.CommandAction;
import com.team1678.frc2024.subsystems.CoralPivot;

import com.team1678.lib.requests.IntakeCoralRequest;
import com.team4678.CommandSwerveDrivetrain;
import com.team6647.frc2025.subsystems.Superstructure;
import com.team6647.frc2025.subsystems.coral_roller.CoralRoller;

public class intakeAuto extends AutoModeBase {
	private Superstructure s = Superstructure.getInstance();

	public intakeAuto() {
	}

	// spotless:off
	@Override
	protected void routine() throws AutoModeEndedException {
		//CoralRoller.getInstance().setState(CoralRoller.State.INTAKING);
		CoralPivot.getInstance().setSetpointMotionMagic(CoralPivot.kIntakingAngle);
		CoralRoller mCoralRoller = CoralRoller.getInstance();
		s.request(
			IntakeCoralRequest.get()
		);
		runAction(new CommandAction(new DriveToPose(CommandSwerveDrivetrain.getInstance(), s.sourcePose.toLegacy())));

		System.out.println("Finished auto!");
	}
	// spotless:on
}