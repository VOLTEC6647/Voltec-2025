package com.team6647.frc2025.auto.actions;

import com.team1678.frc2024.auto.actions.Action;
import com.team1678.frc2024.subsystems.Drive;
import com.team6647.frc2025.auto.modes.configuredQuals.PreparePutCoral;

public class WaitForEnterPathGeneratedAction implements Action {

	public WaitForEnterPathGeneratedAction() {
	}

	@Override
	public boolean isFinished() {
		return PreparePutCoral.enterCoral != null;
	}

	@Override
	public void start() {
	}

	@Override
	public void update() {}

	@Override
	public void done() {}
}
