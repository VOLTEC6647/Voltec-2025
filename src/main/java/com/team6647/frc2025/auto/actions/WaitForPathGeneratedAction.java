package com.team6647.frc2025.auto.actions;

import java.util.function.Supplier;

import com.team1678.frc2024.auto.actions.Action;
import com.team6647.frc2025.auto.modes.configuredQuals.PreparePutCoral;

public class WaitForPathGeneratedAction implements Action {

	Supplier<Object> pointer;
	public WaitForPathGeneratedAction(Supplier<Object> pointer) {
		this.pointer = pointer;
	}

	@Override
	public boolean isFinished() {
		return pointer.get() != null;
	}

	@Override
	public void start() {}

	@Override
	public void update() {}

	@Override
	public void done() {}
}
