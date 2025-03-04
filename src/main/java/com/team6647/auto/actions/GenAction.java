package com.team6647.auto.actions;

import com.team1678.frc2024.auto.actions.Action;

public class GenAction implements Action {

	public interface VoidInterace {
		void f();
	}

	VoidInterace mF;

	public GenAction(VoidInterace f) {
		this.mF = f;
	}

	@Override
	public void start() {
		mF.f();
	}

	@Override
	public void update() {}

	@Override
	public boolean isFinished() {
		return true;
	}

	@Override
	public void done() {}
}
