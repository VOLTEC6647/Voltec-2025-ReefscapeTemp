package com.team6647.frc2025.auto.actions;

import com.team1678.frc2024.auto.AutoModeBase;
import com.team1678.frc2024.loops.CrashTrackingRunnable;

/**
 * This class selects, runs, and stops (if necessary) a specified autonomous
 * mode.
 */
public class AssistModeExecutor {
	private AutoModeBase m_auto_mode;
	private Thread m_thread = null;
	public boolean m_enabled = false;

	public void setAutoMode(AutoModeBase new_auto_mode) {
		m_auto_mode = new_auto_mode;
	}

	public AutoModeBase getAutoMode() {
		return m_auto_mode;
	}

	public void start() {
		if (m_thread == null) {
			m_thread = new Thread(new CrashTrackingRunnable() {
				@Override
				public void runCrashTracked() {
					if (m_auto_mode != null) {
						m_auto_mode.run();
					}
				}
			});
			m_enabled = true;
			m_thread.start();
		}
	}

	public void stop() {
		if (m_auto_mode != null) {
			m_auto_mode.stop();
		}

		m_thread = null;
		m_enabled = false;
	}
}
