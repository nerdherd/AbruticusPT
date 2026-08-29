// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.logging;

public interface Reportable {
	/**
	 * Controls how much data is logged.
	 */
	public enum LOG_LEVEL {
		ALL(1),
		MEDIUM(1),
		MINIMAL(1),
		NONE(1);

		/**
		 * How many loops to skip before logging the values
		 * associated with LOG_LEVEL.
		 */
		public final int throttle;
		LOG_LEVEL(int throttle) { this.throttle = throttle; }
	}

	public void initializeLogging();
}