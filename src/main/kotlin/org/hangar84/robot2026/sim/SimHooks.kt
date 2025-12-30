package org.hangar84.robot2026.sim

import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

object SimHooks {
    fun init() {
        if (!RobotBase.isSimulation()) return
        // Put any sim-only dashboard widgets here
        // Ensure drivetrain chooser exists:
        SimClock.reset()
        SimField.publishOnce("Field")
    }
}