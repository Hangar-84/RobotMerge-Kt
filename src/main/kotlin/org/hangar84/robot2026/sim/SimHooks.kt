package org.hangar84.robot2026.sim

import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

object SimHooks {
    fun init() {
        if (!RobotBase.isSimulation()) return
        // Put any sim-only dashboard widgets here
        SmartDashboard.putBoolean("Sim/DIO9", false) // your fake boot switch
        // Ensure drivetrain chooser exists:
        SimRobotTypeSelector.publishOnce()
        SimClock.reset()
        SimField.publishOnce("Field")
    }
}