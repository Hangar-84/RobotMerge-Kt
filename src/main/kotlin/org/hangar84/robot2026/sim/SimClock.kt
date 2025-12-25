package org.hangar84.robot2026.sim

import edu.wpi.first.wpilibj.Timer

object SimClock {
    private var last = Timer.getFPGATimestamp()

    fun reset() {
        last = Timer.getFPGATimestamp()
    }

    fun dtSeconds(): Double {
        val now = Timer.getFPGATimestamp()
        val dt = now - last
        last = now
        return dt
    }
}