package org.hangar84.robot2026.io

import edu.wpi.first.wpilibj.util.Color

interface LedIO {
    data class Inputs(
        var connected: Boolean = false,
        var lastError: String = ""
    )

    fun updateInputs(inputs: Inputs) {}

    fun connect() {}

    fun setSolid(color: Color) {}

    fun setBreathe(color: Color, periodMs: Int) {}

    fun setChase(color: Color, speedMs: Int, reverse: Boolean = false) {}

    fun setStrobe(color: Color, speedMs: Int) {}

    fun setOff() {}
}