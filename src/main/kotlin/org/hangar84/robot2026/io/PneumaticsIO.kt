package org.hangar84.robot2026.io

interface PneumaticsIO {
    enum class ActuatorState {EXTEND, RETRACT, NEUTRAL}

    data class Inputs(
        var compressorEnabled: Boolean = false,
        var actuatorState: ActuatorState = ActuatorState.NEUTRAL,
        var extendSolenoidOn: Boolean = false,
        var retractSolenoidOn: Boolean = false,
        var pressurePsi: Double? = null,
    )

    fun updateInputs(inputs: Inputs)

    fun setActuatorState(state: ActuatorState)

    fun setCompressorEnabled(enabled: Boolean) {}
}