package org.hangar84.robot2026.io.sim

import org.hangar84.robot2026.io.PneumaticsIO

class SimPneumaticsIO : PneumaticsIO {
    private var compressorEnabled = true
    private var state = PneumaticsIO.ActuatorState.NEUTRAL

    override fun updateInputs(inputs: PneumaticsIO.Inputs) {
        inputs.compressorEnabled = compressorEnabled
        inputs.actuatorState = state
        inputs.extendSolenoidOn = (state == PneumaticsIO.ActuatorState.EXTEND)
        inputs.retractSolenoidOn = (state == PneumaticsIO.ActuatorState.RETRACT)
        inputs.pressurePsi = 110.0
    }

    override fun setActuatorState(newState: PneumaticsIO.ActuatorState) {
        this.state = newState
    }

    override fun setCompressorEnabled(enabled: Boolean) {
        compressorEnabled = enabled
    }
}