package org.hangar84.robot2026.io.sim

import org.hangar84.robot2026.io.PneumaticsIO

class SimPneumaticsIO : PneumaticsIO {

    private var LeftState = PneumaticsIO.State.NEUTRAL
    private var RightState = PneumaticsIO.State.NEUTRAL

    private var compressorActive = true

    override fun updateInputs(inputs: PneumaticsIO.Inputs) {
        inputs.Left = LeftState
        inputs.Right = RightState

        inputs.CompressorEnabled = compressorActive

        inputs.Left_Solenoid_Extend = (LeftState == PneumaticsIO.State.EXTEND)
        inputs.Left_Solenoid_Retract = (LeftState != PneumaticsIO.State.EXTEND)

        inputs.Right_Solenoid_Extend = (RightState == PneumaticsIO.State.EXTEND)
        inputs.Right_Solenoid_Retract = (RightState != PneumaticsIO.State.EXTEND)
    }

    override fun Left(state: PneumaticsIO.State) {
        LeftState = state
    }

    override fun Right(state: PneumaticsIO.State) {
        RightState = state
    }

    override fun setCompressor(enabled: Boolean) {
        compressorActive = enabled
    }
}