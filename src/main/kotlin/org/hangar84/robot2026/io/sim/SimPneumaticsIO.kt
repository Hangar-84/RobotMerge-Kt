package org.hangar84.robot2026.io.sim

import org.hangar84.robot2026.io.PneumaticsIO

class SimPneumaticsIO : PneumaticsIO {

    private var aState = PneumaticsIO.State.NEUTRAL
    private var bState = PneumaticsIO.State.NEUTRAL

    override fun updateInputs(inputs: PneumaticsIO.Inputs) {
        inputs.aState = aState
        inputs.bState = bState

        inputs.aExtendOn = (aState == PneumaticsIO.State.EXTEND)
        inputs.aRetractOn = (aState == PneumaticsIO.State.RETRACT)

        inputs.bExtendOn = (bState == PneumaticsIO.State.EXTEND)
        inputs.bRetractOn = (bState == PneumaticsIO.State.RETRACT)
    }

    override fun setA(state: PneumaticsIO.State) {
        aState = state
    }

    override fun setB(state: PneumaticsIO.State) {
        bState = state
    }
}