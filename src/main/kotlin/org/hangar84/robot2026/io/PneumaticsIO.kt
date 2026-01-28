package org.hangar84.robot2026.io

interface PneumaticsIO {
    enum class State { EXTEND, RETRACT, NEUTRAL }

    data class Inputs(
        var aState: State = State.NEUTRAL,
        var bState: State = State.NEUTRAL,
        var aExtendOn: Boolean = false,
        var aRetractOn: Boolean = false,
        var bExtendOn: Boolean = false,
        var bRetractOn: Boolean = false,
    )

    fun updateInputs(inputs: Inputs)

    fun setA(state: State)
    fun setB(state: State)
}