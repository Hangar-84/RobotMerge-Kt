package org.hangar84.robot2026.io.real

import edu.wpi.first.wpilibj.Compressor
import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj.Solenoid
import org.hangar84.robot2026.io.PneumaticsIO

class CtreTwoValvePnematicsIO(
    private val moduleId: Int,
    private val extendChannel: Int,
    private val retractChannel: Int,
) : PneumaticsIO {

    private val compressor = Compressor(moduleId, PneumaticsModuleType.CTREPCM)
    private val extendValve = Solenoid(moduleId, PneumaticsModuleType.CTREPCM, extendChannel)
    private val retractValve = Solenoid(moduleId, PneumaticsModuleType.CTREPCM, retractChannel)

    private var state: PneumaticsIO.ActuatorState = PneumaticsIO.ActuatorState.NEUTRAL

    init {
        compressor.enableDigital()
    }

    override fun updateInputs(inputs: PneumaticsIO.Inputs) {
        inputs.compressorEnabled = compressor.isEnabled
        inputs.extendSolenoidOn = extendValve.get()
        inputs.retractSolenoidOn = retractValve.get()
        inputs.actuatorState = state

        inputs.pressurePsi = null
    }

    override fun setActuatorState(newState: PneumaticsIO.ActuatorState) {
        state = newState

        // CRITICAL: never allow both true
        when (newState) {
            PneumaticsIO.ActuatorState.EXTEND -> {
                retractValve.set(false) // vent retract side
                extendValve.set(true)   // pressurize extend side
            }
            PneumaticsIO.ActuatorState.RETRACT -> {
                extendValve.set(false)  // vent extend side
                retractValve.set(true)  // pressurize retract side
            }
            PneumaticsIO.ActuatorState.NEUTRAL -> {
                extendValve.set(false)
                retractValve.set(false)
            }
        }
    }

    override fun setCompressorEnabled(enabled: Boolean) {
        if (enabled) compressor.enableDigital() else compressor.disable()
    }
}