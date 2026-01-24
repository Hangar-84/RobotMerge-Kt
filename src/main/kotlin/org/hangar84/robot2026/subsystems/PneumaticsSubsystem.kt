package org.hangar84.robot2026.subsystems

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.hangar84.robot2026.io.PneumaticsIO

class PneumaticsSubsystem(private val io: PneumaticsIO) : SubsystemBase() {

    private val inputs = PneumaticsIO.Inputs()

    override fun periodic() {
        io.updateInputs(inputs)
        // telemetry here using inputs.aState, inputs.bState, inputs.aExtendOn, etc.
    }

    // ----- Actuator A -----
    fun extendA() = io.setA(PneumaticsIO.State.EXTEND)
    fun retractA() = io.setA(PneumaticsIO.State.RETRACT)
    fun neutralA() = io.setA(PneumaticsIO.State.NEUTRAL)

    fun toggleA() {
        val next =
            if (inputs.aState == PneumaticsIO.State.EXTEND) PneumaticsIO.State.RETRACT
            else PneumaticsIO.State.EXTEND
        io.setA(next)
    }

    // ----- Actuator B -----
    fun extendB() = io.setB(PneumaticsIO.State.EXTEND)
    fun retractB() = io.setB(PneumaticsIO.State.RETRACT)
    fun neutralB() = io.setB(PneumaticsIO.State.NEUTRAL)

    fun toggleB() {
        val next =
            if (inputs.bState == PneumaticsIO.State.EXTEND) PneumaticsIO.State.RETRACT
            else PneumaticsIO.State.EXTEND
        io.setB(next)
    }

    // ----- Both -----
    fun setBoth(state: PneumaticsIO.State) {
        io.setA(state)
        io.setB(state)
    }

    fun extendBoth() = setBoth(PneumaticsIO.State.EXTEND)
    fun retractBoth() = setBoth(PneumaticsIO.State.RETRACT)
    fun neutralBoth() = setBoth(PneumaticsIO.State.NEUTRAL)

    fun toggleBoth() {
        val bothExtended =
            inputs.aState == PneumaticsIO.State.EXTEND &&
                    inputs.bState == PneumaticsIO.State.EXTEND

        setBoth(if (bothExtended) PneumaticsIO.State.RETRACT else PneumaticsIO.State.EXTEND)
    }

    // ----- Commands (nice for RobotContainer bindings) -----
    fun extendACommand(): Command = Commands.runOnce({ extendA() }, this)
    fun retractACommand(): Command = Commands.runOnce({ retractA() }, this)
    fun toggleACommand(): Command = Commands.runOnce({ toggleA() }, this)

    fun extendBCommand(): Command = Commands.runOnce({ extendB() }, this)
    fun retractBCommand(): Command = Commands.runOnce({ retractB() }, this)
    fun toggleBCommand(): Command = Commands.runOnce({ toggleB() }, this)

    fun extendBothCommand(): Command = Commands.runOnce({ extendBoth() }, this)
    fun retractBothCommand(): Command = Commands.runOnce({ retractBoth() }, this)
    fun toggleBothCommand(): Command = Commands.runOnce({ toggleBoth() }, this)
}