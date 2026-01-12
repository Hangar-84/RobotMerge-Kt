package org.hangar84.robot2026.subsystems

import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.hangar84.robot2026.io.PneumaticsIO
import org.hangar84.robot2026.telemetry.TelemetryRouter

class PneumaticsSubsystem(
    private val io: PneumaticsIO
) : SubsystemBase() {

    private val isSim = RobotBase.isSimulation()

    private val inputs = PneumaticsIO.Inputs()

    override fun periodic() {


        io.updateInputs(inputs)

        TelemetryRouter.pneumatics(inputs.compressorEnabled,
            inputs.extendSolenoidOn,
            inputs.retractSolenoidOn,
            inputs.actuatorState.name,
            null)
    }

    fun extend() = io.setActuatorState(PneumaticsIO.ActuatorState.EXTEND)
    fun retract() = io.setActuatorState(PneumaticsIO.ActuatorState.RETRACT)
    fun neutral() = io.setActuatorState(PneumaticsIO.ActuatorState.NEUTRAL)

    fun extendCommand(): Command = Commands.runOnce({ extend() }, this)
    fun retractCommand(): Command = Commands.runOnce({ retract() }, this)
    fun neutralCommand(): Command = Commands.runOnce({ neutral() }, this)

    fun toggleCommand(): Command = Commands.runOnce({
        val next = when (inputs.actuatorState) {
            PneumaticsIO.ActuatorState.EXTEND -> PneumaticsIO.ActuatorState.RETRACT
            PneumaticsIO.ActuatorState.RETRACT -> PneumaticsIO.ActuatorState.EXTEND
            PneumaticsIO.ActuatorState.NEUTRAL -> PneumaticsIO.ActuatorState.EXTEND
        }
        io.setActuatorState(next)
    }, this)
}