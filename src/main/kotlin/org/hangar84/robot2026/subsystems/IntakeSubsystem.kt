package org.hangar84.robot2026.subsystems

import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.hangar84.robot2026.io.IntakeIO
import org.hangar84.robot2026.telemetry.TelemetryRouter

class IntakeSubsystem(val io: IntakeIO) : SubsystemBase() {

    private val inputs = IntakeIO.Inputs()

    private val isSim = RobotBase.isSimulation()

    // - Commands -
    internal val INTAKE_COMMAND
        get() = Commands.startEnd(
            { io.setPercent(-1.0) },
            { io.stop() },
            this
        )

    override fun periodic() {
        TelemetryRouter.setBase(
            if (isSim) {
                "Intake/Sim"
            } else {
                "Intake"
            }
        )

        io.updateInputs(inputs)

        // Optional telemetry (uses only inputs)
        TelemetryRouter.num("Intake/leftOutput", inputs.leftAppliedOutput)
        TelemetryRouter.num("Intake/leftVelocity", inputs.leftVelocityRpm)
        TelemetryRouter.num("Intake/leftCurrent", inputs.leftCurrentAmps)
    }

    override fun simulationPeriodic() {
        io.simulationPeriodic(0.02)
    }
}