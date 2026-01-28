package org.hangar84.robot2026.subsystems

import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.hangar84.robot2026.io.LauncherIO
import org.hangar84.robot2026.telemetry.TelemetryRouter

class LauncherSubsystem(val io: LauncherIO) : SubsystemBase() {

    private val inputs = LauncherIO.Inputs()

    private val isSim = RobotBase.isSimulation()

    // - Commands -

    // Only used for controller input.
    internal val LAUNCH_COMMAND
        get() = Commands.startEnd(
            { io.setPercent(1.0) },
            { io.stop() },
            this
        )

    // Only used for Pathplanenr Named Commands.
    fun pulseCommand(seconds: Double): Command =
        LAUNCH_COMMAND.withTimeout(seconds)

    override fun periodic() {
        TelemetryRouter.setBase(
            if (isSim) {
                "Launcher/Sim"
            } else {
                "Launcher"
            }
        )

        io.updateInputs(inputs)

        // Optional telemetry (uses only inputs)
        TelemetryRouter.launcher(
            inputs.leftAppliedOutput, inputs.rightAppliedOutput,
            inputs.leftVelocityRpm, inputs.rightVelocityRpm,
            inputs.leftCurrentAmps, inputs.rightCurrentAmps
        )
    }

    override fun simulationPeriodic() {
        io.simulationPeriodic(0.02)
    }
}