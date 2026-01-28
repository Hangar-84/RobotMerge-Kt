package org.hangar84.robot2026.io.sim

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import org.hangar84.robot2026.io.SwerveIO
import kotlin.math.PI
import kotlin.math.abs

class SimSwerveIO : SwerveIO {

    // internal simulated sensor state
    private val posMeters = doubleArrayOf(0.0, 0.0, 0.0, 0.0)
    private val velMps    = doubleArrayOf(0.0, 0.0, 0.0, 0.0)

    private val turnRad   = doubleArrayOf(0.0, 0.0, 0.0, 0.0)
    private val turnVelRadPerSec = doubleArrayOf(0.0, 0.0, 0.0, 0.0)

    // Simulate Electrical
    private val driveAppliedVolts = doubleArrayOf(0.0, 0.0, 0.0, 0.0)
    private val driveCurrentAmps  = doubleArrayOf(0.0, 0.0, 0.0, 0.0)
    private val turnAppliedVolts  = doubleArrayOf(0.0, 0.0, 0.0, 0.0)
    private val turnCurrentAmps   = doubleArrayOf(0.0, 0.0, 0.0, 0.0)

    // last commanded module states
    private val desired = arrayOf(
        SwerveModuleState(0.0, Rotation2d()),
        SwerveModuleState(0.0, Rotation2d()),
        SwerveModuleState(0.0, Rotation2d()),
        SwerveModuleState(0.0, Rotation2d())
    )

    override fun setModuleStates(fl: SwerveModuleState, fr: SwerveModuleState, rl: SwerveModuleState, rr: SwerveModuleState) {
        desired[0] = fl
        desired[1] = fr
        desired[2] = rl
        desired[3] = rr
    }

    override fun stop() {
        setModuleStates(
            SwerveModuleState(0.0, Rotation2d()),
            SwerveModuleState(0.0, Rotation2d()),
            SwerveModuleState(0.0, Rotation2d()),
            SwerveModuleState(0.0, Rotation2d())
        )
    }

    override fun simulationPeriodic(dtSeconds: Double) {
        val maxTurnRateRadPerSec = Math.toRadians(720.0)
        val nominalVoltage = 12.0

        for (i in 0..3) {
            val traction = 0.5
            velMps[i] = desired[i].speedMetersPerSecond * traction
            posMeters[i] += velMps[i] * dtSeconds

            // Simulate Electrical
            val drivePct = desired[i].speedMetersPerSecond / 4.5
            driveAppliedVolts[i] = drivePct * nominalVoltage
            driveCurrentAmps[i] = abs(drivePct * 40.0) + (if (abs(drivePct) > 0.1) 2.0 else 0.0)

            // turn angle moves toward target
            val target = desired[i].angle.radians
            val current = turnRad[i]
            val error = angleModulusRad(target - current)
            val delta = error.coerceIn(-maxTurnRateRadPerSec * dtSeconds, maxTurnRateRadPerSec * dtSeconds)

            val newAngle = current + delta
            turnVelRadPerSec[i] = if (dtSeconds > 0) delta / dtSeconds else 0.0
            turnRad[i] = newAngle

            // Simulate Turn Electricals Currently Not Implemented
            val turnPct = if (dtSeconds > 0) delta / (maxTurnRateRadPerSec * dtSeconds) else 0.0
            turnAppliedVolts[i] = turnPct * nominalVoltage
            turnCurrentAmps[i] = abs(turnPct * 20.0)
        }
    }

    override fun updateInputs(inputs: SwerveIO.Inputs) {
        fun fill(m: SwerveIO.ModuleInputs, i: Int) {
            m.drivePosMeters = posMeters[i]
            m.driveVelMps = velMps[i]
            m.turnPosRad = turnRad[i]

            // Fix: Populate ALL electrical fields from SwerveIO.ModuleInputs
            m.driveAppliedVolts = driveAppliedVolts[i]
            m.driveCurrentAmps = driveCurrentAmps[i]
        }

        fill(inputs.fl, 0)
        fill(inputs.fr, 1)
        fill(inputs.rl, 2)
        fill(inputs.rr, 3)
    }

    private fun angleModulusRad(radians: Double): Double {
        var x = radians
        while (x > PI) x -= 2.0 * PI
        while (x < -PI) x += 2.0 * PI
        return x
    }
}