package org.hangar84.robot2026.io.interfaces.drivebaseio



interface MecanumIO {

    data class MotorHealth(
        val driveFault: Boolean = false,
        val turnFault: Boolean = false,
        val driveStickyFault: Boolean = false,
        val turnStickyFault: Boolean = false,

        val driveConnected: Boolean = true,
        val absEncoderConnected: Boolean = true,

        val driveTempC: Double = 0.0,
        val turnTempC: Double = 0.0,
        val driveCurrentA: Double = 0.0,
        val turnCurrentA: Double = 0.0,
    )
    data class Inputs(
        var flPosMeters: Double = 0.0,
        var frPosMeters: Double = 0.0,
        var rlPosMeters: Double = 0.0,
        var rrPosMeters: Double = 0.0,

        var flVelMps: Double = 0.0,
        var frVelMps: Double = 0.0,
        var rlVelMps: Double = 0.0,
        var rrVelMps: Double = 0.0,

        var flCurrentAmps: Double = 0.0,
        var frCurrentAmps: Double = 0.0,
        var rlCurrentAmps: Double = 0.0,
        var rrCurrentAmps: Double = 0.0,

        var flAppliedVolts: Double = 0.0,
        var frAppliedVolts: Double = 0.0,
        var rlAppliedVolts: Double = 0.0,
        var rrAppliedVolts: Double = 0.0,

        var flDriveFaulted: Boolean = false,
        var frDriveFaulted: Boolean = false,
        var rlDriveFaulted: Boolean = false,
        var rrDriveFaulted: Boolean = false,

        var flDriveTempC: Double = 0.0,
        var frDriveTempC: Double = 0.0,
        var rlDriveTempC: Double = 0.0,
        var rrDriveTempC: Double = 0.0,

        val health: MotorHealth = MotorHealth()
    )

    fun updateInputs(inputs: Inputs)

    fun setWheelSpeeds(
        fl: Double,
        fr: Double,
        rl: Double,
        rr: Double
    )

    fun stop()

    fun simulationPeriodic(dtSeconds: Double) {}
}