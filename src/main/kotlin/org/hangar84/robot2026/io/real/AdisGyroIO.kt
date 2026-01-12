package org.hangar84.robot2026.io.real

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.ADIS16470_IMU
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis
import org.hangar84.robot2026.io.GyroIO
import org.hangar84.robot2026.io.GyroIO.Inputs

class AdisGyroIO: GyroIO {
    private val imu = ADIS16470_IMU()

    private val yawOffsetDeg = 90.0

    override fun updateInputs(inputs: Inputs) {
        val rawYawDeg = imu.getAngle(IMUAxis.kZ)
        inputs.yaw = Rotation2d.fromDegrees(rawYawDeg - yawOffsetDeg)
        inputs.yawRateDegPerSec = imu.getRate(IMUAxis.kZ)
    }

    override fun zeroYaw() {
        imu.reset()
    }
}