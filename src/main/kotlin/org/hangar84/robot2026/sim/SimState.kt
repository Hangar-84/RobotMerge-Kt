package org.hangar84.robot2026.sim

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.RobotBase

object SimState {
    val isSim get() = RobotBase.isSimulation()

    // -- Common States among the two DriveSubsystems
    var groundTruthPose: Pose2d = Pose2d()
    var estimatedPose = Pose2d()
    var yaw = Rotation2d()

    val fl = SimSensors.EncoderTruth()
    val fr = SimSensors.EncoderTruth()
    val rl = SimSensors.EncoderTruth()
    val rr = SimSensors.EncoderTruth()


    // Wheel distances (meters)
    var simFL = 0.0
    var simFR = 0.0
    var simRL = 0.0
    var simRR = 0.0

    // Wheel velocities (m/s)
    var simFLVel = 0.0
    var simFRVel = 0.0
    var simRLVel = 0.0
    var simRRVel = 0.0

    // Optional: swerve azimuth angles (deg) if you want to show them
    var simFLAngleDeg = 0.0
    var simFRAngleDeg = 0.0
    var simRLAngleDeg = 0.0
    var simRRAngleDeg = 0.0
}