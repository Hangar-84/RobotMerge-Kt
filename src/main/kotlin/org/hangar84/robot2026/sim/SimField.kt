package org.hangar84.robot2026.sim


import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard

object SimField {
    private val field = Field2d()
    private var published = false

    fun publishOnce(key: String = "Field") {
        if (published) return
        SmartDashboard.putData(key, field)
        published = true
    }

    fun setRobotPose(pose: Pose2d) {
        field.robotPose = pose
    }
}