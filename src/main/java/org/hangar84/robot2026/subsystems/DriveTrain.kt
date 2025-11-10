package org.hangar84.robot2026.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase

abstract class Drivetrain : SubsystemBase() {
    abstract fun drive(xSpeed: Double, ySpeed: Double, rot: Double, fieldRelative: Boolean)
}