package org.hangar84.robot2026.commands

import edu.wpi.first.wpilibj2.command.Command
import org.hangar84.robot2026.subsystems.Drivetrain
import java.util.function.DoubleSupplier

class DriveCommand(
    private val drivetrain: Drivetrain,
    private val xSpeed: DoubleSupplier,
    private val ySpeed: DoubleSupplier,
    private val rot: DoubleSupplier
): Command() {
    init {
        addRequirements(drivetrain as edu.wpi.first.wpilibj2.command.SubsystemBase)
    }

    override fun execute() {
        drivetrain.drive(xSpeed.asDouble, ySpeed.asDouble, rot.asDouble, true)
    }
}