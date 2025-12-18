package org.hangar84.robot2026

/*import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.DriverStation
import org.hangar84.robot2026.constants.Constants
import edu.wpi.first.wpilibj2.command.CommandScheduler*/
import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import org.hangar84.robot2026.constants.RobotType
import org.hangar84.robot2026.subsystems.*
import org.hangar84.robot2026.commands.DriveCommand
import org.hangar84.robot2026.subsystems.SwerveDriveSubsystem.MAX_SPEED
import edu.wpi.first.units.Units.MetersPerSecond as MPS
import edu.wpi.first.units.Units.RadiansPerSecond as RPS
import org.hangar84.robot2026.subsystems.SwerveDriveSubsystem.MAX_ANGULAR_SPEED



object RobotContainer {

    private const val DRIVEDEADBAND = 0.5
    private val controller: CommandXboxController = CommandXboxController(0)
    private val buttonA = DigitalInput(9)

    val robotType: RobotType =
        if (buttonA.get()) {
        RobotType.SWERVE
    } else {
        RobotType.MECANUM
    }

    // The robot's subsystems
    private val drivetrain: Drivetrain = when (robotType) {
        RobotType.SWERVE -> SwerveDriveSubsystem
        RobotType.MECANUM -> MecanumDriveSubsystem
    }
    // The driver's controller

    private val autoChooser: SendableChooser<Command> = drivetrain.buildAutoChooser()



    val autonomousCommand: Command
        get() = autoChooser.selected ?: InstantCommand()

    init {
        SmartDashboard.putString("Selected Robot Type", robotType.name)
        SmartDashboard.putData("Auto Chooser", autoChooser)

        configureBindings()

    }
    private fun configureBindings() {
        /*val x: Double = MathUtil.applyDeadband(-controller.leftX, DRIVEDEADBAND)
        val y: Double = MathUtil.applyDeadband(-controller.leftY, DRIVEDEADBAND)
        val rot: Double = MathUtil.applyDeadband(controller.rightX, DRIVEDEADBAND)*/

        if (robotType == RobotType.MECANUM) {
            drivetrain.defaultCommand = drivetrain.run { DriveCommand(
                drivetrain,
                { MathUtil.applyDeadband(controller.leftX, DRIVEDEADBAND) },
                { MathUtil.applyDeadband(-controller.leftY, DRIVEDEADBAND) },
                { MathUtil.applyDeadband(controller.rightX, DRIVEDEADBAND) },
                { false }
            )}
        } else {
            drivetrain.defaultCommand =
                drivetrain.run { DriveCommand(
                drivetrain,
                { MathUtil.applyDeadband(-controller.leftX, DRIVEDEADBAND) * MAX_SPEED.`in`(MPS)},
                { MathUtil.applyDeadband(-controller.leftY, DRIVEDEADBAND) * MAX_SPEED.`in`(MPS) },
                { MathUtil.applyDeadband(controller.rightX, DRIVEDEADBAND) * MAX_ANGULAR_SPEED.`in`(RPS)},
                { true }
            )}
            controller.leftBumper().whileTrue(SwerveDriveSubsystem.PARK_COMMAND)
        }

        controller.leftBumper().onTrue(LauncherSubsystem.INTAKE_COMMAND).onFalse(LauncherSubsystem.STOP_COMMAND)
        controller.rightBumper().onTrue(LauncherSubsystem.LAUNCH_COMMAND).onFalse(LauncherSubsystem.STOP_COMMAND)
    }
}