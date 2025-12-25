package org.hangar84.robot2026

import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import org.hangar84.robot2026.commands.driveCommand
import org.hangar84.robot2026.constants.RobotType
import org.hangar84.robot2026.sim.SimClock.dtSeconds
import org.hangar84.robot2026.sim.SimField.publishOnce
import org.hangar84.robot2026.sim.SimField.setRobotPose
import org.hangar84.robot2026.sim.SimHooks
import org.hangar84.robot2026.sim.SimRobotTypeSelector
import org.hangar84.robot2026.sim.SimState.isSim
import org.hangar84.robot2026.subsystems.Drivetrain
import org.hangar84.robot2026.subsystems.LauncherSubsystem
import org.hangar84.robot2026.subsystems.MecanumDriveSubsystem
import org.hangar84.robot2026.subsystems.SwerveDriveSubsystem
import org.hangar84.robot2026.telemetry.SimTelemetry
import org.hangar84.robot2026.telemetry.Telemetry


object RobotContainer {
    private const val DRIVEDEADBAND = 0.08
    private val controller: CommandXboxController = CommandXboxController(0)
    private val buttonA = DigitalInput(19)

    private fun readBootSelector(): RobotType {
        return if (!buttonA.get()) {
            RobotType.SWERVE
        } else {
            RobotType.MECANUM
        }
    }

    val robotType: RobotType =
        if (RobotBase.isSimulation()) {
            SimRobotTypeSelector.selected()
        } else {
            readBootSelector()
        }

    // The robot's subsystems
    private val drivetrain: Drivetrain = when (robotType) {
        RobotType.SWERVE -> SwerveDriveSubsystem()
        RobotType.MECANUM -> MecanumDriveSubsystem()
    }
    // The driver's controller

    private val autoChooser: SendableChooser<Command> = drivetrain.buildAutoChooser()



    val autonomousCommand: Command
        get() = autoChooser.selected ?: InstantCommand()


    init {
        SmartDashboard.putString("Selected Robot Type", robotType.name)
        SmartDashboard.putData("Auto Chooser", autoChooser)

        SmartDashboard.putBoolean("DS/Enabled", DriverStation.isEnabled())
        SmartDashboard.putBoolean("DS/Auto", DriverStation.isAutonomous())
        SmartDashboard.putBoolean("DS/Teleop", DriverStation.isTeleop())

        SmartDashboard.putNumber("DS/MatchTime", DriverStation.getMatchTime())

        setupDashboard()
        SimHooks.init()
        configureBindings()

    }
    private fun configureBindings() {
        /*val x: Double = MathUtil.applyDeadband(-controller.leftX, DRIVEDEADBAND)
        val y: Double = MathUtil.applyDeadband(-controller.leftY, DRIVEDEADBAND)
        val rot: Double = MathUtil.applyDeadband(controller.rightX, DRIVEDEADBAND)*/

        if (robotType == RobotType.MECANUM) {
            drivetrain.defaultCommand =  driveCommand(
                drivetrain,
                { MathUtil.applyDeadband(controller.leftX, DRIVEDEADBAND) },
                { MathUtil.applyDeadband(-controller.leftY, DRIVEDEADBAND) },
                { MathUtil.applyDeadband(controller.rightX, DRIVEDEADBAND) },
                { false }
            )
        } else if (robotType == RobotType.SWERVE){
            val maxV = drivetrain.maxLinearSpeedMps
            val maxW = drivetrain.maxAngularSpeedRadPerSec
            drivetrain.defaultCommand =
                driveCommand(
                drivetrain,
                { MathUtil.applyDeadband(-controller.leftY, DRIVEDEADBAND) * maxV},
                { MathUtil.applyDeadband(-controller.leftX, DRIVEDEADBAND) * maxV },
                { MathUtil.applyDeadband(controller.rightX, DRIVEDEADBAND) * maxW},
                { true }
            )
            val swerve = drivetrain as SwerveDriveSubsystem
            controller.leftBumper().whileTrue(swerve.PARK_COMMAND)
        }

        controller.leftTrigger().onTrue(LauncherSubsystem.INTAKE_COMMAND).onFalse(LauncherSubsystem.STOP_COMMAND)
        controller.rightTrigger().onTrue(LauncherSubsystem.LAUNCH_COMMAND).onFalse(LauncherSubsystem.STOP_COMMAND)
    }

    // -- Simulation --
    fun setupDashboard() {
        if (!isSim) return
        publishOnce("Field")
    }

    fun periodic() {
        val pose = drivetrain.getPose()
        setRobotPose(pose)
        publishGyroWidgets()
    }

    private var lastYawDeg = 0.0
    private var lastYawTime = Timer.getFPGATimestamp()

    private fun publishGyroWidgets() {
        val yaw = drivetrain.getHeading()
        val now = Timer.getFPGATimestamp()
        val dt = now - lastYawTime

        val yawRateDegPerSec =
            if (dt > 1e-6) (yaw.degrees - lastYawDeg) / dt else 0.0
        if (isSim)
            SimTelemetry.gyro("Gyro", yaw, yawRateDegPerSec)
        else if (!isSim)
            Telemetry.gyro("Gyro", yaw, yawRateDegPerSec)
        SmartDashboard.putNumber("Gyro/PoseYawDeg", drivetrain.getPose().rotation.degrees)

        lastYawDeg = yaw.degrees
        lastYawTime = now
    }


    fun simulationPeriodic() {
        val dt = dtSeconds()
        drivetrain.simulationPeriodic(dt)
    }
}