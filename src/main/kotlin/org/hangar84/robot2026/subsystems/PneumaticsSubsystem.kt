package org.hangar84.robot2026.subsystems

import edu.wpi.first.networktables.GenericEntry
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.hangar84.robot2026.io.PneumaticsIO
import org.hangar84.robot2026.telemetry.TelemetryRouter

class PneumaticsSubsystem(private val io: PneumaticsIO) : SubsystemBase() {

    private val inputs = PneumaticsIO.Inputs()
    private val PneumaticsTable = NetworkTableInstance.getDefault().getTable("Mechanism/Pneumatics")
    private val Extend_Left: GenericEntry = PneumaticsTable.getTopic("Extend Left").getGenericEntry()
    private val Extend_Right: GenericEntry = PneumaticsTable.getTopic("Extend Right").getGenericEntry()
    private val Enable_Compressor: GenericEntry = PneumaticsTable.getTopic("Enable Compressor").getGenericEntry()

    private val isLeftSelectedEntry: GenericEntry = PneumaticsTable.getTopic("Selection/IsLeftSelected").getGenericEntry()
    private val isRightSelectedEntry: GenericEntry = PneumaticsTable.getTopic("Selection/IsRightSelected").getGenericEntry()
    private val isBothSelectedEntry: GenericEntry = PneumaticsTable.getTopic("Selection/IsBothSelected").getGenericEntry()
    private val selectionNameEntry: GenericEntry = PneumaticsTable.getTopic("Selection/Selection").getGenericEntry()
    private val systemEnabledEntry: GenericEntry = PneumaticsTable.getTopic("Selection/System Enabled").getGenericEntry()

    private var pneumaticsEnabled = true
    enum class Selection { LEFT, RIGHT, BOTH }
    private var currentSelection = Selection.BOTH
    private var systemEnabled = true

    init {
            Extend_Left.setBoolean(false)
            Trigger { Extend_Left.getBoolean(false) }
                .whileTrue(extendACommand())
                .onFalse(retractACommand()) // Correct

            Extend_Right.setBoolean(false)
            Trigger { Extend_Right.getBoolean(false) }
                .whileTrue(extendBCommand())
                .onFalse(retractBCommand())

            Enable_Compressor.setBoolean(true) // Default to ON
            Trigger { Enable_Compressor.getBoolean(true) }
                .onTrue(enableCompressorCommand())
                .onFalse(disableCompressorCommand())

        selectionNameEntry.setString(currentSelection.name)
        systemEnabledEntry.setBoolean(systemEnabled)
    }
    override fun periodic() {
        io.updateInputs(inputs)

        isLeftSelectedEntry.setBoolean(currentSelection == Selection.LEFT)
        isRightSelectedEntry.setBoolean(currentSelection == Selection.RIGHT)
        isBothSelectedEntry.setBoolean(currentSelection == Selection.BOTH)
        selectionNameEntry.setString(currentSelection.name)
        systemEnabledEntry.setBoolean(systemEnabled)

        TelemetryRouter.Phneumatics(
            inputs.CompressorEnabled, // Pass the table reference here
            inputs.Left_Solenoid_Extend,
            inputs.Left_Solenoid_Retract,
            inputs.Right_Solenoid_Extend,
            inputs.Right_Solenoid_Retract,
        )
    }

    fun setCompressor(enabled: Boolean) = io.setCompressor(enabled)

    fun enableCompressorCommand(): Command = Commands.runOnce({ setCompressor(true) }, this)
    fun disableCompressorCommand(): Command = Commands.runOnce({ setCompressor(false) }, this)

    fun setSystemEnabled(enabled: Boolean) {
        systemEnabled = enabled
        systemEnabledEntry.setBoolean(enabled)
    }

    fun setSelection(selection: Selection) {
        currentSelection = selection
        selectionNameEntry.setString(selection.name)
    }

    fun cycleSelection() {
        val next = when (currentSelection) {
            Selection.LEFT -> Selection.RIGHT
            Selection.RIGHT -> Selection.BOTH
            Selection.BOTH -> Selection.LEFT
        }
        setSelection(next)
    }

    fun smartToggle() {
        if (!systemEnabled) return
        when(currentSelection) {
            Selection.LEFT -> toggleA()
            Selection.RIGHT -> toggleB()
            Selection.BOTH -> toggleBoth()
        }
    }

    fun smartExtend() {
        if (!systemEnabled) return
        when(currentSelection) {
            Selection.LEFT -> extendA()
            Selection.RIGHT -> extendB()
            Selection.BOTH -> extendBoth()
        }
    }

    fun smartRetract() {
        if (!systemEnabled) return
        when(currentSelection) {
            Selection.LEFT -> retractA()
            Selection.RIGHT -> retractB()
            Selection.BOTH -> retractBoth()
        }
    }

    // ----- Actuator A -----
    fun extendA() = io.Left(PneumaticsIO.State.EXTEND)
    fun retractA() = io.Left(PneumaticsIO.State.RETRACT)
    fun neutralA() = io.Left(PneumaticsIO.State.NEUTRAL)

    fun toggleA() {
        val next =
            if (inputs.Left == PneumaticsIO.State.EXTEND) PneumaticsIO.State.RETRACT
            else PneumaticsIO.State.EXTEND
        io.Left(next)
    }

    // ----- Actuator B -----
    fun extendB() = io.Right(PneumaticsIO.State.EXTEND)
    fun retractB() = io.Right(PneumaticsIO.State.RETRACT)
    fun neutralB() = io.Right(PneumaticsIO.State.NEUTRAL)

    fun toggleB() {
        val next =
            if (inputs.Right == PneumaticsIO.State.EXTEND) PneumaticsIO.State.RETRACT
            else PneumaticsIO.State.EXTEND
        io.Right(next)
    }

    // ----- Both -----
    fun setBoth(state: PneumaticsIO.State) {
        io.Left(state)
        io.Right(state)
    }

    fun extendBoth() = setBoth(PneumaticsIO.State.EXTEND)
    fun retractBoth() = setBoth(PneumaticsIO.State.RETRACT)
    fun neutralBoth() = setBoth(PneumaticsIO.State.NEUTRAL)

    fun toggleBoth() {
        val bothExtended =
            inputs.Left == PneumaticsIO.State.EXTEND &&
                    inputs.Right == PneumaticsIO.State.EXTEND

        setBoth(if (bothExtended) PneumaticsIO.State.RETRACT else PneumaticsIO.State.EXTEND)
    }

    // ----- Commands (nice for RobotContainer bindings) -----
    fun extendACommand(): Command = Commands.runOnce({ extendA() }, this)
    fun retractACommand(): Command = Commands.runOnce({ retractA() }, this)
    fun toggleACommand(): Command = Commands.runOnce({ toggleA() }, this)

    fun extendBCommand(): Command = Commands.runOnce({ extendB() }, this)
    fun retractBCommand(): Command = Commands.runOnce({ retractB() }, this)
    fun toggleBCommand(): Command = Commands.runOnce({ toggleB() }, this)

    fun extendBothCommand(): Command = Commands.runOnce({ extendBoth() }, this)
    fun retractBothCommand(): Command = Commands.runOnce({ retractBoth() }, this)
    fun toggleBothCommand(): Command = Commands.runOnce({ toggleBoth() }, this)
}