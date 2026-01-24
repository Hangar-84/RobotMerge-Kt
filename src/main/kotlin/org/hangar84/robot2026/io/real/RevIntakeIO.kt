package org.hangar84.robot2026.io.real

import com.revrobotics.PersistMode
import com.revrobotics.ResetMode
import com.revrobotics.spark.SparkLowLevel.MotorType
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.hangar84.robot2026.io.IntakeIO

class RevIntakeIO : IntakeIO {
    private val leftIntake = SparkMax(13, MotorType.kBrushed)

    init {
        leftIntake.configure(
            SparkMaxConfig(),
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        )
    }

    override fun setPercent(percent: Double) {
        leftIntake.set(percent)
        DriverStation.reportWarning("Intake Set = $percent", false)
        SmartDashboard.putString("Debug/IntakeCmd", "set=$percent")
    }

    override fun updateInputs(inputs: IntakeIO.Inputs) {
        inputs.leftAppliedOutput = leftIntake.appliedOutput
        inputs.leftCurrentAmps = leftIntake.outputCurrent
    }
}