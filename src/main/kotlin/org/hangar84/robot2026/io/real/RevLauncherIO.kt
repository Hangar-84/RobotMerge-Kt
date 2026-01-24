package org.hangar84.robot2026.io.real

import com.revrobotics.PersistMode
import com.revrobotics.ResetMode
import com.revrobotics.spark.SparkLowLevel.MotorType
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.hangar84.robot2026.io.LauncherIO

class RevLauncherIO : LauncherIO {
    private val leftLaunch = SparkMax(15, MotorType.kBrushed)
    private val rightLaunch = SparkMax(16, MotorType.kBrushed)

    init {
        val rightCfg = SparkMaxConfig().apply {
        }
        val leftCfg = SparkMaxConfig().apply {
            inverted(true)
        }
        leftLaunch.configure(leftCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
        rightLaunch.configure(rightCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
    }

    override fun setPercent(percent: Double) {
        leftLaunch.set(percent)
        rightLaunch.set(percent)
        DriverStation.reportWarning("Launch Set = $percent", false)
        SmartDashboard.putString("Debug/LauncherCmd", "set=$percent")
        // right follows
    }

    override fun updateInputs(inputs: LauncherIO.Inputs) {
        inputs.leftAppliedOutput = leftLaunch.appliedOutput
        inputs.rightAppliedOutput = rightLaunch.appliedOutput
        inputs.leftCurrentAmps = leftLaunch.outputCurrent
        inputs.rightCurrentAmps = rightLaunch.outputCurrent
    }
}