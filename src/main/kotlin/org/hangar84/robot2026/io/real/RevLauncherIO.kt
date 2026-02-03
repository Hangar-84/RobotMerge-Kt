package org.hangar84.robot2026.io.real

import com.revrobotics.PersistMode
import com.revrobotics.ResetMode
import com.revrobotics.spark.SparkLowLevel.MotorType
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.hangar84.robot2026.constants.Constants.Launcher
import org.hangar84.robot2026.io.LauncherIO


class RevLauncherIO : LauncherIO {
    private val leftLaunch = SparkMax(Launcher.Launcher_Left_Motor, MotorType.kBrushed)
    private val rightLaunch = SparkMax(Launcher.Launcher_Right_Motor, MotorType.kBrushed)

    init {
        val rightCfg = SparkMaxConfig().apply {
            smartCurrentLimit(20)
        }
        val leftCfg = SparkMaxConfig().apply {
            inverted(true)
            smartCurrentLimit(20)
        }
        leftLaunch.configure(leftCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
        rightLaunch.configure(rightCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
    }

    override fun setPercent(percent: Double) {
        leftLaunch.set(percent)
        rightLaunch.set(percent)
    }

    override fun updateInputs(inputs: LauncherIO.Inputs) {
        inputs.leftAppliedOutput = leftLaunch.appliedOutput
        inputs.leftCurrentAmps = leftLaunch.outputCurrent
        inputs.leftTempCelsius = leftLaunch.motorTemperature

        inputs.rightAppliedOutput = rightLaunch.appliedOutput
        inputs.rightCurrentAmps = rightLaunch.outputCurrent
        inputs.rightTempCelsius = rightLaunch.motorTemperature
    }
}