import edu.wpi.first.wpilibj.PneumaticsControlModule
import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj.Solenoid
import org.hangar84.robot2026.io.PneumaticsIO

class CtreTwoValvePnematicsIO(
    pcmCanId: Int,
    aExtend: Int, aRetract: Int,
    bExtend: Int, bRetract: Int,
) : PneumaticsIO {

    private val pcm = PneumaticsControlModule(pcmCanId)
    private val LeftExt = Solenoid(pcmCanId, PneumaticsModuleType.CTREPCM, aExtend)
    private val LeftRet = Solenoid(pcmCanId, PneumaticsModuleType.CTREPCM, aRetract)

    private val RightExt = Solenoid(pcmCanId, PneumaticsModuleType.CTREPCM, bExtend)
    private val RightRet = Solenoid(pcmCanId, PneumaticsModuleType.CTREPCM, bRetract)

    private var LeftState = PneumaticsIO.State.NEUTRAL
    private var RightState = PneumaticsIO.State.NEUTRAL

    override fun Left(state: PneumaticsIO.State) {
        LeftState = state
        when (state) {
            PneumaticsIO.State.EXTEND -> { LeftRet.set(false); LeftExt.set(true) }
            PneumaticsIO.State.RETRACT -> { LeftExt.set(false); LeftRet.set(true) }
            PneumaticsIO.State.NEUTRAL -> { LeftExt.set(false); LeftRet.set(false) }
        }
    }

    override fun Right(state: PneumaticsIO.State) {
        RightState = state
        when (state) {
            PneumaticsIO.State.EXTEND -> { RightRet.set(false); RightExt.set(true) }
            PneumaticsIO.State.RETRACT -> { RightExt.set(false); RightRet.set(true) }
            PneumaticsIO.State.NEUTRAL -> { RightExt.set(false); RightRet.set(false) }
        }
    }

    override fun setCompressor(enabled: Boolean) {
        if (enabled) {
            pcm.enableCompressorDigital()
        } else {
            pcm.disableCompressor()
        }
    }

    override fun updateInputs(inputs: PneumaticsIO.Inputs) {
        inputs.Left = LeftState
        inputs.Right = RightState

        inputs.CompressorEnabled = pcm.compressor

        val aIsExtending = LeftExt.get()
        inputs.Left_Solenoid_Extend = aIsExtending
        inputs.Left_Solenoid_Retract = !aIsExtending

        val bIsExtending = RightExt.get()
        inputs.Right_Solenoid_Extend = bIsExtending
        inputs.Right_Solenoid_Retract = !bIsExtending
    }
}