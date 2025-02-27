package pedroPathing.constants;

import com.pedropathing.localization.Encoder;
import com.pedropathing.localization.constants.ThreeWheelConstants;

public class LConstants {
    static {
//        ThreeWheelConstants.forwardTicksToInches = .001989436789;
        ThreeWheelConstants.forwardTicksToInches = 0.003;
        ThreeWheelConstants.strafeTicksToInches = 0.00295;
        ThreeWheelConstants.turnTicksToInches = 0.0031;
//0.0029,
        ThreeWheelConstants.leftY = 7.1875;
        ThreeWheelConstants.rightY = -7.1875;
        ThreeWheelConstants.strafeX = 1.75;

        ThreeWheelConstants.leftEncoder_HardwareMapName = "leftFront";
        ThreeWheelConstants.rightEncoder_HardwareMapName = "rightFront";
        ThreeWheelConstants.strafeEncoder_HardwareMapName = "leftBack";
        ThreeWheelConstants.leftEncoderDirection = Encoder.FORWARD;
        ThreeWheelConstants.rightEncoderDirection = Encoder.REVERSE;
        ThreeWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
    }
}
