package pedroPathing.constants;

import com.pedropathing.localization.Encoder;
import com.pedropathing.localization.constants.ThreeWheelConstants;

public class LConstants {
    static {
//        ThreeWheelConstants.forwardTicksToInches = .001989436789;
        ThreeWheelConstants.forwardTicksToInches = 0.0029;
        ThreeWheelConstants.strafeTicksToInches = 0.003;
        ThreeWheelConstants.turnTicksToInches = .0028;

        ThreeWheelConstants.leftY = 6.75;
        ThreeWheelConstants.rightY = -6.5;
        ThreeWheelConstants.strafeX = 2.5;

        ThreeWheelConstants.leftEncoder_HardwareMapName = "leftFront";
        ThreeWheelConstants.rightEncoder_HardwareMapName = "rightFront";
        ThreeWheelConstants.strafeEncoder_HardwareMapName = "leftBack";
        ThreeWheelConstants.leftEncoderDirection = Encoder.REVERSE;
        ThreeWheelConstants.rightEncoderDirection = Encoder.REVERSE;
        ThreeWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
    }
}
