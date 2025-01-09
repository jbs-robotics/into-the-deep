package pedroPathing.constants;

import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Localizers;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
      static {
        // Select our localizer
        FollowerConstants.localizers = Localizers.THREE_WHEEL;

        // We can change the value of any variable/constant of FollowerConstants.
        FollowerConstants.mass = 16.5; // In kg

        FollowerConstants.leftFrontMotorName = "leftFront";
        FollowerConstants.leftRearMotorName = "leftBack";
        FollowerConstants.rightFrontMotorName = "rightFront";
        FollowerConstants.rightRearMotorName = "rightBack";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        FollowerConstants.xMovement = 75.3066;
        FollowerConstants.yMovement = 55.6702;

        FollowerConstants.forwardZeroPowerAcceleration = -33.4054;
        FollowerConstants.lateralZeroPowerAcceleration = -84.5;
        FollowerConstants.translationalPIDFCoefficients = new CustomPIDFCoefficients(0.4, 0.0,0.04, 0.0);
        FollowerConstants.headingPIDFCoefficients = new CustomPIDFCoefficients(3, 0.0,0.04, 0.0);
        FollowerConstants.zeroPowerAccelerationMultiplier= 4;
        FollowerConstants.drivePIDFCoefficients = new CustomFilteredPIDFCoefficients(0.025, 0.0, 0.00001, 0.6, 0.0);

        FollowerConstants.centripetalScaling = 0.0006;

    }

}
