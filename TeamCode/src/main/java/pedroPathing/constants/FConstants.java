package pedroPathing.constants;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Localizers;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
      static {
        // Turn on voltage compensation
        FollowerConstants.useVoltageCompensationInAuto = true;
        FollowerConstants.useVoltageCompensationInTeleOp = true;
        FollowerConstants.nominalVoltage = 12; // TODO: Change this when tuning
        FollowerConstants.cacheInvalidateSeconds = 0.5;


        // Select our localizer
        FollowerConstants.localizers = Localizers.THREE_WHEEL;

        // We can change the value of any variable/constant of FollowerConstants.
        FollowerConstants.mass = 16.9; // In kg

        FollowerConstants.leftFrontMotorName = "leftFront";
        FollowerConstants.leftRearMotorName = "leftBack";
        FollowerConstants.rightFrontMotorName = "rightFront";
        FollowerConstants.rightRearMotorName = "rightBack";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        FollowerConstants.xMovement = 66.666;
        FollowerConstants.yMovement = 47.1224;

        FollowerConstants.forwardZeroPowerAcceleration = -37.9938;
        FollowerConstants.lateralZeroPowerAcceleration = -92.6107;
        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.4, 0.0, 0.04, 0.0);
        FollowerConstants.headingPIDFCoefficients.setCoefficients(2, 0.0,0.09, 0.0);
        FollowerConstants.zeroPowerAccelerationMultiplier= 6;
        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.0205, 0.0, 0.00003, 0.6, 0.0);

//        FollowerConstants.
        FollowerConstants.centripetalScaling = 0.0006;

    }

}
