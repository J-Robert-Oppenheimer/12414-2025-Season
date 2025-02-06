package pedroPathing.constants;

import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.PINPOINT;

        FollowerConstants.leftFrontMotorName = "leftFront";
        FollowerConstants.leftRearMotorName = "leftBack";
        FollowerConstants.rightFrontMotorName = "rightFront";
        FollowerConstants.rightRearMotorName = "rightBack";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        FollowerConstants.mass = 13;

        FollowerConstants.xMovement = 80;
        FollowerConstants.yMovement = 61.5;

        FollowerConstants.forwardZeroPowerAcceleration = -31.233;
        FollowerConstants.lateralZeroPowerAcceleration = -63.613;

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.4,0.0000001,0.03,0);
        FollowerConstants.useSecondaryTranslationalPID = false;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.4,0,0.001,0);// Not being used, @see useSecondaryTranslationalPID

        FollowerConstants.headingPIDFCoefficients.setCoefficients(2,0.0001,0.0005,0);
        FollowerConstants.useSecondaryHeadingPID = true;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(0.05,0,0.01,0); // Not being used, @see useSecondaryHeadingPID

        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.0125,0.0001,0.001,0.6,0);
        FollowerConstants.useSecondaryDrivePID = true;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.005,0.0001,0.0001,0.6,0); // Not being used, @see useSecondaryDrivePID

        FollowerConstants.zeroPowerAccelerationMultiplier = 5;
        FollowerConstants.centripetalScaling = 0.0002;

        FollowerConstants.pathEndTimeoutConstraint = 500;
        FollowerConstants.pathEndTValueConstraint = 0.995;
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.0007;
    }
}
