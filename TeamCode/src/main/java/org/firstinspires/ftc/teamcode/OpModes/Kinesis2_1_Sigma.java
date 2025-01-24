package org.firstinspires.ftc.teamcode.OpModes;

import android.graphics.Color;

import static com.pedropathing.follower.FollowerConstants.leftFrontMotorDirection;
import static com.pedropathing.follower.FollowerConstants.leftFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.leftRearMotorDirection;
import static com.pedropathing.follower.FollowerConstants.leftRearMotorName;
import static com.pedropathing.follower.FollowerConstants.rightFrontMotorDirection;
import static com.pedropathing.follower.FollowerConstants.rightFrontMotorName;
import static com.pedropathing.follower.FollowerConstants.rightRearMotorDirection;
import static com.pedropathing.follower.FollowerConstants.rightRearMotorName;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.util.Constants;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;
import java.util.List;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * This is the LocalizationTest OpMode. This is basically just a simple mecanum drive attached to a
 * PoseUpdater. The OpMode will print out the robot's pose to telemetry as well as draw the robot
 * on FTC Dashboard (192/168/43/1:8080/dash). You should use this to check the robot's localization.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @version 1.0, 5/6/2024
 */
@Config
@TeleOp( name = "Kinesis 2.1.Sigma")
public class Kinesis2_1_Sigma extends OpMode {
    private PoseUpdater poseUpdater;
    private DashboardPoseTracker dashboardPoseTracker;
    private Telemetry telemetryA;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    private List<DcMotorEx> motors;

    GeneralHardwareMap gHMap = new GeneralHardwareMap(this);

    public int armPosition = 4;
    public double armPositionL = 0;
    public double armPositionH = 0;
    public boolean armQueued = false;
    public boolean armMoving = false;
    public boolean armFinished = false;
    public int armTime = 0;
    public boolean bLast;
    int slidePos = 0;
    double HslidePos = 0;
    int white;
    double dY;
    double low0, highF, lowF;
    double speed = 1;
    double inverseSpeed = -1/speed;
    double E = Math.E;
    double intermJoint;



    // ------------------ Booleans ------------------
    boolean toggle = false;// Tracks the current toggle state
    boolean first = true;
    boolean prevButton = false;
    boolean blueClaw = false;
    boolean yellowClaw = false;
    long elapsedTime = 0;
    long armStartTime = 0; // To track when the arm motion started
    final long ARM_DURATION_MS = 1000;
    private Servo lowL, lowR, highL, highR;
    // -------------------- Colors ------------------\\
    float hue;
    float hue2;
    float saturation;
    float value;
    float[] hsvValues = new float[3];
    float[] hsvValues2 = new float[3];
    int green, red, blue;
    double power;
    int i=0;
    int airFryer =0;
    int gibbs = 0;
    boolean sampleReturn = false;
    double clawDist;
    long slideStartTime;
    long slideStartTime2;
    long currentTime;
    boolean HslideRetract;
    int LastArmPos;
    boolean vSlideLowering = false;
    int lastSlidePos;
    long vSlideStart;

    /**
     * This initializes the PoseUpdater, the mecanum drive motors, and the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        gHMap.init("Kineses");
        lowL = gHMap.zero;
        lowR = gHMap.zero2;
        highL = gHMap.zero3;
        highR = gHMap.zero4;

        Constants.setConstants(FConstants.class, LConstants.class);
        poseUpdater = new PoseUpdater(hardwareMap);

        dashboardPoseTracker = new DashboardPoseTracker(poseUpdater);

        leftFront = hardwareMap.get(DcMotorEx.class, leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, rightFrontMotorName);
        leftFront.setDirection(leftFrontMotorDirection);
        leftRear.setDirection(leftRearMotorDirection);
        rightFront.setDirection(rightFrontMotorDirection);
        rightRear.setDirection(rightRearMotorDirection);

        motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will print your robot's position to telemetry while "
                + "allowing robot control through a basic mecanum drive on gamepad 1.");
        telemetryA.update();

        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
    }

    /**
     * This updates the robot's pose estimate, the simple mecanum drive, and updates the FTC
     * Dashboard telemetry with the robot's position as well as draws the robot's position.
     */
    @Override
    public void loop() {
        currentTime = System.currentTimeMillis();
        poseUpdater.update();
        dashboardPoseTracker.update();

        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x; // this is strafing
        double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double leftFrontPower = (y + x + rx) / denominator;
        double leftRearPower = (y - x + rx) / denominator;
        double rightFrontPower = (y - x - rx) / denominator;
        double rightRearPower = (y + x - rx) / denominator;

        leftFront.setPower(leftFrontPower);
        leftRear.setPower(leftRearPower);
        rightFront.setPower(rightFrontPower);
        rightRear.setPower(rightRearPower);

        telemetryA.addData("x", poseUpdater.getPose().getX());
        telemetryA.addData("y", poseUpdater.getPose().getY());
        telemetryA.addData("heading", poseUpdater.getPose().getHeading());
        telemetryA.addData("total heading", poseUpdater.getTotalHeading());
        telemetryA.update();


        //Right Trigger = Vertical Extend
        //Left Trigger = Vertical Retract
        //Right Bumper = Horizontal Extend
        //Left Bumper = Horizontal Retract

        //Dpad up = sample place high
        //Dpad right = spec place high
        //Dpad down = transit mode/ return to zero

//                gamepad1.start //engage intake

        //Y = Open/Close grabber
        //B = Grabber in Specimen pos
        //X = Grabber in Transfer Pos
        //A = Grabber in Sample Pos
        //Press last button again or autonomous feedback = dropPos if sample present or Receive pos if not


        //                slidePos = (slidePos > -630?gamepad1.right_trigger
        if (slidePos > -660 && gamepad1.right_trigger > 0.5) {
            slidePos-=5;
            if (gamepad1.start) {
                slidePos = -570;
            }
        }

        if (slidePos < 0 && gamepad1.left_trigger > 0.5) {
            slidePos += 5;
        }


//        power = (!vSlideLowering && )?0:1;
        if(slidePos > lastSlidePos)vSlideLowering = true;
        if(vSlideLowering)power = 0.1;
        else if(Math.abs(slidePos) < 2 && Math.abs(gHMap.bone1.getCurrentPosition()) < 3)power = 0;
        else power = 1;
        if(!vSlideLowering)slidePos = lastSlidePos;
        if(Math.abs(slidePos- gHMap.bone1.getCurrentPosition()) < 4)vSlideLowering =false;

        gHMap.bone1.setPower(power);
//        gHMap.bone2.setPower(power);
        gHMap.bone3.setPower(power);


        gHMap.bone1.setTargetPosition(slidePos);
//        gHMap.bone2.setTargetPosition(slidePos);
        gHMap.bone3.setTargetPosition(slidePos);

        //------------------- HORIZONTAL SLIDES --------------------------
        if (gamepad1.right_bumper && HslidePos < 1.0) {
            HslidePos = 0.95; currentTime = slideStartTime;HslideRetract = true;

        }
        if(first && currentTime > slideStartTime + 250){
            gHMap.wrist.setPosition(0.18);
            first = false;
        }
        if(gamepad1.left_bumper){
            HslidePos = 0;
            gHMap.wrist.setPosition(0.0);
            currentTime = slideStartTime2;
        }
        if(slideStartTime2 + 500 > currentTime && HslideRetract && HslidePos < 0.01){
            gHMap.zero8.setPower(0);
            HslideRetract =false;
        }



//        if (gamepad2.left_bumper && HslidePos > 0) HslidePos -= 0.005;
//        if(HslidePos < 0.005)first=true;

        gHMap.ex.setPosition(HslidePos);
        gHMap.ex2.setPosition(HslidePos);



        if (gamepad1.dpad_right) {
            gHMap.zero8.setPower(1);
        }
        if (gamepad1.dpad_left) {
            gHMap.zero8.setPower(-1);
        }
        if (gamepad1.back) {
            gHMap.zero8.setPower(0);
        }

        if (gamepad1.dpad_down) {
            gHMap.wrist.setPosition(0.18);
        }
        else if (gamepad1.dpad_up) {
            gHMap.wrist.setPosition(0.0);
        }


        boolean currentY = gamepad1.y;

        // Toggle logic
        if (currentY && !prevButton) {
            toggle = !toggle;
        }

        // Update the previous button state
        prevButton = currentY;

        gHMap.zero5.setPosition(toggle ? 0.24 : 0);
        if(!sampleReturn) {
            if (armPosition == 1 && !toggle && gamepad1.y && gHMap.bone1.getCurrentPosition() < -600) {
                sampleReturn = true;
            }
        }
        if(sampleReturn){
            gibbs++;
        }

        if(gibbs > 20){
            slidePos = -400;
        }
        if(gibbs > 60){
            gibbs = 0;
            armPosition = 0;
            armQueued = true;
            toggle = false;
            sampleReturn = false;

        }

        // (boolean? yes:no)

        // ------------ arm functions ----------

        if (gamepad1.a && armPosition != 1) {
            armPosition = 1;
            armQueued = true;
        }

        if (gamepad1.x && armPosition != 0) {
            armPosition = 0;
            armQueued = true;
            toggle = false;
        }

        if (gamepad1.b && armPosition != 2) {
            armPosition = 2;
            bLast = true;
            armQueued = true;
            toggle = true;
        }

        if (gamepad1.b && armPosition == 2 && !bLast) {
            armPosition = 3;
            armQueued = true;
        }

        bLast = gamepad1.b;


        //TODO - TRANSITION BEHAVIOR LOOPS RUN IN TELEOP CLASS
        if (armQueued) {
            if (armPosition == 1) { //Sample
                // 0, 0
                //        gHMap.jointPos(0, 0);
                slidePos = -630;
//                gHMap.bone1.setTargetPosition(slidePos);
//                gHMap.bone2.setTargetPosition(slidePos);
//                gHMap.bone3.setTargetPosition(slidePos);
                dY = 0 - armPositionL;
            }
            if (armPosition == 2) {//Wall Grab
                //16 38
                //        gHMap.jointPos(0.16, 0.38);
                slidePos = 0;
//                gHMap.bone1.setTargetPosition(slidePos);
//                gHMap.bone2.setTargetPosition(slidePos);
//                gHMap.bone3.setTargetPosition(slidePos);
                dY = 0.77 - armPositionL;
            }
            if (armPosition == 0) {//Transfer
                //72
                //        gHMap.jointPos(0.72, 0);
                slidePos = 0;
//                gHMap.bone1.setTargetPosition(slidePos);
//                gHMap.bone2.setTargetPosition(slidePos);
//                gHMap.bone3.setTargetPosition(slidePos);
                dY = 0.41 - armPositionL;
//                        gHMap.smoothJoints(armPositionL, 0.065, 0.42);
//                        armPositionL = 0.065;
//                        armPositionH = 0.42;
            }
            if (armPosition == 3) {//Spec Place Position
                //05 22
                dY = 0.24 - armPositionL;
            }
            armQueued = false;
            armMoving = true;
            armTime = (int)speed * -5;
            highL.setPosition(0.62);
            highR.setPosition(0.62);
//                    armStartTime = System.currentTimeMillis() - ((long)speed*-5);

        } // ---------------------------- SWITCH FROM QUEUED TO MOVING ------------------------------

        if(armMoving){
            if(armPosition ==2)slidePos = -200;
//                    elapsedTime = System.currentTimeMillis() - armStartTime;
            intermJoint = dY/(1+Math.pow(E, (inverseSpeed)*armTime))+armPositionL;
            armTime++;
            lowL.setPosition(intermJoint);
            lowR.setPosition(intermJoint);

//                    gHMap.jointPos(dY/(1+Math.pow(Math.E, (-1/speed)*armTime))+low0,0.63);

            if(armTime > speed * 7) {
                armMoving = false;
//                        armFinished = true;
                if (armPosition == 1) { //Sample
                    armPositionL = 0;
                    armPositionH = 0.2;
                    jointPos(armPositionL,armPositionH);
                }
                else if (armPosition == 2) {//Wall Grab
                    armPositionL = 0.77;//0.77
                    armPositionH = 0.38;
                    slidePos = 0;
                    jointPos(armPositionL,armPositionH);
                }
                else if (armPosition == 0) {//Transfer
                    armPositionL = 0.41;
                    armPositionH = 0.34;
                    jointPos(armPositionL,armPositionH);
                }
                else if (armPosition == 3) {//Spec Place Position
                    armPositionL = 0.24;
                    armPositionH = 0.22;
                    jointPos(armPositionL,armPositionH);
                }
            }
        }//Arm moving finished

//                if(armFinished){
//
//                }


        //TODO - AUTO ROUTES - COLOR SENSORS - STREAMLINING


        // -- COLOR FUNCTIONS --
        if(armPosition == 0) {//Transfer System ------------------

            red = gHMap.ClawSense.red();
            green = gHMap.ClawSense.green();
            blue = gHMap.ClawSense.blue();

            // Convert RGB to HSV

            Color.RGBToHSV(
                    (int) (red * 255.0 / 1023),
                    (int) (green * 255.0 / 1023),
                    (int) (blue * 255.0 / 1023),
                    hsvValues
            );

            // Use HSV values
            hue = hsvValues[0];
            saturation = hsvValues[1];
            value = hsvValues[2];

            // Display on telemetry
            telemetry.addData("Hue", hue);
            telemetry.addData("Saturation", saturation);
            telemetry.addData("Value", value);
            telemetry.addData("Distance", gHMap.ClawSense.getDistance(DistanceUnit.INCH));

            if (hue < 30 || hue > 330) {
                telemetry.addData("Color", "Red");
            } else if (hue > 210 && hue < 270) {
                telemetry.addData("Color", "Blue");
                blueClaw = true;
                yellowClaw = false;
            } else if (hue > 50 && hue < 70) {
                telemetry.addData("Color", "Yellow");
                yellowClaw = true;
                blueClaw = false;
            } else {
                telemetry.addData("Color", "Unknown");
            }

            if (gHMap.ClawSense.getDistance(DistanceUnit.INCH) < 2 && hue > 210 && hue < 270) {
                toggle = true;
            } else if (gHMap.ClawSense.getDistance(DistanceUnit.INCH) < 2 && hue > 50 && hue < 80) {
                toggle = true;
            }

            if (hue > 156 && hue < 164) {
                telemetry.addData("Color", "White");
                white++;
                if (white > 40) {
                    toggle = false;
                }
            } else {
                white = 0;
            }
        }


        //SWITCH

        if(HslidePos > 0.01) {

            // Convert RGB to HSV

            Color.RGBToHSV(
                    (int) (gHMap.SlurpSense.red() * 255.0 / 1023),
                    (int) (gHMap.SlurpSense.green() * 255.0 / 1023),
                    (int) (gHMap.SlurpSense.blue() * 255.0 / 1023),
                    hsvValues2
            );
            hue2 = hsvValues2[0];
//

            if (hue2 < 30 || hue2 > 330) {//RED
                gHMap.zero8.setPower(1);
            } else if (hue2 > 210 && hue2 < 270) {//
                airFryer++;
            } else if (hue2 > 50 && hue2 < 85) {
                airFryer++;
            }
            if(airFryer > 8){
                gHMap.wrist.setPosition(0.0);
                airFryer = 0;
                HslidePos = 0;
//                gHMap.zero8.setPower(0);
                slideStartTime = currentTime;
            }

//                    if (gHMap.ClawSense.getDistance(DistanceUnit.INCH) < 2 && hue > 210 && hue < 270) {
//                        toggle = true;
//                    } else if (gHMap.ClawSense.getDistance(DistanceUnit.INCH) < 2 && hue > 50 && hue < 80) {
//                        gHMap.zero8.setPower(-1);
//                    }

        }


        if(armPosition == 2) { //Wall SPecimen

            red = gHMap.ClawSense.red();
            green = gHMap.ClawSense.green();
            blue = gHMap.ClawSense.blue();

            // Convert RGB to HSV

            Color.RGBToHSV(
                    (int) (red * 255.0 / 1023),
                    (int) (green * 255.0 / 1023),
                    (int) (blue * 255.0 / 1023),
                    hsvValues
            );

            // Use HSV values
            hue = hsvValues[0];
            saturation = hsvValues[1];
            value = hsvValues[2];

            // Display on telemetry
            telemetry.addData("Hue", hue);
            telemetry.addData("Saturation", saturation);
            telemetry.addData("Value", value);
            telemetry.addData("Distance", gHMap.ClawSense.getDistance(DistanceUnit.INCH));

            if (hue < 30 || hue > 330) {
                telemetry.addData("Color", "Red");
            } else if (hue > 210 && hue < 270) {
                telemetry.addData("Color", "Blue");
                gHMap.zero8.setPower(1);
            } else if (hue > 50 && hue < 70) {
                telemetry.addData("Color", "Yellow");
            } else {
                telemetry.addData("Color", "Unknown");
            }
            clawDist = gHMap.ClawSense.getDistance(DistanceUnit.INCH);
            if (clawDist < 2 && hue > 210 && hue < 270 && clawDist > 0.5) {
                toggle = true;
                slidePos = -200;
            } else if (clawDist < 2 && hue > 50 && hue < 80 && clawDist > 0.5) {
                toggle = true;
                slidePos = -200;
            }
            if (hue > 156 && hue < 164) {
                telemetry.addData("Color", "White");
                white++;
                if (white > 40) {
                    toggle = false;
                }
            } else {
                white = 0;
            }
        }


        //DEBUG CONTROL
        if(gamepad2.touchpad){
            if(gamepad2.left_bumper){//HSlide in
                HslidePos+= 0.02;
            }
            if(gamepad2.right_bumper){//HSlide out
                HslidePos-= 0.02;
            }


            if(gamepad2.right_stick_y > 0 && armPositionL > 0 && armPositionL < 0.77){
            armPositionL -= gamepad2.right_stick_y * 0.01;
            jointPos(armPositionL,armPositionH);}

            if(gamepad2.left_stick_y > 0 && armPositionH > 0 && armPositionH < 1) {
                armPositionH = gamepad2.left_stick_y * 0.01;
                jointPos(armPositionL, armPositionH);}

            if(gamepad2.dpad_down){
                gHMap.wrist.setPosition(0.18);
            }
            if(gamepad2.dpad_up){
                gHMap.wrist.setPosition(0);
            }


        }








        elapsedTime = currentTime - armStartTime;

        i++;

        //-------------- end of bot functions -----------------
        telemetry.addData("Gibbs", gibbs);
        telemetry.addData("Hue 2", hue2);
        telemetry.addData("Airfryer", airFryer);
        telemetry.addData("CurrentPos LowJoint", armPositionL);
        telemetry.addData("CurrentPos HighJoin", armPositionH);
        telemetry.addData("DY", dY);
        telemetry.addData("ArmPos", armPosition);
        telemetry.addData("SlidePos", slidePos);
        telemetry.addData("Toggle", toggle);
        telemetry.addData("Bone1", gHMap.bone1.getCurrentPosition());
        telemetry.addData("Bone2", gHMap.bone2.getCurrentPosition());
        telemetry.addData("Bone3", gHMap.bone3.getCurrentPosition());
        telemetry.addData("ElapsedTime", (double)elapsedTime/1000);
//                telemetry.addData("armStartTime", armStartTime);
        telemetry.addData("armTime", armTime);
        telemetry.addData("interm Joint", intermJoint);
        telemetry.addData("iterator", i);
        telemetry.addData("Processing Speed", i/((double)elapsedTime/1000));

        telemetry.update();

//        TelemetryPacket packet = new TelemetryPacket();
//        packet.fieldOverlay().setStroke("#3F51B5");
//        Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
//        FtcDashboard.getInstance().sendTelemetryPacket(packet);



































        Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();
    }
    public void jointPos(double lowPos, double highPos){
        lowL.setPosition(lowPos);
        lowR.setPosition(lowPos);
        highL.setPosition(highPos);
        highR.setPosition(highPos);
    }
}
