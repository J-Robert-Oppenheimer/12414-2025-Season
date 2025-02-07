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
@TeleOp( name = "Kinesis 2.5.RaptorLake")
public class Kinesis2_5_RaptorLake extends OpMode {
    private PoseUpdater poseUpdater;
    private DashboardPoseTracker dashboardPoseTracker;
    private Telemetry telemetryA;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    private List<DcMotorEx> motors;

    GeneralHardwareMap gHMap = new GeneralHardwareMap(this);

    public int armPosition = 1;
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
    double Singapore_Im_Senatorian = 0;
    double scaleEncoders = 1.5;



    // ------------------ Booleans ------------------
    boolean toggle = false;// Tracks the current toggle state
    boolean first = false;
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
    double col=0;
    boolean triangle2 = false, square2 = false, lastTriangle2, lastSquare2;
    boolean currentY;

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
        gHMap.armHandler(armPosition, 0);
        armPosition = 0;
        armPositionH = 0.34;
        armPositionL = 0.41;

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

//        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
//        telemetryA.addLine("This will print your robot's position to telemetry while "
//                + "allowing robot control through a basic mecanum drive on gamepad 1.");
//        telemetryA.update();

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

//        telemetryA.addData("x", poseUpdater.getPose().getX());
//        telemetryA.addData("y", poseUpdater.getPose().getY());
//        telemetryA.addData("heading", poseUpdater.getPose().getHeading());
//        telemetryA.addData("total heading", poseUpdater.getTotalHeading());
//        telemetryA.update();


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


//        gamepad1.setLedColor(col,col,col,1000);
        col++;
        col%=255;
        if (slidePos > -660 && gamepad1.right_trigger > 0.5) {
            slidePos-=5;
            if (gamepad1.start) {
                slidePos = -570;
            }
        }
        else if(gamepad1.ps){
//            slidePos = -330;
            highL.setPosition(0.08);

        }

        if (slidePos < 400 && gamepad1.left_trigger > 0.5) {
            slidePos += 5;
        }


//        power = (!vSlideLowering)?0:1;
        if(slidePos > lastSlidePos)vSlideLowering = true;
        if(vSlideLowering)power = 0.4;
        else if(Math.abs(slidePos) < 2 && Math.abs(gHMap.bone1.getCurrentPosition()) < 3)power = 0;
        else power = 1;
        if(!vSlideLowering)lastSlidePos = slidePos;
        if(Math.abs(slidePos- gHMap.bone1.getCurrentPosition()) < 4)vSlideLowering =false;

//        power = (Math.abs(slidePos) < 3 && Math.abs(gHMap.bone1.getCurrentPosition()) < 3?0:1);

        gHMap.bone1.setPower(power);
//        gHMap.bone2.setPower(power);
        gHMap.bone3.setPower(power);
//        slidePos = (slidePos * scaleEncoders);


        gHMap.bone1.setTargetPosition((int)(slidePos* scaleEncoders));
//        gHMap.bone2.setTargetPosition(slidePos);
        gHMap.bone3.setTargetPosition((int)(-1*slidePos* scaleEncoders));

        //------------------- HORIZONTAL SLIDES --------------------------
        if (gamepad1.right_bumper && HslidePos < 1.0) {
            first = true;
            HslidePos = 0.95; slideStartTime = currentTime;HslideRetract = true;
            gHMap.zero8.setPower(-1);

        }
        if(first && currentTime > slideStartTime + 400){
            gHMap.wrist.setPosition(0.18);
            gHMap.wrist2.setPosition(0.18);
            first = false;
        }
        if(gamepad1.left_bumper){
            HslidePos = 0;
            gHMap.wrist.setPosition(0.0);
            gHMap.wrist2.setPosition(0.0);
            slideStartTime2 = currentTime;
            HslideRetract =true;
        }
        if(currentTime - slideStartTime2  > 1000 && HslideRetract && HslidePos < 0.02){
            gHMap.zero8.setPower(1);

        }
        if(currentTime - slideStartTime2  > 2500 && HslideRetract && HslidePos < 0.02){
            gHMap.zero8.setPower(0);
            HslideRetract =false;
        }



//        if (gamepad2.left_bumper && HslidePos > 0) HslidePos -= 0.005;
//        if(HslidePos < 0.005)first=true;

        gHMap.ex.setPosition(HslidePos/2);
        gHMap.ex2.setPosition(HslidePos/2);



        if (gamepad1.dpad_right) {
            gHMap.zero8.setPower(1);
        }
        if (gamepad1.dpad_left) {
            gHMap.zero8.setPower(-1);
        }
        if (gamepad1.back) {
            gHMap.zero8.setPower(0);
        }

//        if (gamepad1.dpad_down) {
//            gHMap.wrist.setPosition(0.18);
//            gHMap.wrist2.setPosition(0.18);
//        }
        else if (gamepad1.dpad_up) {
            gHMap.wrist.setPosition(0.0);
            gHMap.wrist2.setPosition(0.0);
        }


        currentY = gamepad1.y;

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

//        if(gibbs > 20){
//            slidePos = -400;
//        }
        if(gibbs > 15){
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

        if (gamepad1.b && armPosition != 2 && !bLast) {
            armPosition = 2;
            bLast = true;
            armQueued = true;
            toggle = true;
        }

        if (gamepad1.b && armPosition == 2 && !bLast) {
            armPosition = 3;
            armQueued = true;
            bLast = true;
        }

        bLast = gamepad1.b;


        //TODO - TRANSITION BEHAVIOR LOOPS RUN IN TELEOP CLASS
        if (armQueued) {
            if (armPosition == 1) { //Sample
                // 0, 0
                //        gHMap.jointPos(0, 0);
                slidePos = -650;toggle = true;
//                gHMap.bone1.setTargetPosition(slidePos);
//                gHMap.bone2.setTargetPosition(slidePos);
//                gHMap.bone3.setTargetPosition(slidePos);
                dY = 0 - armPositionL;
            }
            if (armPosition == 2) {//Wall Grab
                //16 38
                //        gHMap.jointPos(0.16, 0.38);
//                gHMap.bone1.setTargetPosition(slidePos);
//                gHMap.bone2.setTargetPosition(slidePos);
//                gHMap.bone3.setTargetPosition(slidePos);
                toggle = true;
                dY = 0.715 - armPositionL;
            }
            if (armPosition == 0) {//Transfer
                //72
                //        gHMap.jointPos(0.72, 0);
                slidePos = 0;
//                gHMap.bone1.setTargetPosition(slidePos);
//                gHMap.bone2.setTargetPosition(slidePos);
//                gHMap.bone3.setTargetPosition(slidePos);
                dY = 0.408 - armPositionL;
//                        gHMap.smoothJoints(armPositionL, 0.065, 0.42);
//                        armPositionL = 0.065;
//                        armPositionH = 0.42;
            }
            if (armPosition == 3) {//Spec Place Position
                //05 22
                dY = 0.71 - armPositionL;
                slidePos = -125;
            }
            armQueued = false;
            armMoving = true;
            armTime = (int)speed * -5;
            highL.setPosition(0.6);
            highR.setPosition(0.6);
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

            if(armTime > speed * 4) {
                armMoving = false;
//                        armFinished = true;
                if (armPosition == 1) { //Sample
                    armPositionL = 0;
                    armPositionH = 0.2;
                    jointPos(armPositionL,armPositionH);
                }
                else if (armPosition == 2) {//Wall Grab
                    armPositionL = 0.73;//0.77
                    armPositionH = 0.43;
                    slidePos = -50;
                    toggle = false;
                    jointPos(armPositionL,armPositionH);
                }
                else if (armPosition == 0) {//Transfer
                    toggle = false;
                    armPositionL = 0.408;
                    armPositionH = 0.34;
                    jointPos(armPositionL,armPositionH);
                }
                else if (armPosition == 3) {//Spec Place Position
//                    armPositionL = 0.24;
                    armPositionL = 0.71;
//                    armPositionH = 0.22;
                    armPositionH = 0.62;
                    slidePos = -430;
                    jointPos(armPositionL,armPositionH);
                }
            }
        }//Arm moving finished


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

            if (hue > 210 && hue < 270) {
                toggle = true;//gHMap.ClawSense.getDistance(DistanceUnit.INCH) < 2 &&
            } else if (hue > 50 && hue < 80) {
                toggle = true;//gHMap.ClawSense.getDistance(DistanceUnit.INCH) < 2 &&
            }

            if (hue > 156 && hue < 164) {
                telemetry.addData("Color", "White");
                white++;
                if (white > 30) {
                    toggle = false;
                }
            } else {
                white = 0;
            }
        }


        //SWITCH

        if(HslidePos > 0.01 && triangle2) {//

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
            if (hue2 > 156 && hue2 < 164 && HslidePos > 0.5) {
                gHMap.zero8.setPower(-1);
            }

            if(airFryer > 2){
                gHMap.wrist.setPosition(0.0);
                gHMap.wrist2.setPosition(0.0);
                airFryer = 0;
                HslidePos = 0;
//                gHMap.zero8.setPower(0);
                HslideRetract = true;
                slideStartTime2 = currentTime;
            }

//                    if (gHMap.ClawSense.getDistance(DistanceUnit.INCH) < 2 && hue > 210 && hue < 270) {
//                        toggle = true;
//                    } else if (gHMap.ClawSense.getDistance(DistanceUnit.INCH) < 2 && hue > 50 && hue < 80) {
//                        gHMap.zero8.setPower(-1);
//                    }


        }
        if(gamepad1.start && gamepad1.back){
            gHMap.bone1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            gHMap.bone3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            gHMap.bone1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            gHMap.bone3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slidePos=0;
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
            } else if (hue > 50 && hue < 70) {
                telemetry.addData("Color", "Yellow");
            } else {
                telemetry.addData("Color", "Unknown");
            }
            clawDist = gHMap.ClawSense.getDistance(DistanceUnit.INCH);

            if (clawDist < 2 && hue > 210 && hue < 270 && clawDist > 0.5) {
                toggle = true;
                Singapore_Im_Senatorian++;
                if(Singapore_Im_Senatorian == 3)slidePos = -120;

                if(Math.abs(gHMap.bone1.getCurrentPosition())-130 < 12) {
                    armPosition = 3;
                    armQueued = true;
                }
            }
//            else if (clawDist < 2 && hue > 50 && hue < 80 && clawDist > 0.5) {
//                toggle = true;
//                Singapore_Im_Senatorian++;
//                if(Singapore_Im_Senatorian > 7)slidePos = -130;
//                if(Math.abs(gHMap.bone1.getCurrentPosition()+130) < 3){
//                    armPosition = 3;
//                    armQueued = true;
//                }
//            }
            else{
                Singapore_Im_Senatorian = 0;
            }
            if (hue > 156 && hue < 164) {
                telemetry.addData("Color", "White");
                white++;
                if (white > 13) {
                    toggle = false;
                }
            } else {
                white = 0;
            }
        }


        //DEBUG CONTROL
        if(gamepad2.ps) {
            if (gamepad2.left_bumper) {//HSlide in
                HslidePos -= 0.02;
            }
            if (gamepad2.right_bumper) {//HSlide out
                HslidePos += 0.02;
            }


            if (gamepad2.right_stick_y > 0 && armPositionL > 0 && armPositionL < 0.77) {
                armPositionL -= gamepad2.right_stick_y * 0.01;
                jointPos(armPositionL, armPositionH);
            }

            if (gamepad2.left_stick_y > 0 && armPositionH > 0 && armPositionH < 1) {
                armPositionH = gamepad2.left_stick_y * 0.01;
                jointPos(armPositionL, armPositionH);
            }

            if (gamepad2.dpad_down) {
                gHMap.wrist.setPosition(0.18);
                gHMap.wrist2.setPosition(0.18);
            }
            if (gamepad2.dpad_up) {
                gHMap.wrist.setPosition(0);
                gHMap.wrist2.setPosition(0);
            }

        }

            // Toggle logic
            if (gamepad1.dpad_down && !lastTriangle2) {
                triangle2  = !triangle2 ;
            }
            // Update the previous button state
            lastTriangle2 = gamepad1.dpad_down;





             ;


        if(gamepad2.start && gamepad2.back){
            gHMap.bone1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            gHMap.bone3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            gHMap.bone1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            gHMap.bone3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slidePos = 0;
        }
        if(triangle2){
                gamepad1.setLedColor(0,255,0,10000);
            }
            else{
                gamepad1.setLedColor(255,0,0,10000);
            }











        elapsedTime = currentTime - armStartTime;

        i++;

        //-------------- end of bot functions -----------------
        telemetry.addData("Gibbs", gibbs);
        telemetry.addData("Power", power);
        telemetry.addData("DOWN?", vSlideLowering);
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
        telemetry.addData("SlideTime", slideStartTime);
        telemetry.addData("TIME", currentTime);
//                telemetry.addData("armStartTime", armStartTime);
        telemetry.addData("armTime", armTime);
        telemetry.addData("interm Joint", intermJoint);
        telemetry.addData("iterator", i);
        telemetry.addData("Processing Speed", i/((double)elapsedTime/1000));
        telemetry.addData("GAME COLOR", col);

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
