package org.firstinspires.ftc.teamcode.OpModes;

import android.graphics.Color;

import java.util.ArrayList;
import java.util.Objects;


import com.acmerobotics.dashboard.config.Config;


import com.pedropathing.util.Timer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.ControlSystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@Config
public class GeneralHardwareMap {

    //h


    //Define runtime
    public ElapsedTime runtime = new ElapsedTime();
//    SP stdPos = new SP();

    //Define opMode

    public OpMode opMode;


    //Define all hardware
    public VoltageSensor batteryVoltageSensor;
    AnalogInput encoder;
    public DcMotor frontLeft, frontRight, backLeft, backRight;
    public DcMotor slideLength, slideAngle, hangL, hangR;
    public DcMotor bone1,bone2,bone3;
    public Servo clawAngleH, clawAngleVL, clawAngleVR, claw;
    public Servo slideAngleR, slideAngleL, ex, ex2;
    public Servo wrist, wrist2;
    public WebcamName bonoboCam;
    public HuskyLens husky;
    public DistanceSensor distanceSensor;
    public ColorSensor colorSensorCenter;
    public ColorSensor colorSensorLeft;
    public ColorSensor colorSensorRight;
    public BNO055IMU gyro;//Can we do it?
    public Limelight3A limelight;
    public Servo limeServo;
    public Servo zero, zero2, zero3, zero4, zero5;
    public CRServo zero8;
    public Servo lowJointLeft, lowJointRight, highJointLeft, highJointRight;
    public ColorRangeSensor SlurpSense, ClawSense, INTERCLAW;

    public boolean halfSpeedToggle = true;
    public boolean aLast = false;
    float[] HSVVALUES = new float[3];


    public boolean drivingReverse = false;
    public boolean yLast = false;


    public double LOW0;
    public double LOWF;
    public double HIGHF;
    public int colorDetect = 0;
    public int yellowDetect = 0;
    public boolean CLAWBEHAVIOR;
    public double yMovement;
    public double xMovement;
    public double rotation;
    public double drivePower;
    public double slidePower;
    public double lowerSlideLim = 0;
    public double upperSlideLim = 2600;
    public static double pos;
    public static double lowerSpecimen = 12;
    public static double upperSpecimen = 24;
    public static double slideTickPerInch = 100;
    public static double slideAngleTickPerDegree = 0.009111111;//3.95833
    public static double minSlideLength = 12;
    public Telemetry telemetry;
    public static double stackNum = 0;
    Timer COLORTIME = new Timer();

    public GeneralHardwareMap(OpMode opMode) {
        this.opMode = opMode;
    }


    public GeneralHardwareMap(LinearOpMode opMode) {this.opMode = opMode;}
//    public void initSTDPoints(String Color, String Side){stdPos.initializeStandardPoints(Side, Color);}
    public void resetSlideEncoders(){slideLength.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); slideLength.setPower(1);}
    public void resetSlideAngleEncoders(){slideAngle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); slideAngle.setPower(1);}
    public void lockArm2Height(double Height, double Length){
        double theta = Math.toDegrees(Math.atan((double) Height / Length));
        if (theta > -10 && theta < 100){
            theta = (theta*slideAngleTickPerDegree + 0.16);
            slideAngleL.setPosition(theta);
            slideAngleR.setPosition(theta);
        }
        double SlideLength = Math.sqrt(Math.pow(Height,2)+Math.pow(Length,2));
        if (SlideLength > 17 && SlideLength < 53 && Length < 34){//54
            SlideLength = ((SlideLength - 17) * slideTickPerInch);
            slideLength.setTargetPosition((int)SlideLength);
        }
    }

    public void SetArm2Angle(double theta){
        if (theta > -10 && theta < 100){
            theta = (theta*slideAngleTickPerDegree + 0.16);
            slideAngleL.setPosition(theta);
            slideAngleR.setPosition(theta);
        }}
    public void SetArmExtend2Length(int Extend){
        Extend = (int)(Extend* slideTickPerInch);
        slideLength.setTargetPosition(Extend);
    }

    public void initRANDOMOTOR(String motorName) {
        slideAngle = this.opMode.hardwareMap.dcMotor.get(motorName);
        slideAngle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
    public void initRANDOMOTOR2(String motorName) {
        slideLength = this.opMode.hardwareMap.dcMotor.get(motorName);
        slideLength.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);}
    public void openClaw(){zero5.setPosition(0);}
    public void closeClaw(){zero5.setPosition(0.24);}

    public void jointPos(double lowerAngle, double upperAngle){
        zero.setPosition(lowerAngle);
        zero2.setPosition(lowerAngle);
        zero4.setPosition(upperAngle);
        zero3.setPosition(upperAngle);
    }


        /*
        public double getColor() {
            int red = colorSensor.red();
            int blue = colorSensor.blue();
            int green = colorSensor.green();
            if(red > 200 && green > 200 && blue < 100){return 1;}//Yellow
            if(red > 200 && green < 100 && blue < 100){return 2;}//Red
            if(red < 100 && green < 100 && blue > 200){return 3;}//Blue
            //if(red < 100 && green < 100 && blue > 200){return 1;}MAKE A GREY
            else{return 0;}

        }
        */
    public void initLL() {
        limeServo = this.opMode.hardwareMap.servo.get("Lime");
        limeServo.setPosition(0.5);
        limelight = this.opMode.hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.start();

    }
public void upperGrabberJoint(double g){}

    public void init(String opModeType) {

        //Always intialize these
        //huskyLens = this.opMode.hardwareMap.get(HuskyLens.class, "huskylens");
        //distanceSensor = this.opMode.hardwareMap.get(DistanceSensor.class, "distanceSensor");
//            colorSensorCenter = this.opMode.hardwareMap.get(ColorSensor.class, "colorSensorCenter");
//            colorSensorLeft = this.opMode.hardwareMap.get(ColorSensor.class, "colorSensorRight");
//            colorSensorRight = this.opMode.hardwareMap.get(ColorSensor.class, "colorSensorLeft");

        VoltageSensor batteryVoltageSensor = this.opMode.hardwareMap.voltageSensor.iterator().next();

        pos = 0.8;
        //Initialize motors only if in teleOp
        if (opModeType.equals("TELEOP")) {

            frontLeft = this.opMode.hardwareMap.dcMotor.get("leftFront");
            frontLeft.setDirection(DcMotor.Direction.REVERSE);
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            frontRight = this.opMode.hardwareMap.dcMotor.get("rightFront");
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            backLeft = this.opMode.hardwareMap.dcMotor.get("leftBack");
            backLeft.setDirection(DcMotor.Direction.REVERSE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            backRight = this.opMode.hardwareMap.dcMotor.get("rightBack");
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            slideAngle = this.opMode.hardwareMap.dcMotor.get("slideAngle");
            slideAngle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slideAngle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideAngle.setTargetPosition(0);
            slideAngle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideAngle.setDirection(DcMotorSimple.Direction.REVERSE);
            slideAngle.setTargetPosition(0);

            slideLength = this.opMode.hardwareMap.dcMotor.get("slideLength");
            slideLength.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slideLength.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideLength.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideLength.setDirection(DcMotorSimple.Direction.REVERSE);
            slideLength.setTargetPosition(0);

            clawAngleVL = this.opMode.hardwareMap.servo.get("clawAngleVL");
            clawAngleVL.setPosition(0);

            clawAngleVR = this.opMode.hardwareMap.servo.get("clawAngleVR");
            clawAngleVR.setDirection(Servo.Direction.REVERSE);
            clawAngleVR.setPosition(0);

            clawAngleH = this.opMode.hardwareMap.servo.get("clawAngleH");
            //clawAngleH.setPosition(0.44);
            clawAngleH.setPosition(1);

            claw = this.opMode.hardwareMap.servo.get("claw");
            claw.scaleRange(0.4, 0.7);
            claw.setPosition(1);
            //claw.setPosition(0);

            slideAngleR = this.opMode.hardwareMap.servo.get("slideAngleR");
            slideAngleR.setPosition(0.54);

            slideAngleL = this.opMode.hardwareMap.servo.get("slideAngleL");
            slideAngleL.setDirection(Servo.Direction.REVERSE);
            slideAngleL.setPosition(0.54);

            hangL = this.opMode.hardwareMap.dcMotor.get("hangL");
            hangL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hangL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hangL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hangL.setDirection(DcMotorSimple.Direction.REVERSE);

            hangR = this.opMode.hardwareMap.dcMotor.get("hangR");
            hangR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hangR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hangR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            husky = this.opMode.hardwareMap.get(HuskyLens.class, "husky");
            encoder = this.opMode.hardwareMap.get(AnalogInput.class, "encoder");
            distanceSensor = this.opMode.hardwareMap.get(DistanceSensor.class, "distanceSensor");
        }
        if(opModeType.equals("Matt")) {
            slideAngle = this.opMode.hardwareMap.dcMotor.get("slideAngle");
            slideAngle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slideAngle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideAngle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideAngle.setDirection(DcMotorSimple.Direction.REVERSE);
            slideAngle.setTargetPosition(0);

            slideLength = this.opMode.hardwareMap.dcMotor.get("slideLength");
            slideLength.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            slideLength.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideLength.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideLength.setTargetPosition(0);


        }
        if(opModeType.equals("MattyPatty")) {
//            slideAngle = this.opMode.hardwareMap.dcMotor.get("slideAngle");
//            slideAngle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            slideAngle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            slideLength = this.opMode.hardwareMap.dcMotor.get("slideLength");
            slideLength.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slideLength.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        }
//Left servos on outtake needs reverse direction
        //0.6 for claw angle
        if(opModeType.equals("EX")) {
//            slideAngle = this.opMode.hardwareMap.dcMotor.get("slideAngle");
//            slideAngle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            slideAngle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            ex = this.opMode.hardwareMap.servo.get("extend");//max range 0.28 or 0.29
//            ex.setPosition(0.0);
            //ex.scaleRange(0.0,0.26);
            //1 is port 0 and 5 is port 4
            zero  = this.opMode.hardwareMap.servo.get("4");//Lower Joint L
            zero2  = this.opMode.hardwareMap.servo.get("3");//Lower Joint
//            zero3  = this.opMode.hardwareMap.servo.get("2");//UpperJoint L
            zero4  = this.opMode.hardwareMap.servo.get("5");//UpperJoint
            zero5  = this.opMode.hardwareMap.servo.get("1");//Claw
            zero8 = this.opMode.hardwareMap.crservo.get("zero");//Slurp
            wrist = this.opMode.hardwareMap.servo.get("wrist");


            zero2.setPosition(0);
            //zero.setDirection(Servo.Direction.REVERSE);//REAL
            zero2.setDirection(Servo.Direction.REVERSE);
//            zero3.setDirection(Servo.Direction.REVERSE);
//            zero4.setDirection(Servo.Direction.REVERSE);
//            zero5.setDirection(Servo.Direction.REVERSE);
            wrist.setDirection(Servo.Direction.REVERSE);

            ex2  = this.opMode.hardwareMap.servo.get("extend2");
            ex2.setDirection(Servo.Direction.REVERSE);
            //ex2.scaleRange(0.03,0.29);
//            ex2.setPosition(0);
            //ex2.setDirection(Servo.Direction.REVERSE);//IF NEEDED
            //wrist  = this.opMode.hardwareMap.servo.get("wrist");
//            wrist.setPosition(0);
            //wrist2  = this.opMode.hardwareMap.servo.get("wrist2");
            //wrist2.setPosition(0);
            //wrist2.setDirection(Servo.Direction.REVERSE);IF NEEDED

            bone1 = this.opMode.hardwareMap.dcMotor.get("bone1");
            bone1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            bone1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bone2 = this.opMode.hardwareMap.dcMotor.get("bone2");
            bone2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            bone2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bone3 = this.opMode.hardwareMap.dcMotor.get("bone3");
            bone3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            bone3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        }

        if(opModeType.equals("Kineses")) {


            ex = this.opMode.hardwareMap.servo.get("extend");

            ex2  = this.opMode.hardwareMap.servo.get("extend2");
            ex.setDirection(Servo.Direction.REVERSE);
            ex.scaleRange(0.0,0.05);
            ex2.scaleRange(0.0,0.05);
            ex2.setPosition(0);
            ex.setPosition(0.0);
            zero  = this.opMode.hardwareMap.servo.get("4");//Lower Joint
            zero2  = this.opMode.hardwareMap.servo.get("3");//Lower Joint L
            zero3  = this.opMode.hardwareMap.servo.get("2");//UpperJoint L
            zero4  = this.opMode.hardwareMap.servo.get("5");//UpperJoint
            zero5  = this.opMode.hardwareMap.servo.get("1");
            zero8 = this.opMode.hardwareMap.crservo.get("zero");//Slurp



//            zero.setDirection(Servo.Direction.REVERSE);
            zero.setPosition(0);
            zero.setDirection(Servo.Direction.REVERSE);
            zero2.setPosition(0);

            zero3.setDirection(Servo.Direction.REVERSE);


            zero3.setPosition(0);
            zero4.setPosition(0);
            zero5.setPosition(0);



            //ex2.setDirection(Servo.Direction.REVERSE);IF NEEDED
            wrist  = this.opMode.hardwareMap.servo.get("wrist");

//            wrist.setDirection(Servo.Direction.REVERSE);
            wrist.setPosition(0);
            wrist2  = this.opMode.hardwareMap.servo.get("wrist2");
            wrist2.setDirection(Servo.Direction.REVERSE);
            //wrist2.setPosition(0);
            //wrist2.setDirection(Servo.Direction.REVERSE);IF NEEDED
//            lowJointLeft  = this.opMode.hardwareMap.servo.get("lowJointLeft");
//            lowJointRight  = this.opMode.hardwareMap.servo.get("lowJointRight");
//            highJointLeft    = this.opMode.hardwareMap.servo.get("highJointLeft");
//            highJointRight  = this.opMode.hardwareMap.servo.get("highJointRight");

            bone1 = this.opMode.hardwareMap.dcMotor.get("bone1");
            bone1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            bone1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            bone2 = this.opMode.hardwareMap.dcMotor.get("bone2");
            bone2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            bone2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            bone3 = this.opMode.hardwareMap.dcMotor.get("bone3");
            bone3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            bone3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//BACKUP
//            int dEnc = 10;
//            int lastEnc = 0;
//            Timer iterator = new Timer();
//            iterator.resetTimer();
//            while((dEnc > 1 || iterator.getElapsedTimeSeconds() < 0.5) && iterator.getElapsedTimeSeconds()<1.5){
//                bone1.setPower(0.5);
////                bone2.setPower(0.5);
//                bone3.setPower(0.5);
//                dEnc = Math.abs(Math.abs(bone1.getCurrentPosition()) - lastEnc);
//                lastEnc = Math.abs(bone1.getCurrentPosition());
//
//            }
//            bone1.setPower(0);
//            bone3.setPower(0);
//            while(iterator.getElapsedTimeSeconds() < 2.5) {
//                if(iterator.getElapsedTimeSeconds()>2) {
//                    for (int i = 0; i < 20; i++) {
//                        bone1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                        bone2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                        bone3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                    }
//                }
//            }


            //AFTER AUTOMATIC ENCODER RESET -----------

            bone1.setTargetPosition(0);
            bone1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bone1.setPower(0.6);

            bone2.setTargetPosition(0);
            bone2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bone2.setPower(0.6);

            bone3.setTargetPosition(0);
            bone3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bone3.setPower(0.6);

//            ClawSense = this.opMode.hardwareMap.get(ColorSensor.class, "clawSense");
            ClawSense = this.opMode.hardwareMap.get(ColorRangeSensor.class, "clawSense");
//            SlurpSense = this.opMode.hardwareMap.get(ColorSensor.class, "slurpSense");
            SlurpSense = this.opMode.hardwareMap.get(ColorRangeSensor.class, "slurpSense");
            zero.setPosition(0.005);
            zero2.setPosition(0.005);

//0.09 and 0.18 is pos for transfer

        }







        //telemetry.addData("Battery Voltage: ", batteryVoltageSensor.getVoltage());
        //telemetry.update();
    }

    public double averageLastContents(ArrayList<Double> arr, int LOOKBACK){
        int len = arr.size();
        int count = Math.min(len, LOOKBACK);
        double sum = 0;
        for(int i = len - count; i < len; i++){
            sum += arr.get(i);
        }
        return sum/count;
    }

    public double getServoPos() {
        return(encoder.getVoltage() / 3.2 * 360);
    }

//        public void servoRunToPosition(double pos) {
//            if (getServoPos() < pos) {
//                slideAngleL.setPower(-1);// - Math.pow(2.5, (double)(getServoPos() - pos) / (10000)));
//                slideAngleR.setPower(1);// + Math.pow(2.5, (double)(getServoPos() - pos) / (10000)));
//            } else if (getServoPos() > pos){
//                slideAngleL.setPower(1);// + Math.pow(2.5, (double)(pos - getServoPos()) / (10000)));
//                slideAngleR.setPower(-1);// - Math.pow(2.5, (double)(pos - getServoPos()) / (10000)));
//            } else {
//                slideAngleL.setPower(0);
//                slideAngleR.setPower(0);
//            }
//        }


    /*
    public void initForApril(){
        WebcamName bonoboCam = hardwareMap.get(WebcamName.class, "Webcam 1");
    }

     */

    public void freaktonomous(){
        double dY = -10;
        double iY = 22;
        double L = 12;
        int speed = 20;
        double inverse = (double) -1 /speed;
        for(int i = speed * -5; i < 7*speed; i++){
            lockArm2Height(dY/(1+Math.pow(Math.E, inverse*i))+iY, L);
        }
        lockArm2Height(iY+dY, L);
        openClaw();
    }


    public void smoothJoints(double low0, double lowF, double highF){
        double dY = lowF-low0;
//        double iY = low0;
        double T = 0.62;
        int speed = 40;
        double inverse = (double) -1 /speed;
        for(int i = speed * -5; i < 7*speed; i++){
           // lockArm2Height(dY/(1+Math.pow(Math.E, inverse*i))+low0, L);
            jointPos(dY/(1+Math.pow(Math.E, inverse*i))+low0,T);
        }
        jointPos(lowF, highF);
    }
    public void motorPos(int Pos){
        bone1.setTargetPosition(Pos);
        bone3.setTargetPosition(Pos);
    }
    public void HSlidePos(double Pos){
        ex.setPosition(Pos);
        ex2.setPosition(Pos);}
    public void WristPos(double Pos){
        wrist.setPosition(Pos);
        wrist2.setPosition(Pos);
    }
    public void autoExtend(){zero8.setPower(-1);HSlidePos(0.95);WristPos(0.18);}
    public void autoExtendHalf(){zero8.setPower(-1);HSlidePos(0.55);WristPos(0.18);}
    public void armHandler(int lastArmPos, int desiredArmPos){
        int SLIDEPOS0 = 0;
        int SLIDEPOSF = 0;
        double POWER = 1;
        if(lastArmPos != desiredArmPos){
            switch (lastArmPos) {
                case 0: // Transfer
                    //description = "Transfer position: Low joint = 0.41, High joint = 0.34";
                    LOW0 = 0.41;
                    break;

                case 1: // Sample
                    //description = "Sample position: Low joint = 0.0, High joint = 0.2";
                    LOW0 = 0.0;
                    POWER = 0.3;
                    break;

                case 2: // Wall Grab
                    //description = "Wall Grab position: Low joint = 0.77, High joint = 0.38";
                    LOW0 = 0.77;
                    break;

                case 3: // Spec Place Position
                    //description = "Spec Place position: Low joint = 0.24, High joint = 0.22";
                    LOW0 = 0.24;
                    break;

                default: // Catch-all for invalid cases
                    //description = "Invalid arm position! Please use a value between 0 and 3.";
                    break;
            }
            //DETERMINE THE LAST POSITION OF THE ARM

            switch (desiredArmPos) {
                case 0: // Transfer
                    //description = "Transfer position: Low joint = 0.41, High joint = 0.34";
                    LOWF = 0.41;
                    HIGHF = 0.34;
                    SLIDEPOS0 = -50;
                    SLIDEPOSF = -17;
                    break;

                case 1: // Sample
                    //description = "Sample position: Low joint = 0.0, High joint = 0.2";
                    LOWF = 0;
                    HIGHF = 0.2;
                    SLIDEPOS0 = -630;
                    SLIDEPOSF = -630;
                    break;

                case 2: // Wall Grab
                    //description = "Wall Grab position: Low joint = 0.77, High joint = 0.38";
                    LOWF = 0.77;
                    HIGHF = 0.38;
                    SLIDEPOS0 = -100;
                    SLIDEPOSF = -0;

                    break;

                case 3: // Spec Place Position
                    //description = "Spec Place position: Low joint = 0.24, High joint = 0.22";
                    LOWF = 0.24;
                    HIGHF = 0.2;
                    SLIDEPOS0 = -50;
                    SLIDEPOSF = -125;
                    break;

                default: // Catch-all for invalid cases
                    //description = "Invalid arm position! Please use a value between 0 and 3.";
                    break;
            }
            //DETERMINE DESIRED POSITION
            bone1.setPower(POWER);bone3.setPower(POWER);
            motorPos(SLIDEPOS0);
            smoothJoints(LOW0,LOWF,HIGHF);
            motorPos(SLIDEPOSF);
            //RUN THE CLAW TO THE DESIRED POINT

        }
    }

    public void autoColorBehavior(int CurrentArmPos, boolean Blue, String sensor, Telemetry telemetry){
        float hue;

        int i = 0;
        COLORTIME.resetTimer();
        boolean sample = false, specimen = false, CONTINUE = false;
        switch (sensor) {
            case "CLAW":
                if(CurrentArmPos == 2) {
                    while (!CONTINUE) {
                        Color.RGBToHSV(
                                (int) (ClawSense.red() * 255.0 / 1023),
                                (int) (ClawSense.green() * 255.0 / 1023),
                                (int) (ClawSense.blue() * 255.0 / 1023),
                                HSVVALUES
                        );
                        hue = HSVVALUES[0];
                        if (hue < 30 || hue > 330 || (hue > 50 && hue < 85) || (hue > 210 && hue < 270)) {
                            CONTINUE = true;
                            closeClaw();
                        }

                        i++;
                        if (i > 600) {
                            closeClaw();
                            CONTINUE = true;
                        }
                    }
                    armHandler(CurrentArmPos, 3);
                }

                break;

            case "SLURP":
                while(COLORTIME.getElapsedTimeSeconds() < 10 && yellowDetect < 1 && colorDetect < 1) {
                    Color.RGBToHSV(
                            (int) (SlurpSense.red() * 255.0 / 1023),
                            (int) (SlurpSense.green() * 255.0 / 1023),
                            (int) (SlurpSense.blue() * 255.0 / 1023),
                            HSVVALUES
                    );
                    hue = HSVVALUES[0];

                    if (hue < 30 || hue > 330) {//RED
                        if(Blue) {
                            zero8.setPower(1);
                        } else {colorDetect++;}
                    } else if (hue > 210 && hue < 270) {//BLUE
                        if(!Blue) {
                            zero8.setPower(1);
                        } else{colorDetect++;}
                    } else if (hue > 50 && hue < 85) {yellowDetect++;}
                    if (hue > 156 && hue < 164) {zero8.setPower(-1);}

                    if (yellowDetect > 0) {
                        wrist.setPosition(0.0);
                        wrist2.setPosition(0.0);
                        HSlidePos(0);sample = true;
                    }
                    else if (colorDetect > 0) {
                        wrist.setPosition(0.0);
                        wrist2.setPosition(0.0);
                        HSlidePos(0);specimen = true;
                    }
                    telemetry.addData("HUE", hue);
                    telemetry.update();
                }
                while(!CONTINUE && COLORTIME.getElapsedTimeSeconds() < 20){
                    zero8.setPower(0);
                    Color.RGBToHSV(
                            (int) (ClawSense.red() * 255.0 / 1023),
                            (int) (ClawSense.green() * 255.0 / 1023),
                            (int) (ClawSense.blue() * 255.0 / 1023),
                            HSVVALUES
                    );
                    hue = HSVVALUES[0];
                    if(hue < 30 || hue > 330 || (hue > 50 && hue < 85) || (hue > 210 && hue < 270)){
                        CONTINUE = true;
                        closeClaw();
                    }
                    closeClaw();
//                    if(i>600){
//                        closeClaw();CONTINUE = true;
//                    }
                    telemetry.addData("HUE", hue);
                    telemetry.update();
                }
//                if(specimen)armHandler(CurrentArmPos, 2);
//                else if(sample)armHandler(CurrentArmPos, 1);
//                else armHandler(CurrentArmPos, 1);
                armHandler(CurrentArmPos, 1);



                break;

            default: // Catch-all for invalid cases
                //description = "Invalid arm position! Please use a value between 0 and 3.";
                break;
        }




    }

}