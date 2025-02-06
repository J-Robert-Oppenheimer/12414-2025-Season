package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.OpModes.GeneralHardwareMap;

import java.util.ArrayList;
import java.util.List;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "TheSunGodRa")
public class EyeOfRa extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    public static double A = Math.tan(0.0776671517137);
    public static double h = 10.0;
    public static double n = 10.2;
    public static double b = Math.tan(0.775798852511);
    double x= 0;
    double SmoothY = 0;
    double SmoothX = 0;
    double calculatedX =0;
    double y;
    double calculatedY = 0;
    double theta = 0;
    double SmoothTheta = 0;
    ArrayList<Double> Xs = new ArrayList<>();
    ArrayList<Double> Ys = new ArrayList<>();
    ArrayList<Double> Thetas = new ArrayList<>();
    GeneralHardwareMap gHMap = new GeneralHardwareMap(this);
    LLResult result;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;



    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    public  Pose samplePose;// = new Pose(0, 0, Math.toRadians(180));
//    private final Pose scorePose0 = new Pose(37.5, 60, Math.toRadians(180));


    private Path coverSample, scorePreload, subP, subDrop, midP, midDrop, wallP, wallDrop, p1, p2, p3, p4, score1, score2, score3, score4, park;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */



    }//End of Build Paths

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                gHMap.limelight.pipelineSwitch(5);
                setPathState(1);
                break;


            case 1:
                /* LIME TARGETING MATH */
                result = gHMap.limelight.getLatestResult();
                if (result != null) {
                    if (result.isValid()) {
                        List<LLResultTypes.ColorResult> color = result.getColorResults();
                        if (!color.isEmpty()) {
                            x = color.get(0).getTargetXPixels()-300;
                            y = -1* (color.get(0).getTargetYPixels()-480);
                            theta = color.get(0).getTargetXDegrees();
                            calculatedY = (n + h/((y/480*(A+b))-b))*-1;
                            calculatedX = -1*Math.tan(Math.toRadians(theta))*(calculatedY+n);
                            //Outer Loop

                        }
                    }
                }
                Xs.add(calculatedX);
                Ys.add(calculatedY);
//                Thetas.add(theta);
                if(Ys.size() > 10) {
                    SmoothX = gHMap.averageLastContents(Xs, 8);
                    SmoothY = gHMap.averageLastContents(Ys, 8);
//                    calculatedY = (n + h / ((SmoothY / 480 * (A + b)) - b)) * -1;
//                    calculatedX = Math.tan(Math.toRadians(SmoothTheta)) * (SmoothY + n);
                }




                /* DECIDE NEXT PATH */
                if(pathTimer.getElapsedTimeSeconds() > 7.5) {
                    samplePose = new Pose(SmoothY+n-14, SmoothX+7, Math.toRadians(0));
                    coverSample = new Path(new BezierLine(new Point(startPose), new Point(samplePose)));
                    coverSample.setLinearHeadingInterpolation(startPose.getHeading(), samplePose.getHeading());
                    setPathState(2);
                }
                break;

            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    follower.followPath(coverSample,true);
                    setPathState(4);
                }
                break;

            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    gHMap.autoExtend();
                    if(pathTimer.getElapsedTimeSeconds() > 3) {
                        setPathState(6);
                    }
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */

                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    gHMap.autoExtend();
                    if(pathTimer.getElapsedTimeSeconds() > 0.5) {
                        setPathState(6);
                    }

                break;

            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */

                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    gHMap.WristPos(0);
                    gHMap.HSlidePos(0);
                if(pathTimer.getElapsedTimeSeconds() > 6) {
                    setPathState(8);
                }
                break;

            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(midDrop,true);
                    setPathState(10);
                }
                break;

            case 10:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(wallP,true);
                    setPathState(12);
                }
                break;

            case 12:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(wallDrop,true);
                    setPathState(14);
                }
                break;

            case 14:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(p1,true);
                    setPathState(16);
                }
                break;

            case 16:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(score1,true);
                    setPathState(18);
                }
                break;

            case 18:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(p2,true);
                    setPathState(20);
                }
                break;

            case 20:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(score2,true);
                    setPathState(22);
                }
                break;

            case 22:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(p3,true);
                    setPathState(24);
                }
                break;

            case 24:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(score3,true);
                    setPathState(26);
                }
                break;

            case 26:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(p4,true);
                    setPathState(28);
                }
                break;

            case 28:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(score4,true);
                    setPathState(30);
                }
                break;

            case 30:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(score3,true);
                    setPathState(32);
                }
                break;

            case 32:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(park,true);
                    setPathState(33);
                }
                break;

            case 33:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Level 1 Ascent */

                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/


    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        gHMap.initLL();
        gHMap.init("Ra");

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
//        gHMap.init("Kineses");
//        gHMap.armHandler(ArmPos,0);
//        ArmPos = 0;
//        while (opmodeTimer.getElapsedTimeSeconds() < 5){
//            if (opmodeTimer.getElapsedTimeSeconds() > 4) {
//                gHMap.closeClaw();
//            }
//        }


        //        gHMap.init("Kineses");
//        gHMap.smoothJoints(0,0.28,0.30);


        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }



    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}

