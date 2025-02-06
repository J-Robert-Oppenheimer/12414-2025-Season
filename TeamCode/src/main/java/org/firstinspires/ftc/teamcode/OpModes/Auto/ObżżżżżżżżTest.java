package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.OpModes.GeneralHardwareMap;

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

@Autonomous(name = "ObżżżżżżTest")
public class ObżżżżżżżżTest extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    GeneralHardwareMap gHMap = new GeneralHardwareMap(this);

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;
    private double LowJointPos;
    private double HighJointPos;
    public int ArmPos = 1;
    public double inter = 25;
    public double spacing = 1.5;
    public double specPlace = 67;


    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(9.5, 54.5, Math.toRadians(180));
    private final Pose scorePose0 = new Pose(37.5, specPlace, Math.toRadians(180));
    private final Pose scorePose0I = new Pose(inter, specPlace, Math.toRadians(180));
    private final Pose scorePose1 = new Pose(37.5, specPlace+spacing*2, Math.toRadians(180));
    private final Pose scorePose1I = new Pose(inter, specPlace+spacing*2, Math.toRadians(180));
    private final Pose scorePose2 = new Pose(37.5, specPlace+spacing*3, Math.toRadians(180));
    private final Pose scorePose2I = new Pose(inter, specPlace+spacing*3, Math.toRadians(180));
    private final Pose scorePose3 = new Pose(37.5, specPlace+spacing*4, Math.toRadians(180));
    private final Pose scorePose3I = new Pose(inter, specPlace+spacing*4, Math.toRadians(180));
    private final Pose scorePose4 = new Pose(37.5, specPlace+spacing*5, Math.toRadians(180));
    private final Pose scorePose4I = new Pose(inter, specPlace+spacing*5, Math.toRadians(180));
    private final Pose wall = new Pose(27, 22, Math.toRadians(310)); //Intake
    private final Pose wallD = new Pose(15.5, 22, Math.toRadians(310)); //Placement
    private final Pose mid = new Pose(22, 12, Math.toRadians(0)); //Intake
    private final Pose midD = new Pose(15.5, 12, Math.toRadians(0)); //Placement
    private final Pose sub = new Pose(22, 23, Math.toRadians(0)); //Intake
    private final Pose subD = new Pose(15.5, 23, Math.toRadians(0)); //Placement
    private final Pose pickup = new Pose(9.8, 22, Math.toRadians(0)); // Intake
    private final Pose parkPose = new Pose(9.8, 24, Math.toRadians(180));

    private Path scorePreload, subP, subDrop, midP, midDrop, wallP, wallDrop, p1, p2, p3, p4, park;
    private PathChain score1;
    private PathChain score2;
    private PathChain score3;
    private PathChain score4;

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
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose0)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose0.getHeading());

        subP = new Path(new BezierLine(new Point(scorePose0), new Point(sub)));
        subP.setLinearHeadingInterpolation(scorePose0.getHeading(), sub.getHeading());

        subDrop = new Path(new BezierLine(new Point(sub), new Point(subD)));
        subDrop.setLinearHeadingInterpolation(sub.getHeading(), subD.getHeading());

        midP = new Path(new BezierLine(new Point(subD), new Point(mid)));
        midP.setLinearHeadingInterpolation(subD.getHeading(), mid.getHeading());

        midDrop = new Path(new BezierLine(new Point(mid), new Point(midD)));
        midDrop.setLinearHeadingInterpolation(mid.getHeading(), midD.getHeading());

        wallP = new Path(new BezierLine(new Point(midD), new Point(wall)));
        wallP.setLinearHeadingInterpolation(midD.getHeading(), wall.getHeading());

        wallDrop = new Path(new BezierLine(new Point(wall), new Point(wallD)));
        wallDrop.setLinearHeadingInterpolation(wall.getHeading(), wallD.getHeading());

        p1 = new Path(new BezierLine(new Point(wallD), new Point(pickup)));
        p1.setLinearHeadingInterpolation(wallD.getHeading(), pickup.getHeading());


        score1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup), new Point(scorePose1I)))
                .setLinearHeadingInterpolation(pickup.getHeading(), scorePose1I.getHeading())
                .addPath(new BezierLine(new Point(scorePose1I), new Point(scorePose1)))
                .setConstantHeadingInterpolation(scorePose1.getHeading())
                .build();

        p2 = new Path(new BezierLine(new Point(scorePose1), new Point(pickup)));
        p2.setLinearHeadingInterpolation(scorePose1.getHeading(), pickup.getHeading());

//        score2 = new Path(new BezierLine(new Point(pickup), new Point(scorePose2)));
//        score2.setLinearHeadingInterpolation(pickup.getHeading(), scorePose2.getHeading());

        score2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup), new Point(scorePose2I)))
                .setLinearHeadingInterpolation(pickup.getHeading(), scorePose2I.getHeading())
                .addPath(new BezierLine(new Point(scorePose2I), new Point(scorePose2)))
                .setConstantHeadingInterpolation(scorePose1.getHeading())
                .build();

        p3 = new Path(new BezierLine(new Point(scorePose2), new Point(pickup)));
        p3.setLinearHeadingInterpolation(scorePose2.getHeading(), pickup.getHeading());

//        score3 = new Path(new BezierLine(new Point(pickup), new Point(scorePose3)));
//        score3.setLinearHeadingInterpolation(pickup.getHeading(), scorePose3.getHeading());
        score3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup), new Point(scorePose3I)))
                .setLinearHeadingInterpolation(pickup.getHeading(), scorePose3I.getHeading())
                .addPath(new BezierLine(new Point(scorePose3I), new Point(scorePose3)))
                .setConstantHeadingInterpolation(scorePose1.getHeading())
                .build();

        p4 = new Path(new BezierLine(new Point(scorePose3), new Point(pickup)));
        p4.setLinearHeadingInterpolation(scorePose3.getHeading(), pickup.getHeading());

//        score4 = new Path(new BezierLine(new Point(pickup), new Point(scorePose4)));
//        score4.setLinearHeadingInterpolation(pickup.getHeading(), scorePose4.getHeading());
        score4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup), new Point(scorePose4I)))
                .setLinearHeadingInterpolation(pickup.getHeading(), scorePose4I.getHeading())
                .addPath(new BezierLine(new Point(scorePose4I), new Point(scorePose4)))
                .setConstantHeadingInterpolation(scorePose1.getHeading())
                .build();

        park = new Path(new BezierLine(new Point(scorePose4), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose4.getHeading(), parkPose.getHeading());
    }//End of Build Paths

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
//                gHMap.closeClaw();
//                if(ArmPos != 3) {
//                    gHMap.armHandler(ArmPos, 3);
//                    ArmPos = 3;
//                }
                follower.followPath(scorePreload);
                setPathState(2);
                break;


            case 1:

//                if(pathTimer.getElapsedTimeSeconds() > 1) {gHMap.armHandler(ArmPos, 4);
//                }
//                if(pathTimer.getElapsedTimeSeconds() > 2) {setPathState(2);gHMap.armHandler(ArmPos, 0);
//                    ArmPos = 0;
//                    }
                break;

            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(subP,true);
                    setPathState(4);
                }
                break;

            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(subDrop,true);
                    setPathState(6);
                }
                break;

            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(midP,true);
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

