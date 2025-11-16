package org.firstinspires.ftc.teamcode.Auto;


import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class newAuto extends OpMode{


        private TelemetryManager panelsTelemetry; // Panels Telemetry instance
        public Follower follower; // Pedro Pathing follower instance
        private int pathState; // Current autonomous path state (state machine)
        private Paths paths; // Paths defined in the Paths class
        private Timer pathTimer;
    public PathChain scorePreload, scorepickup1, pickup1score, scorepickup2, pickup2score, scorepickup3, pickup3score, scorestart;


    @Override
        public void init() {
            panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

            paths = new Paths(follower); // Build paths

            panelsTelemetry.debug("Status", "Initialized");
            panelsTelemetry.update(telemetry);
        }

        @Override
        public void loop() {
            follower.update(); // Update Pedro Pathing
            pathState = autonomousPathUpdate(); // Update autonomous state machine
            //autonomousPathUpdate();

            // Log values to Panels and Driver Station
            panelsTelemetry.debug("Path State", pathState);
            panelsTelemetry.debug("X", follower.getPose().getX());
            panelsTelemetry.debug("Y", follower.getPose().getY());
            panelsTelemetry.debug("Heading", follower.getPose().getHeading());
            panelsTelemetry.update(telemetry);

        }

        public  class Paths {

            public Paths(Follower follower) {
                scorePreload = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(77.000, 7.000), new Pose(107.000, 106.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                        .build();

                scorepickup1 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(107.000, 106.000), new Pose(105.000, 84.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                        .build();

                pickup1score = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(105.000, 84.000), new Pose(107.000, 106.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                        .build();

                scorepickup2 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(107.000, 106.000), new Pose(105.000, 60.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                        .build();

                pickup2score = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(105.000, 60.000), new Pose(107.000, 106.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                        .build();

                scorepickup3 = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(107.000, 106.000), new Pose(105.000, 35.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                        .build();

                pickup3score = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(105.000, 35.000), new Pose(107.000, 106.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                        .build();
                scorestart = follower
                        .pathBuilder()
                        .addPath(
                                new BezierLine(new Pose(107.000, 106.000), new Pose(77.00, 7.000))
                        )
                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                        .build();
            }
        }

    public void setPathState(int pState) {
        pathState = pState;
        //pathTimer.resetTimer();
    }

        public int autonomousPathUpdate() {
            // Add your state machine Here
            // Access paths with paths.pathName
            // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
            switch (pathState) {
                case 0:
                    follower.followPath(scorePreload);
                    setPathState(1);
                    break;
                case 1:
                    if (!follower.isBusy()) {
                        follower.followPath(scorepickup1, true);
                        setPathState(2);
                    }
                    break;
                case 2:
                    if (!follower.isBusy()) {
                        follower.followPath(pickup1score, true);
                        setPathState(3);
                    }
                    break;
                case 3:
                    if (!follower.isBusy()) {
                        follower.followPath(scorepickup2, true);
                        setPathState(4);
                    }
                    break;
                case 4:
                    if (!follower.isBusy()) {
                        follower.followPath(pickup2score, true);
                        setPathState(5);
                    }
                    break;
                case 5:
                    if (!follower.isBusy()) {
                        follower.followPath(scorepickup3, true);
                        setPathState(6);
                    }
                    break;
                case 6:
                    if (!follower.isBusy()) {
                        follower.followPath(pickup3score, true);
                        setPathState(7);
                    }
                case 7:
                    if (!follower.isBusy()) {
                        follower.followPath(scorestart, true);
                        setPathState(8);
                    }
                    break;
                case 8:
                    if (!follower.isBusy()) {

                        setPathState(-1);
                    }
                    break;



            }
            return pathState;

        }
}