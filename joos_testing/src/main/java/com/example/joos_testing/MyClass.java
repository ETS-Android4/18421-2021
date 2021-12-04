package com.example.joos_testing;

import com.amarcolini.joos.geometry.Pose2d;
import com.amarcolini.joos.geometry.Vector2d;
import com.amarcolini.joos.gui.GUI;
import com.amarcolini.joos.gui.style.Themes;
import com.amarcolini.joos.gui.trajectory.WaypointBuilder;
import com.amarcolini.joos.path.QuinticSpline;
import com.amarcolini.joos.trajectory.config.TankConstraints;

public class MyClass {
    public static void main(String[] args) {
        new GUI()
                .followTrajectory(
                        new WaypointBuilder(
                                new Pose2d(-39.00, 62.40, -1.57080),
                                -1.57080,
                                new TankConstraints(
                                        312.0,
                                        18.0,
                                        30.0,
                                        30.0,
                                        Math.toRadians(90.0),
                                        Math.toRadians(100.0)
                                )
                        )
                                .splineTo(new Vector2d(-54.85, 52.30), Math.toRadians(180.0))
                                .wait(2.0)
                                .back(26.322604207555816)
                                .splineTo(new Vector2d(-13.95, 42.80), Math.toRadians(-90.0))
                                .wait(2.0)
                                .splineTo(new Vector2d(46.85, 37.85), 0.0)
                                .build()
                )
                .start();
    }
}