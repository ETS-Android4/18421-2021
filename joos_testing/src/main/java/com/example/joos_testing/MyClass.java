package com.example.joos_testing;

import com.amarcolini.joos.geometry.Vector2d;
import com.amarcolini.joos.gui.GUI;
import com.amarcolini.joos.gui.trajectory.WaypointBuilder;
import com.amarcolini.joos.path.QuinticSpline;

public class MyClass {
    public static void main(String[] args) {
        new QuinticSpline(
                new QuinticSpline.Knot(0.0, 0.0),
                new QuinticSpline.Knot(30.0, -30.0)
        );
    }
}