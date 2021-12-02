package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.amarcolini.joos.geometry.Vector2d;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import java.util.HashMap;

import org.opencv.core.*;
import org.opencv.core.Core.*;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.*;
import org.opencv.objdetect.*;
import org.openftc.easyopencv.OpenCvPipeline;

import kotlin.Pair;

@Config
public class DuckDetector extends OpenCvPipeline {

	//Outputs
	private final Mat hslThresholdOutput = new Mat();
	private final Mat pointSelectionOutput = new Mat();
	private Position position = Position.ONE;
	public static Vector2d location1 = new Vector2d();
	public static Vector2d location2 = new Vector2d();
	public static Vector2d location3 = new Vector2d();
	public static Mode mode = Mode.POINTS;

	public enum Mode {
		RAW, THRESHOLD, POINTS
	}

	public enum Position {
		ONE, TWO, THREE
	}


	/**
	 * Segment an image based on hue, saturation, and luminance ranges.
	 *
	 * @param input The image on which to perform the HSL threshold.
	 * @param hue The min and max hue
	 * @param sat The min and max saturation
	 * @param lum The min and max luminance
	 * @param out The image in which to store the output.
	 */
	private void hslThreshold(Mat input, double[] hue, double[] sat, double[] lum,
		Mat out) {
		Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2HLS);
		Core.inRange(out, new Scalar(hue[0], lum[0], sat[0]),
			new Scalar(hue[1], lum[1], sat[1]), out);
	}


	@Override
	public Mat processFrame(Mat input) {
		// Compute threshold on input:
		double[] hslThresholdHue = {17.805755395683452, 37.89473684210526};
		double[] hslThresholdSaturation = {144.46942446043167, 255.0};
		double[] hslThresholdLuminance = {0.0, 255.0};
		hslThreshold(input, hslThresholdHue, hslThresholdSaturation, hslThresholdLuminance, hslThresholdOutput);

		// Find corresponding barcode location:
		final Moments moments = Imgproc.moments(hslThresholdOutput);
		final Vector2d duck = new Vector2d(
				moments.m10 / moments.m00,
				moments.m01 / moments.m00
		);
		final double d1 = duck.distTo(location1);
		final double d2 = duck.distTo(location2);
		final double d3 = duck.distTo(location3);
		final double max = Collections.max(Arrays.asList(d1, d2, d3));
		if (d1 == max) position = Position.ONE;
		else if (d2 == max) position = Position.TWO;
		else if (d3 == max) position = Position.THREE;

		// Draw locations:
		pointSelectionOutput.setTo(input);
		Imgproc.circle(pointSelectionOutput, new Point(location1.getX(), location1.getY()), 10, new Scalar(255, 0, 0));
		Imgproc.circle(pointSelectionOutput, new Point(location2.getX(), location2.getY()), 10, new Scalar(0, 255, 0));
		Imgproc.circle(pointSelectionOutput, new Point(location3.getX(), location3.getY()), 10, new Scalar(0, 0, 255));
		Imgproc.circle(pointSelectionOutput, new Point(duck.getX(), duck.getY()), 15, new Scalar(255, 255, 255), 3);

		switch (mode) {
			case THRESHOLD: return hslThresholdOutput;
			case POINTS: return pointSelectionOutput;
			default: return input;
		}
	}

	public Position getLastPosition() {
		return position;
	}
}

