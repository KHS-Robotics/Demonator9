package frc.robot.vision;

import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;

public class ColorBlock {
	private double size, weight, distance, x, y, width, height;
	int sig;

	private final static double sizeWeight = 0.01;
	private final static double distWeight = 0.01;

	public final static double xHalf = (315 / 2);
	public final static double yHalf = (207 / 2);
	// (0, 0) --> (315, 0)
    // (0, 0) --> (0, 207)
	
	public ColorBlock(double x, double y, double width, double height, int sig) {
		this.x = x;
		this.y = y;
		this.width = width;
		this.height = height;
		this.sig = sig;
	}

	public ColorBlock(Block block) {
		x = block.getX();
		y = block.getY();
		width = block.getWidth();
		height = block.getHeight();
		sig = block.getSignature();
	}

	public void update() {
		size = calcSize();
		distance = calcDist();
		weight = calcWeight(distance, size);
	}

	public double calcWeight(double dist, double size) {
		double val = (sizeWeight * size) + (distWeight * dist);
		return val;
	}

	public double calcDist() {
		double totalDist = Math.sqrt(Math.pow((x - xHalf), 2) + Math.pow((y - yHalf), 2));
		return totalDist;
	}

	public double calcSize() {
		return width * height;
	}

	public double getSize(ColorBlock args) {
		return (args.getWidth() * args.getLength());
	}

	public double getSize() {
		return size;
	}

	public double getDist() {
		return distance;
	}

	public double getWeight() {
		return weight;
	}

	public double getLength() {
		return height;
	}

	public double getWidth() {
		return width;
	}

	public int getSig() {
		return sig;
	}
}