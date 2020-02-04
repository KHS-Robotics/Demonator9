package frc.robot.vision;

import java.util.ArrayList;

import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;

public class PixyCam {
	private final static double xHalf = (315 / 2);
    private final static double yHalf = (207 / 2);
    // (0, 0) --> (315, 0)
    // (0, 0) --> (0, 207)

    private final static double sizeWeight = 0.01;
	private final static double distWeight = 0.01;
	
	public static double calcWeight(double dist, double size) {
		double val = (sizeWeight * size) + (distWeight * dist);
		return val;
	}

	public static double getDist(Block args) {
		double totalDist = Math.sqrt(Math.pow((args.getX() - xHalf), 2) + Math.pow((args.getY() - yHalf), 2));
		return totalDist;
	}

	public static int getSize(Block args) {
		return (args.getWidth() * args.getHeight());
	}

	public static double getSize(ColorBlocks args) {
		return (args.getWidth() * args.getLength());
	}

	public static ArrayList<Block> sortByCenter(ArrayList<Block> args) {
		args.sort((a, b) -> Double.compare(Math.abs(xHalf - a.getX()) + Math.abs(yHalf - a.getY()),
			Math.abs(yHalf - b.getX()) + Math.abs(yHalf - b.getY())));
		return args;
	}

	public static ArrayList<Block> getBlocks(Pixy2 pixy) {
		pixy.getCCC().getBlocks(true, 255, 8);
		ArrayList<Block> blocks = pixy.getCCC().getBlocks();

		return blocks;
	}
}