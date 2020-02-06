package frc.robot.vision;

import java.util.ArrayList;

import frc.robot.RobotContainer;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;

public class PixyCam {
	public static ArrayList<Block> sortByCenter(ArrayList<Block> args) {
		args.sort(
				(a, b) -> Double.compare(Math.abs(ColorBlock.xHalf - a.getX()) + Math.abs(ColorBlock.yHalf - a.getY()),
						Math.abs(ColorBlock.yHalf - b.getX()) + Math.abs(ColorBlock.yHalf - b.getY())));
		return args;
	}

	public static ArrayList<ColorBlock> sortByWeight(ArrayList<ColorBlock> args) {
		args.sort((a, b) -> Double.compare(a.getWeight(), b.getWeight()));
		return args;
	}

	public static ColorBlock averageDupBlocks(ArrayList<Integer> sigs, ArrayList<Block> blocks) {
		double totalWidth = 0, totalHeight = 0, xAverage = 0, yAverage = 0, smallX, largeX, smallY, largeY;
		
		for (int i = 0; i < sigs.size(); i++) {
			totalWidth += blocks.get(sigs.get(i)).getWidth();
			totalHeight += blocks.get(sigs.get(i)).getHeight();
		}
		
		smallX = blocks.get(0).getX() - (blocks.get(0).getWidth() / 2.0);
		largeX = blocks.get(0).getX() + (blocks.get(0).getWidth() / 2.0);

		smallY = blocks.get(0).getY() - (blocks.get(0).getHeight() / 2.0);
		largeY = blocks.get(0).getY() + (blocks.get(0).getHeight() / 2.0);

		for (int i = 0; i < sigs.size(); i++) {
			xAverage += (blocks.get(sigs.get(i)).getWidth() / totalWidth) * blocks.get(i).getX();
			yAverage += (blocks.get(sigs.get(i)).getHeight() / totalHeight) * blocks.get(i).getY();

			if(blocks.get(i).getX() - (blocks.get(i).getWidth() / 2.0) < smallX) {
				smallX = blocks.get(i).getX() - (blocks.get(i).getWidth() / 2.0);
			} 
			if(blocks.get(i).getX() + (blocks.get(i).getWidth() / 2.0) > largeX) {
				largeX = blocks.get(i).getX() - (blocks.get(i).getWidth() / 2.0);
			}

			if(blocks.get(i).getY() - (blocks.get(i).getHeight() / 2.0) < smallY) {
				smallY = blocks.get(i).getY() - (blocks.get(i).getHeight() / 2.0);
			} 
			if(blocks.get(i).getY() + (blocks.get(i).getHeight() / 2.0) > largeY) {
				largeY = blocks.get(i).getY() - (blocks.get(i).getHeight() / 2.0);
			}
		}

		double width = largeX - smallX, height = largeY - smallY;

		return new ColorBlock(xAverage, yAverage, width, height, blocks.get(sigs.get(0)).getSignature());
	}

	public static ArrayList<Block> getBlocks() {
		Pixy2 pixy = RobotContainer.pixy;
		pixy.getCCC().getBlocks(true, 255, 8);
		ArrayList<Block> blocks = pixy.getCCC().getBlocks();
		ArrayList<Integer> sigs = new ArrayList<>();

		for (int i = 0; i < blocks.size(); i++) {
			sigs.add(blocks.get(i).getSignature());
		}

		for (int sig = 1; sig <= 4; sig++) {
			ArrayList<Integer> currentIdxs = new ArrayList<>();
			for (int i = 0; i < sigs.size(); i++) {
				if (sigs.get(i) == sig) {
					currentIdxs.add(i);
				}
			}

			averageDupBlocks(currentIdxs, blocks);
		}

		return blocks;
	}
}