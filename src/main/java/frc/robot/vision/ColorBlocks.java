package frc.robot.vision;

import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;

public class ColorBlocks {
	private double size, weight, distance;
	private Block block;
	
	public ColorBlocks(Block block) {
		this.block = block;
		update();
	}

	public void update() {
		size = PixyCam.getSize(block);
		distance = PixyCam.getDist(block);
		weight = PixyCam.calcWeight(distance, size);
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
		return block.getHeight();
	}

	public double getWidth() {
		return block.getWidth();
	}
}