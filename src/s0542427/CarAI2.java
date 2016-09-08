/*
 * DummyAI
 */

package s0542427;

import lenz.htw.ai4g.ai.AI;
import lenz.htw.ai4g.ai.Acceleration;
import lenz.htw.ai4g.ai.Info;

import javax.swing.plaf.basic.BasicInternalFrameTitlePane.MaximizeAction;

import org.lwjgl.opengl.GL11;

public class CarAI2 extends AI {
	
	public CarAI2(Info info){
		super(info);
	}
	
	public String getName() {
		return "Dummy";
	}

	public Acceleration update(boolean arg0) {
		return new Acceleration(0, 0);
	}
	
	private void checkVertex() {
		
	}


	

	
}
