package s0542427;

import java.util.ArrayList;
import java.util.List;


public class Vertex implements Comparable<Vertex>{
	public int x,y;
	private boolean driveable;
	public double cost = 5;
	public double initalCost = 5;
	ArrayList<Vertex> adj;
	private Vertex previousPointer;
	
	
	public Vertex(int x,int y,boolean d){
		this.x = x;
		this.y = y;
		this.driveable = d;
		adj = new ArrayList<Vertex>();
		previousPointer = null;
	}
	
	public void addNeighbour(Vertex v){
		adj.add(v);
	}
	
	public boolean isDriveable(){
		return driveable;
	}
	
	public ArrayList<Vertex> getList(){
		return adj;
	}
	
	public Vertex getPrevious(){
		return previousPointer;
	}
	
	public void setPrevious(Vertex v){
		previousPointer = v;
	}

	public int compareTo(Vertex arg0) {
		if(this.cost > arg0.cost){
			return 1;
		}
		if(this.cost < arg0.cost){
			return -1;
		}
		return 0;
	}
	
}
