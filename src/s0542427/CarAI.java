package s0542427;

import lenz.htw.ai4g.ai.AI;
import lenz.htw.ai4g.ai.Acceleration;
import lenz.htw.ai4g.ai.Info;

import java.awt.Point;
import java.awt.Polygon;
import java.awt.Rectangle;
import java.awt.geom.Area;
import java.awt.geom.Line2D;
import java.awt.geom.Path2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.PriorityQueue;
import java.util.Stack;

import javax.swing.plaf.basic.BasicInternalFrameTitlePane.MaximizeAction;

import org.lwjgl.opengl.GL11;

//TODO PFAD GENERATION CHECKEN

public class CarAI extends AI {
	private Polygon[] obs;
	
	private float GOAL_RADIUS = 1f;
	private float BREAK_RADIUS = 10f;
	private float TIME_ANG = 0.9f;
	private float TIME_VEL = 1f;
	private int CELL_SPACES = 50; // Unterteilungen 
	private int SKIP_POINTS = 2;

	private float TOLERANCE = 0.22f;
	private float BREAK_ANGLE = 1.5f;
	
	private Point endGoal;
	private Point previousEndGoal;
	private Point currentGoal;
	private float currentX;
	private float currentY;
	private float currentOrientation;
	private float goalOrientation;

	private float angleVel;
	private float linVel;
	private	float angAccel ;
	
	private int trackWidth;
	private int trackHeight;
	private Vertex[][] cells;
	private Vertex pathVertex;
	private Vertex currentCheckpoint;
	private Stack<Vertex> path;
	private Stack<Vertex> pathCopy;
	private Vertex[] pathCopyArray;
	private int pathCounter;
	
	private int cellWidth;
	private int cellHeight;
	private int endGoalCell_X;
	private int  endGoalCell_Y;
	private Vertex endGoalVertex; 
	
	private Polygon[] fast;
	private Polygon[] slow;
	
	public CarAI(Info info){
		super(info);
		trackWidth = info.getTrack().getWidth();
		trackHeight = info.getTrack().getHeight();
		cellWidth = (trackWidth/CELL_SPACES);
		cellHeight = (trackHeight/CELL_SPACES);
		obs = info.getTrack().getObstacles();
		fast = info.getTrack().getFastZones();
		slow  = info.getTrack().getSlowZones();
		endGoal = info.getCurrentCheckpoint();
		previousEndGoal = new Point(0,0);
		path = new Stack<Vertex>();
		pathCopy = new Stack<Vertex>();
		makeGraph();
	}
	
	public String getName() {
		return "Long";
	}

	public Acceleration update(boolean arg0) {
		endGoal = info.getCurrentCheckpoint();
		currentX = info.getX();
		currentY = info.getY();
		int currentCell_X = Math.round(currentX/(trackWidth/CELL_SPACES));
		int currentCell_Y = Math.round(currentY/(trackHeight/CELL_SPACES));
		
		currentCell_X = clamp(currentCell_X);
		currentCell_Y = clamp(currentCell_Y);
		
		Vertex currentVertex = cells[currentCell_X][currentCell_Y];
//		System.out.println("CURRENT CELL : "+currentVertex.x+" "+currentVertex.y+" | "+currentX+" | "+currentY);
		
		if(previousEndGoal.x != endGoal.x && previousEndGoal.y != endGoal.y){
		//pathCounter = 0;
		 endGoalCell_X = (int) (endGoal.getX()/(trackWidth/CELL_SPACES));
		 endGoalCell_Y = (int) (endGoal.getY()/(trackHeight/CELL_SPACES));
		endGoalVertex = cells[endGoalCell_X][endGoalCell_Y];		
		
		checkVertex();
		
		findPath(currentVertex,endGoalVertex);
		currentCheckpoint = path.pop();
		//currentCheckpoint = pathCopyArray[0];
		currentGoal = new Point(currentCheckpoint.x*cellWidth+(cellWidth/2),currentCheckpoint.y*cellHeight+(cellHeight/2));
		previousEndGoal = (Point) endGoal.clone();
		
		}
		
//		System.out.println("GOAL CELL : "+currentCheckpoint.x+" "+currentCheckpoint.y+" | "+currentGoal.x+" | "+currentGoal.y);
//		System.out.println("END GOAL: "+endGoal.getX()+ " | "+endGoal.getY());
//		System.out.println("Path size: "+path.size());
		if(distance(currentX,currentY,currentGoal.x,currentGoal.y) < 20){
			//pathCounter++;
			//currentCheckpoint = pathCopyArray[pathCounter];
			if(path.size() <= 2){
				currentCheckpoint = endGoalVertex;
				currentGoal = endGoal;
			}
			else{
			if(path.size() < SKIP_POINTS){
				currentCheckpoint = path.pop();				
			}
			else{
				for(int i = 1; i < SKIP_POINTS;i++){
					path.pop();
				}
				currentCheckpoint = path.pop();
			}
			currentGoal = new Point(currentCheckpoint.x*cellWidth+(cellWidth/2),currentCheckpoint.y*cellHeight+(cellHeight/2));
			}
		}
		
//		if(clearLineToGoal() && currentGoal != endGoal)
//		{
//			currentGoal = endGoal;
//			System.out.println("GOAL IN SIGHT!");
//		}
//		else{System.out.println("GOAL NOT IN SIGHT");}
		
		
		currentOrientation = info.getOrientation();	
		goalOrientation = (float) Math.atan2(currentGoal.y-currentY, currentGoal.x-currentX);
		float orientationDiff = (goalOrientation-currentOrientation);
		if(orientationDiff > Math.PI){
			orientationDiff -= 2*Math.PI;
		}else if(orientationDiff < -Math.PI){
			orientationDiff += 2*Math.PI;
		}
		float orientationDiffABS = Math.abs(orientationDiff);
		float distanceDiff = (float) Math.sqrt(Math.pow((currentGoal.x-currentX),2)+Math.pow((currentGoal.y-currentY),2));
		
//		System.out.println("GOAL :"+goalOrientation);
//		System.out.println("CURRENT: "+currentOrientation);
//		System.out.println("DIFF: "+orientationDiff);
		// --- ALIGN ---
		
		if(orientationDiffABS > TOLERANCE){ //Toleranz
			linVel = 2f;
			if(orientationDiffABS > BREAK_ANGLE){ //Differenz größer als Abbremswinkel
				angleVel = info.getMaxAngularVelocity()*Math.signum(orientationDiff);	
//				System.out.println("ALIGN: MAX SPIN");
			}
			else{	
				angleVel = (Math.signum(orientationDiff) * (info.getMaxAngularVelocity())/BREAK_ANGLE);
//				System.out.println("ALIGN: BELOW BREAK ANGLE");
				//System.out.println("BREAKANGLE   " + orientationDiffABS + "   " + angleVel);
			}
	//		System.out.println("angleVel "+angleVel);
	//		System.out.println("orientationDiff "+orientationDiff);
			
		}
		else{
			// --- ARRIVE ---
			angleVel = 0.01f;
			if(distanceDiff < GOAL_RADIUS){
				linVel = 16f;
//				System.out.println("ARRIVE: IN GOAL RADIUS");
			}else if (distanceDiff < BREAK_RADIUS)
			{
				TIME_VEL = 1.5f;
				linVel = (float) ((distanceDiff)*info.getMaxVelocity())/BREAK_RADIUS;
//				System.out.println("ARRIVE: IN BREAK RADIUS");
			}
			else{
				TIME_VEL = 1f;
				linVel = info.getMaxVelocity();
				
			}
		}
		
		angAccel = (angleVel-info.getAngularVelocity())/TIME_ANG;
		float linAccel = ( linVel - info.getVelocity().length()) / TIME_VEL;
		return new Acceleration(linAccel, angAccel);
	}
	
	private void checkVertex() {
		if(cells[endGoalCell_X][endGoalCell_Y].isDriveable() == false){
			for(int x = -1;x<2;x++){
				for(int y = -1;y<2;y++){
					try{
						if(cells[endGoalCell_X+x][endGoalCell_Y+y].isDriveable()){
							endGoalVertex = cells[endGoalCell_X+x][endGoalCell_Y+y];
							return;
						} 
					}catch (ArrayIndexOutOfBoundsException e){
						
						}
					}
				}
			}
	}

	private int clamp(int val) {
		if(val < 0) return 0;
		else if(val > 24) return 24;
		else return val;
	}

//	private boolean clearLineToGoal() {
//		Point current = new Point((int)info.getX(),(int)info.getY());
//		
//		Area lineArea = new Area(line);
//		
//		for(int i = 0;i < obs.length;i++){
//			Area poly = new Area(obs[i]);
//			lineArea.intersect(poly);
//			if(!lineArea.isEmpty()){
//				return false;
//			}
//		}
//		return true;
//	}

	public void makeGraph(){
		cells = new Vertex[CELL_SPACES][CELL_SPACES];
		
		for (int x = 0; x < CELL_SPACES;x++){
			for (int y = 0; y < CELL_SPACES;y++){
				boolean notInObs = true;
				Rectangle cellRect = new Rectangle(x*cellWidth,y*cellHeight,cellWidth,cellHeight);
				for(int i = 0; i < obs.length;i++ ){
					if(obs[i].intersects((cellRect))){
						cells[x][y] = new Vertex(x,y,false);
						notInObs = false;
						break;
					}
				}
				if(notInObs){
					cells[x][y] = new Vertex(x,y,true);
					
					for(int i = 0;i<fast.length;i++){
						if(fast[i].intersects(cellRect)){
							cells[x][y].cost = 1;
							cells[x][y].initalCost = 1;
							break;
						}
					}
					
					for(int i = 0;i<slow.length;i++){
						if(slow[i].intersects(cellRect)){
							cells[x][y].cost = 10000;
							cells[x][y].initalCost = 10000;
							break;
						}
					}
					
				}
			}
		}
		
			for (int y = 0; y < CELL_SPACES;y++){
				for (int x = 0; x < CELL_SPACES-1;x++){
				Vertex currentVert = cells[x][y];
				if(y-1 >= 0){ //OBEN
					if(cells[x][y-1].isDriveable()){
						currentVert.addNeighbour(cells[x][y-1]);
					}
				}
				
				if(x+1 < CELL_SPACES){
					if(cells[x+1][y].isDriveable()){
						currentVert.addNeighbour(cells[x+1][y]);
					}
				}
				
				if(y+1 < CELL_SPACES){ //Unten
					if(cells[x][y+1].isDriveable()){
						currentVert.addNeighbour(cells[x][y+1]);
					}
				}
				
				if(x-1 >= 0 ){
					if(cells[x-1][y].isDriveable()){
						currentVert.addNeighbour(cells[x-1][y]);
					}
				}
			}
		}
	}
	
	public void clearGraph(){
		for(int i = 0;i < cells.length;i++){
			for (int j = 0;j< cells[0].length;j++){
					cells[i][j].setPrevious(null);
					cells[i][j].cost = cells[i][j].initalCost; 
			}
		}
	}

	public void findPath(Vertex start, Vertex goal){
		path = new Stack<Vertex>();
		PriorityQueue<Vertex> Q = new PriorityQueue<Vertex>();
		ArrayList<Vertex> F = new ArrayList<Vertex>();
		Q.add(start);
		while(!Q.isEmpty()){
			Vertex v = Q.poll();   //TODO: Umändern nach v mit kleinsten kosten
			
			if(v.equals(goal)){
				break;
			}
			
			ArrayList<Vertex> neighbours = v.getList();	
			for(Vertex n: neighbours){
				if(!F.contains(n)){
					if(!Q.contains(n)){
						n.cost = Integer.MAX_VALUE;
						Q.add(n);	
					}
					if(v.cost+n.initalCost < n.cost ){
						n.cost = v.cost+n.initalCost;
						n.setPrevious(v);
					}
				}
				F.add(v);
			}
		}
		Vertex addToPath = goal;
//		System.out.println("PATH TO CHECKPOINT:END->  X: "+goal.x+" / Y: "+goal.y);
		try{
			while(addToPath.getPrevious() != null){
			path.push(addToPath);
//			System.out.println(" -> X: "+addToPath.x+" / Y: "+addToPath.y);
			addToPath = addToPath.getPrevious();
			}
		}catch(NullPointerException e){
			
		}
		pathCopy = (Stack<Vertex>)path.clone();
		pathCopyPrep();
		clearGraph();
	}
	
	public double distance(float x1, float y1, float x2, float y2){
		return Math.sqrt(Math.pow((x1-x2),2)+Math.pow((y1-y2),2));
	}
	
	public void pathCopyPrep(){
		pathCopyArray = new Vertex[pathCopy.size()];
		int counter = 0;
		while(pathCopy.size() > 0){
			pathCopyArray[counter] = pathCopy.pop(); 
			counter++;
		}
	}
	
	public void doDebugStuff() {
		super.doDebugStuff();
		GL11.glBegin(GL11.GL_LINES);
		GL11.glColor3f(1f, 1f, 1f);
		GL11.glVertex2f(info.getX(), info.getY()); 
		GL11.glVertex2f(currentGoal.x, currentGoal.y);
		GL11.glColor3f(1f, 1f, 0f);
		GL11.glVertex2f(info.getX(), info.getY()); 
		GL11.glVertex2d(info.getCurrentCheckpoint().getX(), info.getCurrentCheckpoint().getY());
		GL11.glEnd();
		
		int cellWidth = (trackWidth/CELL_SPACES);
		int cellHeight = (trackHeight/CELL_SPACES);
		
		GL11.glBegin(GL11.GL_LINES);
		GL11.glColor3f(1f, 0, 0);
		GL11.glVertex2i(currentGoal.x*cellWidth, currentGoal.y*cellHeight);
		GL11.glVertex2i((currentGoal.x*cellWidth)+cellWidth, (currentGoal.y*cellHeight)+cellHeight);
		GL11.glVertex2i((currentGoal.x*cellWidth)+cellWidth, currentGoal.y*cellHeight);
		GL11.glVertex2i(currentGoal.x*cellWidth, (currentGoal.y*cellHeight)+cellHeight);
		
		GL11.glEnd();
	
		for(int x = 0; x< CELL_SPACES; x++){
			for(int y = 0;y <CELL_SPACES; y++){
				if(cells[x][y].isDriveable()){				
					GL11.glBegin(GL11.GL_LINE_STRIP);
					if(cells[x][y].initalCost == 1)GL11.glColor3f(0f, 1f, 0f);
					else if(cells[x][y].initalCost == 10000)GL11.glColor3f(0f, 0f, 1f);
					else{GL11.glColor3f(1f, 1f, 1f);}
				GL11.glVertex2i(x*cellWidth, y*cellHeight);
				GL11.glVertex2i((x*cellWidth)+cellWidth, y*cellHeight);
				GL11.glVertex2i((x*cellWidth)+cellWidth, (y*cellHeight)+cellHeight);
				GL11.glVertex2i(x*cellWidth, (y*cellHeight)+cellHeight);
				GL11.glVertex2i(x*cellWidth, y*cellHeight);
				GL11.glEnd();
					
				}
			}
		}
		GL11.glBegin(GL11.GL_LINE_STRIP);
		GL11.glColor3f(0f, 0.3f, 1);
		for(int i = 0;i < pathCopyArray.length;i++){
			Vertex pathPoint = pathCopyArray[i]; 
			GL11.glVertex2f((pathPoint.x*cellWidth)+cellWidth/2,(pathPoint.y*cellHeight)+cellHeight/2);
		}
		GL11.glEnd();
		
		
	}

}
