package graph;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Consumer;

/**
 * A class which represents an edge between 2 nodes.
 */	
public class CapEdge {
	private int startNode;
	private int endNode;
	private double betweenness;
	
	/**
	 * Create a new CapEdge between a starting node and an ending node.
	 */		
	public CapEdge(int start, int end) {
		startNode = start;
		endNode = end;
		betweenness = 0.0;
	}
	
	/**
	 * Get the starting node of the edge. 
	 * @return The node where the edge starts.
	 */			
	public int getStartNode() {
		return startNode;
	}
	
	/**
	 * Get the ending node of the edge. 
	 * @return The node where the edge ends.
	 */		
	public int getEndNode() {
		return endNode;
	}
	
	public double getBetweenness() {
		return betweenness;
	}
	
	public void setBetweenness(double num) {
		betweenness = num;
	}

}
