package graph;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.HashSet;
import java.util.function.Consumer;

/**
 * A class which represents a node in the MapGraph class.
 * Edges represent roads to and from the node.
 */	
public class CapNode implements Comparable<CapNode>  {
	private int node;
	private HashSet<CapEdge> edges;
	private double distance;

	/**
	 * Create a new CapNode with a specific int.
	 */		
	public CapNode(int n) {
		node = n;
		edges = new HashSet<CapEdge>();
		distance = Double.POSITIVE_INFINITY;
	}
	
	/**
	 * Get the edges in the CapNode
	 * @return The edges in the graph.
	 */	
	public HashSet<CapEdge> getEdges() {
		return edges;
	}
	
	/**
	 * Add an edge to the list of edges in the CapNode.
	 */		
	public void addEdge(CapEdge edge) {
		edges.add(edge);
	}
	
	/**
	 * Get the geographic location of the node.
	 * @return The GeographicPoint of the node.
	 */		
	public int getNode() {
		return node;
	}
	
	public void setDistance(double num) {
		distance = num;
	}
	
	public double getDistance() {
		return distance;
	}
	
	/**
	 * Sort CapNodes based on integer value of node.
	 * 	@Override
	 */		
	public int compareTo(CapNode curr) {
		if(distance < curr.getDistance()) {
			return -1;
		}
		if(distance > curr.getDistance()) {
			return 1;
		}
		return 0;
	}
	
	
}
