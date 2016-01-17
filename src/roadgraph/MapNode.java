package roadgraph;

import java.util.ArrayList;
import java.util.List;

import geography.GeographicPoint;

/** 
 * @author Benson
 *
 * A class that represents a map location and 
 * the edges from the location.
 * 
 */

public class MapNode {
	// Initialize variables
	private GeographicPoint location;
	private List<MapEdge> edges;

	/**
	 * Constructor
	 * @param location Location of map node
	 */
	public MapNode(GeographicPoint location) {
		this.location = location;
		this.edges = new ArrayList<>();
	}
	
	/**
	 * Get the location
	 * @return The location
	 */
	public GeographicPoint getLocation() {
		return location;
	}
	
	/**
	 * Add an edge to the node
	 * @param edge The edge to add.
	 */
	public void addEdge(MapEdge edge) {
		edges.add(edge);
	}
	
	/**
	 * Get the number of edges from the node
	 * @return The number of edges from the node.
	 */
	public int getNumEdges() {
		return edges.size();
	}
	
	/**
	 * Get a list of neighbors from the node
	 * @return List of neighbors
	 */
	public List<GeographicPoint> getNeighbors() {
		List<GeographicPoint> n = new ArrayList<>();
		for (MapEdge e : edges) {
			n.add(e.getEnd());
		}
		return n;
	}
}
