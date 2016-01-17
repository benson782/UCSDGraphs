/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	// Initialize variables
	private Map<GeographicPoint, MapNode> vertices;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		vertices = new HashMap<>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		return vertices.keySet().size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		return new HashSet<GeographicPoint>(vertices.keySet());
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		int count = 0;
		for (GeographicPoint loc : vertices.keySet()) {
			MapNode node = vertices.get(loc);
			count += node.getNumEdges();
		}
		return count;
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// Add new MapNode if not in the vertices map 
		if (!vertices.containsKey(location) && location != null) {
			vertices.put(location, new MapNode(location));
			return true;
		}
		// Return false if already added
		return false;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

		// Check input parameters
		if (!vertices.containsKey(from)) {
			throw new IllegalArgumentException("From parameter not added to graph: " + from.toString());
		}
		if (!vertices.containsKey(to)) {
			throw new IllegalArgumentException("To parameter not added to graph: " + to.toString());
		}
		if (roadName == null) {
			throw new IllegalArgumentException("RoadName is null");
		}
		if (roadType == null) {
			throw new IllegalArgumentException("RoadType is null");
		}
		if (length < 0) {
			throw new IllegalArgumentException("Invalid length: " + length);
		}
		
		// Add edge to the node's edge list
		MapEdge edge = new MapEdge(from, to, roadName, roadType, length);
		MapNode node = vertices.get(from);
		node.addEdge(edge);
		vertices.put(from, node);
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// Initialize parent HashMap
		Map<GeographicPoint, GeographicPoint> parentMap = new HashMap<>();
		
		// search for path
		boolean isFound = bfsSearch(start, goal, parentMap, nodeSearched);
			
		// if no path found, return null
		if (isFound == false) {
			//System.out.println("No path exists");
			return null;
		}
		
		// reconstruct the path
		List<GeographicPoint> path = constructPath(start, goal, parentMap);
		
		return path;
	}
	
	/** Helper method that performs the BFS
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param parentMap The parent map of location nodes
	 * @param nodeSearched A hook for visualization
	 * @return true if a path was found.  false if no path was found
	 */
	private boolean bfsSearch(GeographicPoint start,
			GeographicPoint goal,
			Map<GeographicPoint, GeographicPoint> parentMap,
			Consumer<GeographicPoint> nodeSearched) {
		
		// Initialize variables
		boolean isFound = false;
		Queue<GeographicPoint> q = new LinkedList<>();
		Set<GeographicPoint> visited = new HashSet<>();
		
		// Add start to the queue
		q.add(start);
		
		// Find the path
		while (!q.isEmpty()) {
			GeographicPoint current = q.remove();
			
			// break if goal found
			if (current.equals(goal)) {
				isFound = true;
				break;
			}
			
			// get the current's neighbors
			for (GeographicPoint n : vertices.get(current).getNeighbors()) {
				if (!visited.contains(n)) {
					// add to visited
					visited.add(n);
					
					// add current as n's parent in the parent map
					parentMap.put(n, current);
					
					// add to queue
					q.add(n);
					
					// Hook for visualization.  See writeup.
					nodeSearched.accept(n);
				}
			}
		}

		return isFound;
	}
	
	/** Helper method that constructs the path from the start to the goal
	 * 
	 * @param start The start location
	 * @param goal The goal location
	 * @param parentMap The parent map of location nodes
	 * @return 
	 */
	private List<GeographicPoint> constructPath(GeographicPoint start,
			GeographicPoint goal,
			Map<GeographicPoint, GeographicPoint> parentMap) {
		LinkedList<GeographicPoint> path = new LinkedList<>();
		
		// reconstruct the path going backwards from the goal to the start
		GeographicPoint current = goal;
		while (!current.equals(start)) {
			path.addFirst(current);
			current = parentMap.get(current);
		}
		path.addFirst(start);
		
		return path;
	}

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
	}

	
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		System.out.println("DONE.");
		System.out.println("Number of vertices: " + theMap.getNumVertices());
		System.out.println("Number of edges: " + theMap.getNumEdges());
		System.out.println("Vertices:");
		for (GeographicPoint point : theMap.getVertices()) {
			System.out.println(point);
		}
		GeographicPoint start = new GeographicPoint(1, 1);
		GeographicPoint goal = new GeographicPoint(4, 1);
		List<GeographicPoint> pathBfs = theMap.bfs(start, goal);
		System.out.println("Path from " + start + " to " + goal);
		for (GeographicPoint point : pathBfs) {
			System.out.println(point);
		}

		
		// You can use this method for testing.  
		
		/* Use this code in Week 3 End of Week Quiz
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/
		
	}
	
}
