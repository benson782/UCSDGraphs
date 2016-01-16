package roadgraph;

import java.util.ArrayList;
import java.util.List;

import geography.GeographicPoint;

/** 
 * @author Benson
 *
 */

public class MapNode {
	private GeographicPoint location;
	private List<MapEdge> edges;

	public MapNode(GeographicPoint location) {
		this.location = location;
		this.edges = new ArrayList<>();
	}
	
	public GeographicPoint getLocation() {
		return location;
	}
	
	public void addEdge(MapEdge edge) {
		edges.add(edge);
	}
	
}
