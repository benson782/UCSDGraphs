package roadgraph;

import geography.GeographicPoint;

/**
 * @author Benson
 *
 * A class the represents a map node's edge.
 */

public class MapEdge {
	// Initialize variables
	private GeographicPoint start;
	private GeographicPoint end;
	private String roadName;
	private String roadType;
	private double length;
	
	/**
	 * Constructor
	 * @param start  The start of the edge
	 * @param end The end of the edge
	 * @param roadName The road name
	 * @param roadType The road type
	 * @param length The length of the edge
	 */
	public MapEdge(GeographicPoint start, 
			GeographicPoint end,
			String roadName,
			String roadType,
			double length) {
		this.start = start;
		this.end = end;
		this.roadName = roadName;
		this.roadType = roadType;
		this.length = length;
	}

	/**
	 * Get the start of the edge
	 * @return Start of the edge
	 */
	public GeographicPoint getStart() {
		return start;
	}

	/**
	 * Get the end of the edge
	 * @return End of the edge
	 */
	public GeographicPoint getEnd() {
		return end;
	}

	/**
	 * Get the road name of the edge
	 * @return The road name
	 */
	public String getRoadName() {
		return roadName;
	}

	/**
	 * Get the road type of the edge
	 * @return The road type
	 */
	public String getRoadType() {
		return roadType;
	}

	/** 
	 * Get the length of the edge
	 * @return The length
	 */
	public double getLength() {
		return length;
	}
}
