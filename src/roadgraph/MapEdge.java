package roadgraph;

import geography.GeographicPoint;

/**
 * @author Benson
 *
 */

public class MapEdge {
	private GeographicPoint start;
	private GeographicPoint end;
	private String roadName;
	private String roadType;
	private double length;
	
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

	public GeographicPoint getStart() {
		return start;
	}

	public GeographicPoint getEnd() {
		return end;
	}

	public String getRoadName() {
		return roadName;
	}

	public String getRoadType() {
		return roadType;
	}

	public double getLength() {
		return length;
	}
}
