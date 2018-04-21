package geography;

import java.awt.geom.Point2D.Double;

@SuppressWarnings("serial")
public class GeographicPoint extends Double {
	
	public GeographicPoint(double latitude, double longitude)
	{
		super(latitude, longitude);
	}
	
	/**
	 * Calculates the geographic distance in km between this point and 
	 * the other point. 
	 * @param other
	 * @return The distance between this lat, lon point and the other point
	 */
	public double distance(GeographicPoint other)
	{
		return getDist(this.getX(), this.getY(),
                other.getX(), other.getY());     
	}
	
	/**
	 * Returns the distance in kilometers of the two points.
	 * @param lat1 - The latitude of the first point.
	 * @param lon1 - The longitude of the first point.
	 * @param lat2 - The latitude of the second point.
	 * @param lon2 - The longitude of the second point.
	 * @return The distance in kilometers of the two points.
	 */
    public static double getDist(double lat1, double lon1, double lat2, double lon2)
    {
    	int R = 6373; // radius of the earth in kilometres
    	double lat1rad = Math.toRadians(lat1);
    	double lat2rad = Math.toRadians(lat2);
    	double deltaLat = Math.toRadians(lat2-lat1);
    	double deltaLon = Math.toRadians(lon2-lon1);

    	double a = Math.sin(deltaLat/2) * Math.sin(deltaLat/2) +
    	        Math.cos(lat1rad) * Math.cos(lat2rad) *
    	        Math.sin(deltaLon/2) * Math.sin(deltaLon/2);
    	double c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));

    	double d = R * c;
    	return d;
    }
    
    public String toString()
    {
    	return "Lat: " + getX() + ", Lon: " + getY();
    }
	
	
}
