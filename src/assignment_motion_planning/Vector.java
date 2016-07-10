package assignment_motion_planning;

import java.util.Arrays;

public class Vector implements Comparable<Vector> {
    private double[] vector;
    
    /**
     * Constructor
     * @param cs values for each coordinate
     */
    public Vector(double... v) {
        vector = Arrays.copyOf(v, v.length);
    }
    
    /**
     * Get the dimension
     * @return the dimension
     */
    public int getDimension() {
        return vector.length;
    }
    
    /**
     * Return the index-th coordinate
     * @param index the index
     * @return      the value in the index-th coordinate
     */
    public double get(int index) {
        return vector[index];
    }
    
    @Override
    public int hashCode() {
        return Arrays.hashCode(vector);
    }
    
    @Override
    public boolean equals(Object o) {
        if (!(o instanceof Vector))
            return false;
        return Arrays.equals(vector, ((Vector)o).vector);
    }
    
    @Override
    public String toString() {
        return "Vector: " + Arrays.toString(vector);
    }

    @Override
    public int compareTo(Vector o) {
        assert(getDimension() == o.getDimension());
        for (int i = 0; i < getDimension(); ++i) {
            int comparison = Double.compare(get(i), o.get(i));
            if (comparison != 0)
                return comparison;
        }
        return 0;
    }
}
