/*
    National Technical University of Athens
    Developer: Dionysis "dionyziz" Zindros <dionyziz@gmail.com>
*/

#ifndef ASTAR_H
#define ASTAR_H

#include <list>
#include <vector>
#include <set>
#include <cassert>
#include <cstdio>
#include <cstdlib>

struct Point {
    int index1;
    int index2;
};

struct fPoint {
    float index1;
    float index2;
};

inline bool validPoint( Point a, Point mapSize ) {
    return a.index1 >= 0 && a.index2 >= 0 && a.index1 < mapSize.index1 && a.index2 < mapSize.index2;
}

inline bool operator <( Point a, Point b ) {
    if ( a.index1 != b.index1 ) {
        return a.index1 < b.index1;
    }
    return a.index2 < b.index2;
}

inline bool operator ==( Point a, Point b ) {
    return a.index1 == b.index1 && a.index2 == b.index2;
}

inline bool operator !=( Point a, Point b ) {
    return !( a == b );
}

inline Point makePoint( int index1, int index2 ) {
    Point ret;

    ret.index1 = index1;
    ret.index2 = index2;

    return ret;
}

inline int manhattan( Point source, Point target ) {
    return abs( ( double )source.index1 - target.index1 ) + abs( ( double )source.index2 - target.index2 );
}
inline int non_adm (Point source,Point target){
	return (( ( double )source.index1 - target.index1 )*( ( double )source.index1 - target.index1 ) + ( ( double )source.index2 - target.index2 )*( ( double )source.index2 - target.index2 ));
}
	
struct Edge {
    public:
        int parent;
        Point from;
        Point to;
        int heuristic; // approximation of how far the "to" end of Edge is from the target
        int distance; // actual distance between the "to" end of the Edge and the source

        Edge( int parent, Point from, Point to, int heuristic, int distance ):
            parent( parent ), from( from ), to( to ),
            heuristic( heuristic ), distance( distance ) {};
};

std::list< Edge > aStar(
    Point,
    Point,
    std::vector< std::vector< bool > > const &,
    Point
);

#endif
