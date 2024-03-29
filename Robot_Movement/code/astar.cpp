/*
    National Technical University of Athens
    Developer: Dionysis "dionyziz" Zindros <dionyziz@gmail.com>
*/

#include "astar.h"

using namespace std;

class CompareEdges {
    private:
        vector< Edge > const & _E;
    public:
        CompareEdges( vector< Edge > const &E ) :_E( E ) {}
        inline bool operator() ( const int &a, const int &b ) {
            int diff = ( _E[ b ].distance + _E[ b ].heuristic ) - ( _E[ a ].distance + _E[ a ].heuristic );

            if ( diff == 0 ) {
                return a < b;
            }
            return diff > 0;
        }
};

void enqueue(
        int a,
        vector< Edge > const &E,
        Point mapSize,
        vector< vector< bool > > const &obstacle,
        set< Point > const &visited,
        set< int, CompareEdges > &frontier
    ) {
    Edge e = E[ a ];

    if ( !validPoint( e.to, mapSize ) ) {
        // to is out of bounds
      //  printf( "Out of bounds.\n" );
        return;
    }
    if ( obstacle[ e.to.index1 ][ e.to.index2 ] ) {
        // to is obstacle
      //  printf( "Obstacle.\n" );
        return;
    }
    if ( visited.find( e.to ) != visited.end() ) {
        // to is already visited
       // printf( "Already visited.\n" );
        return;
    }
  //  printf( "Enqueued (%i, %i) with id %i.\n", e.to.index1 + 1, e.to.index2 + 1, a );
    assert( e.distance >= 0 );
    assert( e.distance <= mapSize.index1 * mapSize.index2 );
    assert( e.heuristic >= 0 );
    assert( validPoint( e.from, mapSize ) );

    frontier.insert( a );

    assert( !frontier.empty() );
}

list< Edge > aStar(
        Point source,
        Point target,
        vector< vector< bool > > const &obstacle,
        Point mapSize
    ) {
    list< Edge > ret;
    int iterations = 0;
    vector< Edge > E;
    set< int, CompareEdges > frontier( E );
    set< Point > visited;

    // printf( "Target is (%i, %i)\n", target.index1 + 1, target.index2 + 1 );
    assert( E.empty() );
    assert( frontier.empty() );
    assert( visited.empty() );

    assert( validPoint( source, mapSize ) );
    assert( validPoint( target, mapSize ) );

    // seed the frontier with a circular edge connecting the source with the source
    E.push_back( Edge( NULL, source, source,/* non_adm(source,target)*/ manhattan( source, target ), 0 ) ); //choose my heuristic
    frontier.insert( 0 );

    while ( !frontier.empty() ) {
        // printf( "Frontier contains %i edges.\n", frontier.size() );
        set< int, CompareEdges >::iterator it = frontier.begin();
        int a = *it;
        Edge e = E[ a ];
        assert( validPoint( e.from, mapSize ) );
        assert( validPoint( e.to, mapSize ) );
        frontier.erase( it );
        if ( visited.find( e.to ) != visited.end() ) {
            // Edge "to" has already been visited.
            // Because the heuristic we ues is admissible,
            // the past visit is always going to better than this one,
            // so skip it.
            // printf( "(%i, %i) has already been visited.\n", e.to.index1, e.to.index2 );
            continue;
        }
     //   printf( "At (%i, %i).\n", e.to.index1 + 1, e.to.index2 + 1 );
        if ( e.to == target ) {
            // arrived on target
            // clean-up and return

            while ( e.from != e.to ) {
                ret.push_back( e );
                e = E[ e.parent ];
            }
            ret.push_back( e );

            E.clear();
            visited.clear();
            frontier.clear();

            printf( "A* algorithm completed in %i steps.\n", iterations );

            return ret;
        }
        visited.insert( e.to );
        // enqueue all cells adjacent to q
        for ( int index1 = e.to.index1 - 1; index1 <= e.to.index1 + 1; index1 += 2 ) {
            Point next = makePoint( index1, e.to.index2 );
           // printf( "(%i, %i) => (%i, %i): ", e.to.index1 + 1, e.to.index2 + 1, index1 + 1, e.to.index2 + 1 );
            E.push_back( Edge( a, e.to, next, manhattan( next, target ), e.distance + 1 ) );
            enqueue( E.size() - 1, E, mapSize, obstacle, visited, frontier );
        }
        for ( int index2 = e.to.index2 - 1; index2 <= e.to.index2 + 1; index2 += 2 ) {
            Point next = makePoint( e.to.index1, index2 );
         //   printf( "(%i, %i) => (%i, %i): ", e.to.index1 + 1, e.to.index2 + 1, e.to.index1 + 1, index2 + 1 );
            E.push_back( Edge( a, e.to, next, manhattan( next, target ), e.distance + 1 ) );
            enqueue( E.size() - 1, E, mapSize, obstacle, visited, frontier );
        }
        // it's impossible to iterate more than the map size if the algorithm is implemented correctly
        ++iterations;
        assert( iterations <= mapSize.index1 * mapSize.index2 );
    }

    assert( false ); // target should not be unreachable
}
