#include <vector>
#include <cstdio>
#include <cassert>
#include <list>
#include "astar.h"


struct Check{
	int index1;
	int index2;
	int timer;
};


using namespace std;

int main() {
    Point mapSize;
    vector< vector< bool > > obstacle;
    Point A, B, target;
    int timeA=1,timeB=1,TimeToTarget,TimeToTargetB;
    int k,i;
    
    /**
     
     Read the coordinates
    
     **/
    
    
    scanf( "%i %i\n",&mapSize.index1, &mapSize.index2);
    scanf("%i %i\n",&A.index1,&A.index2);
    scanf("%i %i\n",&B.index1,&B.index2);
    scanf("%i %i\n",&target.index1,&target.index2);
    
    
    --A.index1;
    --A.index2;
    --B.index1;
    --B.index2;
    --target.index1;
    --target.index2;
    
    assert( mapSize.index1 > 0 );
    assert( mapSize.index2 > 0 );
    assert( mapSize.index1 <= 400000 );
    assert( mapSize.index2 <= 400000 );
    assert( validPoint( A, mapSize ) );
    assert( validPoint( B, mapSize ) );
    assert( validPoint( target, mapSize ) );
    
    scanf("%i\n",&k);
    Point P[k];
    for(i=0;i<k;i++){
    	scanf("%i %i\n",&P[i].index1,&P[i].index2);
    	P[i].index1--;
    	P[i].index2--;
    	assert(validPoint(P[i],mapSize));
    }

    for ( int index1 = 0; index1 < mapSize.index1; ++index1 ) {
        obstacle.push_back( vector< bool >( mapSize.index2 ) );
    }

    for ( int index2 = 0; index2 < mapSize.index2; ++index2 ) {
        for ( int index1 = 0; index1 < mapSize.index1; ++index1 ) {
            assert( obstacle[ index1 ][ index2 ] || !obstacle[ index1 ][ index2 ] );
            char c;
            scanf( "%c", &c );
            assert( c == 'X' || c == 'O' );
            obstacle[ index1 ][ index2 ] = (c == 'X');
        }
        scanf( "\n" );
    }
    assert( !obstacle[ target.index1 ][ target.index2 ] );
    assert( !obstacle[ A.index1 ][ A.index2 ] );
    assert( !obstacle[ B.index1 ][ B.index2 ] );
    for(i=0;i<k;i++){
    	assert(!obstacle[P[i].index1][P[i].index2]);
    }
    Point lastEndA,lastEndB;
    list< Edge > APath,BPath;
    list <struct Check> temp;
    list <struct Check> temp2;
    Point prev;
    
    struct Check helper;
    for (i=0;i<k;i++){
    	if(i==0){
    		APath= aStar(P[0],A,obstacle,mapSize);
    		for ( list< Edge >::iterator it = APath.begin(); it != APath.end(); ++it ) {
			helper.index1=it->from.index1;
			helper.index2=it->from.index2;
			helper.timer=timeA;
			temp.push_back(helper);
			timeA++;	
			if (it->from == P[0])
				break;		
		}
    	}
    	else{
    		APath = aStar(P[i],P[i-1],obstacle,mapSize);
    		for ( list< Edge >::iterator it = APath.begin(); it != APath.end(); ++it ) {
			helper.index1=it->from.index1;
			helper.index2=it->from.index2;
			helper.timer=timeA;
			temp.push_back(helper);
			timeA++;
			if (it->from == P[i])
				break;			
		}
    	}
    }
    APath = aStar(target,P[k-1],obstacle,mapSize);
    for ( list< Edge >::iterator it = APath.begin(); it != APath.end(); ++it ) {
	helper.index1=it->from.index1;
	helper.index2=it->from.index2;
	helper.timer=timeA;
	temp.push_back(helper);
	timeA++;
	if(it->from==target){
		TimeToTarget=timeA-1;
		break;
	}
    }
    APath = aStar(A,target,obstacle,mapSize);
    for ( list< Edge >::iterator it = APath.begin(); it != APath.end(); ++it ) {
	helper.index1=it->from.index1;
	helper.index2=it->from.index2;
	helper.timer=timeA;
	temp.push_back(helper);
	timeA++;
	if(it->from==A)
		break;
    }
    
    
    for (i=0;i<k;i++){
    	if(i==0){
    		BPath= aStar(P[0],B,obstacle,mapSize);
    		for ( list< Edge >::iterator it = BPath.begin(); it != BPath.end(); ++it ) {
			helper.index1=it->from.index1;
			helper.index2=it->from.index2;
			helper.timer=timeB;
			temp2.push_back(helper);
			timeB++;	
			if (it->from == P[0])
				break;		
		}
    	}
    	else{
    		BPath = aStar(P[i],P[i-1],obstacle,mapSize);
    		for ( list< Edge >::iterator it = BPath.begin(); it != BPath.end(); ++it ) {
			helper.index1=it->from.index1;
			helper.index2=it->from.index2;
			helper.timer=timeB;
			temp2.push_back(helper);
			timeB++;
			if (it->from == P[i])
				break;			
		}
    	}
    }
    BPath = aStar(target,P[k-1],obstacle,mapSize);
    for ( list< Edge >::iterator it = BPath.begin(); it != BPath.end(); ++it ) {;
	helper.index1=it->from.index1;
	helper.index2=it->from.index2;
	helper.timer=timeB;
	temp2.push_back(helper);
	timeB++;
	if(it->from==target){
		TimeToTargetB=timeB-1;
		break;
	}
    }
    BPath = aStar(B,target,obstacle,mapSize);
    for ( list< Edge >::iterator it = BPath.begin(); it != BPath.end(); ++it ) {;
	helper.index1=it->from.index1;
	helper.index2=it->from.index2;
	helper.timer=timeB;
	temp2.push_back(helper);
	timeB++;
	if(it->from==B)
		break;
    }
    
    timeA--;
    timeB--;
if (TimeToTarget > TimeToTargetB){
    printf( "Robot A path:\n" );
    //visualizelist(temp,obstacle,mapSize);
    for ( list< struct Check >::iterator it = temp.begin(); it != temp.end(); ++it ) {
    	printf("(%i, %i) at moment:%i\n",it->index1 +1 ,it->index2 +1 ,it->timer);
    }
    timeB=1;
    for (i=0;i<k;i++){
    	if(i==0){
    		BPath= aStar(P[0],B,obstacle,mapSize);
    		//visualize( BPath, obstacle, mapSize );
    		printf( "Robot B path:\n" );
    		prev=BPath.begin()->from;
    		for ( list< Edge >::iterator it = BPath.begin(); it != BPath.end(); ++it ) {
			if(timeA > timeB){
				helper=temp.front();
				temp.pop_front();
				if (helper.index1==it->from.index1 && helper.index2==it->from.index2 && helper.timer==timeB){
                    //printf("Collision Happens ==> B waits here for a moment\n"); // an 8elw to vazw gia na fainetai pote exw collision
					printf( "(%i, %i) at moment:%i\n", prev.index1 + 1, prev.index2 + 1,timeB );
					--it;
				}
				else{
				printf( "(%i, %i) at moment:%i\n", it->from.index1 + 1, it->from.index2 + 1,timeB );
				prev=it->from;
				}
			}
			else{
				printf( "(%i, %i) at moment:%i\n", it->from.index1 + 1, it->from.index2 + 1,timeB );
				prev=it->from;
			}
			timeB++;	
			if (it->from == P[0])
				break;		
		}
    	}
    	else{
    		BPath= aStar(P[i],P[i-1],obstacle,mapSize);
    		//visualize( BPath, obstacle, mapSize );
    		prev=BPath.begin()->from;
    		for ( list< Edge >::iterator it = BPath.begin(); it != BPath.end(); ++it ) {
			if(timeA > timeB){
				helper=temp.front();
				temp.pop_front();
				if (helper.index1==it->from.index1 && helper.index2==it->from.index2 && helper.timer==timeB){
                    //printf("Collision Happens ==> B waits here for a moment\n"); // an 8elw to vazw gia na fainetai pote exw collision
					printf( "(%i, %i) at moment:%i\n", prev.index1 + 1, prev.index2 + 1,timeB );
					--it;
				}
				else{
				printf( "(%i, %i) at moment:%i\n", it->from.index1 + 1, it->from.index2 + 1,timeB );
				prev=it->from;
				}
			}
			else{
				printf( "(%i, %i) at moment:%i\n", it->from.index1 + 1, it->from.index2 + 1,timeB );
				prev=it->from;
			}
			timeB++;	
			if (it->from == P[i])
				break;		
		}
    	}
       
     }
    BPath = aStar(target,P[k-1],obstacle,mapSize);
    //visualize( BPath, obstacle, mapSize );
    prev=BPath.begin()->from;
    for ( list< Edge >::iterator it = BPath.begin(); it != BPath.end(); ++it ) {
	if (it->from == target){
		while (timeB<=TimeToTarget){
			printf( "(%i, %i) at moment:%i\n", prev.index1 + 1, prev.index2 + 1,timeB );
			timeB++;
		}
		break;
	}
	if(timeA > timeB){
		helper=temp.front();
		temp.pop_front();
		if (helper.index1==it->from.index1 && helper.index2==it->from.index2 && helper.timer==timeB){
            //printf("Collision Happens ==> B waits here for a moment\n"); // an 8elw to vazw gia na fainetai pote exw collision
			printf( "(%i, %i) at moment:%i\n", prev.index1 + 1, prev.index2 + 1,timeB );
			--it;
		}
		else{
		printf( "(%i, %i) at moment:%i\n", it->from.index1 + 1, it->from.index2 + 1,timeB );
		prev=it->from;
		}
	}
	else{
		printf( "(%i, %i) at moment:%i\n", it->from.index1 + 1, it->from.index2 + 1,timeB );
		prev=it->from;
	}
	timeB++;	
	if (it->from == target)
		break;
    }
    BPath = aStar(B,prev,obstacle,mapSize);
    //visualize( BPath, obstacle, mapSize );
    prev=BPath.begin()->from;
    for ( list< Edge >::iterator it = BPath.begin(); it != BPath.end(); ++it ) {
	if (timeA > timeB){
		helper=temp.front();
		temp.pop_front();
		if (helper.index1==it->from.index1 && helper.index2==it->from.index2 && helper.timer==timeB){
            //printf("Collision Happens ==> B waits here for a moment\n"); // an 8elw to vazw gia na fainetai pote exw collision
			printf( "(%i, %i) at moment:%i\n", prev.index1 + 1, prev.index2 + 1,timeB );
			--it;
		}
		else{
		printf( "(%i, %i) at moment:%i\n", it->from.index1 + 1, it->from.index2 + 1,timeB );
		prev=it->from;
		}
	}	
	else{
		printf( "(%i, %i) at moment:%i\n", it->from.index1 + 1, it->from.index2 + 1,timeB );
		prev=it->from;
	}
	timeB++;	
	if (it->from == B)
		break;
   } 
}  




if (TimeToTarget <= TimeToTargetB){
    printf( "Robot B path:\n" );
    for ( list< struct Check >::iterator it = temp2.begin(); it != temp2.end(); ++it ) {
    	printf("(%i, %i) at moment:%i\n",it->index1 +1 ,it->index2 +1 ,it->timer);
    }
    timeA=1;
    for (i=0;i<k;i++){
    	if(i==0){
    		APath= aStar(P[0],A,obstacle,mapSize);
    		//visualize( APath, obstacle, mapSize );
    		printf( "Robot A path:\n" );
    		prev=APath.begin()->from;
    		for ( list< Edge >::iterator it = APath.begin(); it != APath.end(); ++it ) {
			if(timeA < timeB){
				helper=temp2.front();
				temp2.pop_front();
				if (helper.index1==it->from.index1 && helper.index2==it->from.index2 && helper.timer==timeA){
                    //printf("Collision Happens ==> A waits here for a moment\n"); // an 8elw to vazw gia na fainetai pote exw collision
					printf( "(%i, %i) at moment:%i\n", prev.index1 + 1, prev.index2 + 1,timeA );
					--it;
				}
				else{
				printf( "(%i, %i) at moment:%i\n", it->from.index1 + 1, it->from.index2 + 1,timeA );
				prev=it->from;
				}
			}
			else{
				printf( "(%i, %i) at moment:%i\n", it->from.index1 + 1, it->from.index2 + 1,timeA );
				prev=it->from;
			}
			timeA++;	
			if (it->from == P[0])
				break;		
		}
    	}
    	else{
    		APath= aStar(P[i],P[i-1],obstacle,mapSize);
    		//visualize( APath, obstacle, mapSize );
    		prev=APath.begin()->from;
    		for ( list< Edge >::iterator it = APath.begin(); it != APath.end(); ++it ) {
			if(timeA < timeB){
				helper=temp2.front();
				temp2.pop_front();
				if (helper.index1==it->from.index1 && helper.index2==it->from.index2 && helper.timer==timeA){
                    //printf("Collision Happens ==> A waits here for a moment\n"); // an 8elw to vazw gia na fainetai pote exw collision
					printf( "(%i, %i) at moment:%i\n", prev.index1 + 1, prev.index2 + 1,timeA );
					--it;
				}
				else{
				printf( "(%i, %i) at moment:%i\n", it->from.index1 + 1, it->from.index2 + 1,timeA );
				prev=it->from;
				}
			}
			else{
				printf( "(%i, %i) at moment:%i\n", it->from.index1 + 1, it->from.index2 + 1,timeA );
				prev=it->from;
			}
			timeA++;	
			if (it->from == P[i])
				break;		
		}
    	}
       
     }
    APath = aStar(target,P[k-1],obstacle,mapSize);
    //visualize( APath, obstacle, mapSize );
    prev=APath.begin()->from;
    for ( list< Edge >::iterator it = APath.begin(); it != APath.end(); ++it ) {
	if (it->from == target){
		while (timeA<=TimeToTargetB){
			printf( "(%i, %i) at moment:%i\n", prev.index1 + 1, prev.index2 + 1,timeB );
			timeA++;
		}
		break;
	}
	if(timeA < timeB){
		helper=temp2.front();
		temp2.pop_front();
		if (helper.index1==it->from.index1 && helper.index2==it->from.index2 && helper.timer==timeA){
            //printf("Collision Happens ==> A waits here for a moment\n"); // an 8elw to vazw gia na fainetai pote exw collision
			printf( "(%i, %i) at moment:%i\n", prev.index1 + 1, prev.index2 + 1,timeA );
			--it;
		}
		else{
		printf( "(%i, %i) at moment:%i\n", it->from.index1 + 1, it->from.index2 + 1,timeA );
		prev=it->from;
		}
	}
	else{
		printf( "(%i, %i) at moment:%i\n", it->from.index1 + 1, it->from.index2 + 1,timeA );
		prev=it->from;
	}
	timeA++;	
	if (it->from == target)
		break;
    }
    APath = aStar(A,prev,obstacle,mapSize);
    //visualize( APath, obstacle, mapSize );
    prev=APath.begin()->from;
    for ( list< Edge >::iterator it = APath.begin(); it != APath.end(); ++it ) {
	if (timeA < timeB){
		helper=temp2.front();
		temp2.pop_front();
		if (helper.index1==it->from.index1 && helper.index2==it->from.index2 && helper.timer==timeA){
			//printf("Collision Happens ==> A waits here for a moment\n"); // an 8elw to vazw gia na fainetai pote exw collision
			printf( "(%i, %i) at moment:%i\n", prev.index1 + 1, prev.index2 + 1,timeA );
			--it;
		}
		else{
		printf( "(%i, %i) at moment:%i\n", it->from.index1 + 1, it->from.index2 + 1,timeA );
		prev=it->from;
		}
	}	
	else{
		printf( "(%i, %i) at moment:%i\n", it->from.index1 + 1, it->from.index2 + 1,timeA );
		prev=it->from;
	}
	timeA++;	
	if (it->from == A)
		break;
   } 
}   
  
    printf( "\n" );


    return 0;
}
