#ifndef HULLO_H
#define HULLO_H
/*

  https://github.com/superwills/Hullinator
  Hull.h - Convex hull creation from point cloud
  Accompanies my answer at http://gamedev.stackexchange.com/questions/25397/obb-vs-obb-collision-detection/60225#60225
  version 1.0.0, Aug 4, 2013 319p

  Copyright (C) 2013 William Sherif

  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.

  William Sherif
  will.sherif@gmail.com

*/

#include "Vectorf.h"
#include "Intersectable.h"
#include <set>
using namespace std;



// indexed edge
struct IEdge
{
  int a,b ;
  IEdge( int ia, int ib ) : a(ia), b(ib) {
    if( a==b ) warning ( "Degenerate edge %d==%d", a,b ) ;
  }
  
  // Verts describe the same edge
  bool same( const IEdge& e ) const {
    // a---b
    // b---a
    return (a==e.a && b==e.b)||(a==e.b && b==e.a);
  }
  void flip() { ::swap(a,b); }
} ;

struct ITri
{
  int a,b,c ;
  
  ITri( int ia, int ib, int ic ) : a(ia),b(ib),c(ic){}
  bool hasEdge( const IEdge& edge ) const {
    // edge.a and edge.b must BOTH be in a,b,c
    // assuming the edge is not degenerate
    return (edge.a==a || edge.a==b || edge.a==c ) &&
           (edge.b==a || edge.b==b || edge.b==c ) ;
  }
  
  // Do I have AT LEAST ONE of your edges
  bool sharesEdge( const ITri& o ) const {
    return hasEdge( IEdge(o.a,o.b) ) || 
           hasEdge( IEdge(o.b,o.c) ) || 
           hasEdge( IEdge(o.c,o.a) ) ;
  }
} ;

// The interface of this class is really Vector3f.  You pass in Vector3f's to
// specify the point cloud via addPtToBound(Vector3f), then you call hull.expandToContainAllPts().
// After you are all done, the hull's points are in hull.finalPts and hull.finalNormals.
//
// You may rework this class to hide its internals more deeply if you wish.

// Convex hull def'n
struct Hull
{
  vector<Vector3f> verts ;

  // indices of the actual hull.  in creating the convex hull no new vertices are added,
  // only _different points_ of the original point cloud's points may be used in specifying the hull.
  vector<int> indices ;

  //AABB aabb ; // used to find pts closest to AABB corners.
  vector<int> remIndices ; // candidate pts for the hull when being constructed.
  
  // The UNIQUE set of hull points
  vector<Vector3f> finalPts ;
  
  // The group of triangles representing the final convex hull.
  vector<Triangle> finalTris ;
  
  // The group of distinct normals on the final convex hull.
  // if two normals are __very similar__, (two coplanar tris) then they
  // are considered as 1 normal.  That means one less axis to test in SAT testing,
  // since SAT only uses face normals and not actual Triangle position in space.
  vector<Vector3f> finalNormals ;
  
  // This is the distance that is tolerable for pts to be conisdered inside
  // the hull while they are really outside it.
  float tolerance ;
  
  // the extreme pts go in the hull to start
  //Vector3f N[3]={ HUGE,HUGE,HUGE }, P[3]={-HUGE,-HUGE,-HUGE} ; // mins, maxes
  
  AABB aabb ;
  // The indices of the minimum and maximum vertices,
  // in each of the xyz axes independently.
  int mins[3], maxes[3] ;
  
  // ids of vertices CLOSEST TO these corners of the AABB.
  vector<int> extremeCorners ;
  
  // Has to be BUNCH of values b/c there could be a # of values on the perimeter
  //map< int, vector<Vector3f> > Ns,Ps ; // i used these for finding the AVERAGE pts.
  Hull()
  {
    tolerance = 0.5 ; // THE BIGGER YOU SET THIS, THE MORE LIKELY
    // THE HULL IS TO "IGNORE" OUTLIERS OF THE HULL AT ANY STAGE.
    // IF A POINT IS LESS THAN `TOLERANCE` UNITS AWAY FROM THE HULL AT
    // ANY STAGE, IT IS CONSIDERED "INSIDE" THE HULL AND ITS DISCARDED WITHOUT
    // ACTUALLY BOUNDING IT.  YOU CAN RAISE THIS NUMBER TO COME UP WITH
    // MORE COARSE (FEWER TRIS) BUT STILL KIND OF ACCURATE HULLS.
  }
  
  // clear out all old information
  void clear()
  {
    verts.clear() ;  indices.clear() ;  remIndices.clear() ;
    finalPts.clear() ;  finalNormals.clear() ;  finalTris.clear() ;
    aabb = AABB() ;
  }
  
  void solve()
  {
    initFromExtremePts() ;
    expandToContainAllPts() ;
  }
  
  // Finds the initial set of extreme points for the hull.
  void findExtreme()
  {
    for( int i = 0 ; i < 3 ; i++ )  mins[i]=maxes[i]=0;
    indices.clear() ;
    remIndices.clear() ;
    aabb = AABB() ;
    // find the extreme pts
    for( int i = 0 ; i < verts.size() ; i++ )
    {
      aabb.bound( verts[ i ] ) ;
      remIndices.push_back( i ) ; // add the index to the indices to process.
      // Look for 6 pts that minimize and maximize x,y,z axes
      // find the 6 pts (may not be distinct) with:
      //   smallest x (mins[0]), smallest y (mins[1]), sm z (mins[2]), largest x maxes[0], lg y, lg z.
      for( int axis = 0 ; axis < 3 ; axis++ )
      {
        if( verts[i].elts[axis] < verts[mins[axis]].elts[axis] ) // this is the new minimum point FOR THAT AXIS
          mins[axis] = i ;
        if( verts[i].elts[axis] > verts[maxes[axis]].elts[axis] ) // this is the new maximum point FOR THAT AXIS
          maxes[axis] = i ;
      }
    }
    
    // The tolerance should be related to the extreme points
  }
  
  void initFromExtremePts()
  {
    findExtreme() ;
    
    // IF THE 6 EXTREME POINTS ARE NOT UNIQUE, then
    // the initial mesh could be malformed if we don't take precaution.
    
    // Those 6 extreme pts form the initial hull mesh.
    // it's a really crude approximation, but if no points lie that far outside,
    // it could actually be a pretty good approx.
    /*
    // 8 tri  version
    int *P=maxes, *N=mins; // alias names
    addTri( P[2], P[0], P[1] ) ; // bitwise so it doesn't short cct
    addTri( P[2], P[1], N[0] ) ;
    addTri( P[2], N[0], N[1] ) ;
    addTri( P[2], N[1], P[0] ) ;
    
    addTri( N[2], P[1], P[0] ) ;
    addTri( N[2], N[0], P[1] ) ;
    addTri( N[2], N[1], N[0] ) ;
    addTri( N[2], P[0], N[1] ) ;
    //*/
    //warning( "Degenerate tri added to hull `%s`", name.c_str() ) ;
    
    // Use the AABB, and find the 4 pts that are nearest the pxpypz corners.. etc.
    aabb.recomputeCorners() ;
    
    // Find the 8 vertices closest to the extreme corners
    // If you look in recomputeCorners(), you will see the indices used here
    struct Corner{
      int aabbCornerIndex;
      int closestIndexSoFar ;
      float dist2 ;
      Corner( int iAABBCornerIndex ) {
        aabbCornerIndex=iAABBCornerIndex;
        // Init these 2 with vals that will be overwritten 1st iteration
        closestIndexSoFar=0;
        dist2=HUGE;
      }
    } ;
    vector<Corner> corners ;
    for( int i = 0 ; i < aabb.corners.size() ; i++ )
      corners.push_back( Corner( i ) ) ;
    
      // check if the selected vertex for pxpypz is indeed THE CLOSEST ONE to pxpypz or no
    for( int c = 0 ; c < corners.size() ; c++ )
    {
      for( int i = 0 ; i < verts.size() ; i++ )
      {
        float dist2 = distance2( verts[i], aabb.corners[corners[c].aabbCornerIndex] ) ;
        if( dist2 < corners[c].dist2 )
        {
          corners[c].dist2 = dist2 ;
          corners[c].closestIndexSoFar = i ;
        }
      }
    }
    // 1 nxnypz, 2 nxpynz, 4 pxnynz, 7 pxpypz
    // From a diagram of the AABB cube,
    //   2----G
    //  /|   /|
    // D-A--7 4
    // |/   |/
    // 1----F
    // Faces
    extremeCorners.resize(8);
    for( int i = 0 ; i < aabb.corners.size() ; i++ )
      extremeCorners[i] = corners[i].closestIndexSoFar ;
    
    // see if the index values of the extreme corners are unique
    set<int> uniqueExtremeCorners ;
    uniqueExtremeCorners.insert( extremeCorners[1] ) ;
    uniqueExtremeCorners.insert( extremeCorners[2] ) ;
    uniqueExtremeCorners.insert( extremeCorners[4] ) ;
    uniqueExtremeCorners.insert( extremeCorners[7] ) ;
    
    if( uniqueExtremeCorners.size() != 4 )
    {
      // Even if this happens, the convex hull still might come out correctly.
      warning( "Extreme corners bad, only %d of them, %d %d %d %d", uniqueExtremeCorners.size(),
        extremeCorners[1], extremeCorners[2], extremeCorners[4], extremeCorners[7] ) ;
        
      
    }
    
    // wind: 1,4,7 | 1,7,2 | 1,2,4 | 4,2,7
    addTri( extremeCorners[1], extremeCorners[4], extremeCorners[7] ) ;
    addTri( extremeCorners[1], extremeCorners[7], extremeCorners[2] ) ;
    addTri( extremeCorners[1], extremeCorners[2], extremeCorners[4] ) ;
    addTri( extremeCorners[4], extremeCorners[2], extremeCorners[7] ) ;
    
    if( !convexityTest() )
    {
      error( "BAD SEED HULL. Your hull will malfunction because the initial tetrahedron got fucked up. "
        "Hullinator apologizes, but you have to give me better spread-out points." ) ;
        
      drawDebugExtremePts() ;
    }
  }
  
  // This is the test to make sure you indeed HAVE A convex polygon.
  // basically no face can have its normal pointing towards ANY other point
  // in the shape.  if it does, a face is BACKWARDS.
  bool convexityTest() const
  {
    for( int i = 0 ; i < indices.size() ; i+=3 )
    {
      Plane plane( verts[indices[i]], verts[indices[i+1]], verts[indices[i+2]] ) ;
      
      for( int j = 0 ; j < indices.size() ; j++ )
      {
        if( i==j ) { j+=2 ; skip; } // don't test your verts against the tri you came out of.
        if( plane.distanceToPoint( verts[indices[j]] ) > tolerance )
        {
          warning( "Your convex polygon is not convex." ) ;
          addPermDebugPoint( verts[indices[j]], Red ) ;
          addPermDebugTriLine( verts[indices[i]], verts[indices[i+1]], verts[indices[i+2]], Red ) ;
          return 0 ;
        }
      }
    }
    return 1 ;
  }
  
  void drawDebugLines( const Vector4f& color ) const {
    for( int i = 0 ; i < indices.size() ; i+=3 )
      addDebugTriLine( verts[indices[i]], verts[indices[i+1]], verts[indices[i+2]], color ) ;
  }

  void drawDebug( const Vector4f& color ) const {
    //for( int i = 0 ; i < indices.size() ; i+=3 )
    //  addDebugTriSolid( verts[indices[i]], verts[indices[i+1]], verts[indices[i+2]], color ) ;
    for( int i = 0 ; i < finalTris.size() ; i++ )
    {
      addDebugTriSolid( finalTris[i].a,finalTris[i].b,finalTris[i].c, color ) ;
    }
    
  }
  
  void drawDebugExtremePts() const {
    // SHOW THE ORIGINAL EXTREME POINTS
    /*
    for( int axis = 0 ; axis < 3 ; axis++ )
    {
      Vector4f color( !axis, (axis==1)?1.f:0.f, (axis==2)?1.f:0.f, 1.f ) ;
      addDebugPoint( verts[mins[axis]], color*0.5 ) ;
      addDebugPoint( verts[maxes[axis]], color ) ;
    }
    //*/
    
    // Show the points closest to the AABB's corners
    for( int i = 0 ; i < extremeCorners.size() ; i++ )
      addDebugPoint( verts[extremeCorners[i]], Vector4f( i&1, i&2, i&4, 1.f ) ) ;

  }
  
  void drawDebugOriginalPts() const
  {
    for( int i = 0 ; i < verts.size() ; i++ )
      addDebugPoint( verts[i], inside(i)?Green:Red ) ;
  }
  
  void drawDebugFaceNormals() const
  {
    for( int i = 0 ; i < finalTris.size() ; i++ )
    {
      Vector3f c = finalTris[i].triCentroid() ;
      addDebugLine( c, c+finalTris[i].plane.normal, Yellow ) ;
    }
  }
  
  void drawDebugRemainingPts( const Vector3f& o, const Vector4f& color ) const {
    // DISPLAY THE remIndices, AND A LINE TO THEIR CLOSEST FUCKING HULL FACE
    for( int i = 0 ; i < remIndices.size() ; i++ )
    {
      const Vector3f& pt = verts[remIndices[i]] ;
      if( inside( remIndices[i] ) ) 
        ; // addDebugPoint( o + pt, Green ) ;
      else
      {
        Vector3f ptOnTri ;
        distanceToClosestTri( remIndices[i], ptOnTri ) ;
        addDebugPoint( o + pt, Red ) ;
        addDebugLine( o + pt, o + ptOnTri, Red ) ;
      }
    }
  }
  
  // You want to bind this pt too.
  void addPtToBound( const Vector3f& pt ) {
    // I'll only add it if its unique. insert each point only once.
    // If we get past this for loop the pt will be added.
    for( int i = 0 ; i < verts.size() ; i++ )
      if( verts[i].isNear( pt ) ) 
        return ;//we had pt already
    verts.push_back( pt ) ; // is a UNIQUE vertex of the submesh.
  }
  
  bool addTri( int ia, int ib, int ic ) {
    if( ia==ib || ia==ic || ib==ic ){
      //warning( "Degenerate tri %d %d %d", ia, ib, ic ) ;
      return 0 ; // degenerate
    }
    
    // THIS IS THE ONLY WAY TO ADD A TRIANGLE TO THE HULL.
    // There you have a triangle added.
    indices.push_back( ia ) ;    indices.push_back( ib ) ;    indices.push_back( ic ) ;
    //addDebugLine( debugPASS1 + tri.triCentroid(), debugPASS1 + tri.triCentroid() + tri.plane.normal*0.1f, Yellow ) ;
    return 1 ;
  }
  
  // You have to find ALL faces that can "see" pt,
  // otherwise the polyhedron could become concave again.
  // expand the hull to include pt.
  void expandToInclude( int pti ) 
  {
    // 3 new tris imposed, then triIndex needs to be DELETED,
    vector<int> facesThatPtCanSee ; // a single index that looks up into indices
    // is only needed to specify a tri.
    // the triangle is 3 verts, but the other 2 can be gotten by i+1, and i+2.
    // we go through this backwards, so that in removal, the indices of indices marked for removal
    // aren't going to be wrong AS WE'RE REMOVING THEM
    for( int i = (int)indices.size()-3 ; i >= 0 ; i-=3 )
    {
      Plane plane( verts[indices[i]], verts[indices[i+1]], verts[indices[i+2]] ) ;
      if( plane.distanceToPoint( verts[pti] ) >= 0.f )
        facesThatPtCanSee.push_back( i ) ; // keep the index into the index array. the other 2 are implied by
        // being the next 2 in the index array.
    }
        
    // So, ALL THOSE FACES MUST BE REMOVED.
    // the perimeter is the set of edges that only occur ONCE.
    vector<IEdge> perimeter ; // the set of edges that see 
    for( int i = 0 ; i < facesThatPtCanSee.size() ; i++ ) // Each ONE is a tri.
    {
      int si = facesThatPtCanSee[i] ; /// starting index in indices for the tri.
      int i1=indices[si+0], i2=indices[si+1], i3=indices[si+2] ;
      //info( "Face starting at indices[%d], indices are (%d %d %d)", si, i1, i2, i3 ) ;
      
      // We have to analyze 3 edges at a time, but its not importnat they're distinctly named
      IEdge edges[3] = { IEdge( i1,i2 ), IEdge( i2,i3 ), IEdge( i3,i1 ) } ;
      
      // If any of the edges ONLY OCCURS ONCE in the facesThatPtCanSee set, then it is a horizon edge.
      for( int edgeNo = 0 ; edgeNo < 3 ; edgeNo++ )
      {
        IEdge &edge = edges[edgeNo] ; // This is the edge we're trying to see if is unique the selected set
        bool isHorizonEdge = 1 ; // assume its unique
        
        // Next loop tries to eliminate the possibility that it is a horizon edge
        // by checking for another occurrence of that same edge in the faceTahtPtCanSee group.
        for( int j = 0 ; isHorizonEdge &&  // we still belive it could be a horizon edge
             j < facesThatPtCanSee.size() ; j++ )
        {
          if( j==i ) skip ;  // Don't check the triangle i came from, to see if there is an edge repeat.
          
          // at each face in the faces that pt can see set, we get the 3 edges of the face in jedges.
          int jsi = facesThatPtCanSee[j] ;
          int j1=indices[jsi+0], j2=indices[jsi+1], j3=indices[jsi+2] ;
          
          // Now see if the edge we're checking is the same as any of the 3 edges of this face.
          IEdge jedges[3] = { IEdge( j1,j2 ), IEdge( j2,j3 ), IEdge( j3,j1 ) } ; // 3 edges of the face
          for( int jEdgeNo=0 ; jEdgeNo < 3 ; jEdgeNo++ )
          {
            if( edge.same(jedges[jEdgeNo]) ) {
              isHorizonEdge=0; // its the same. 
              //info( "Edge %d %d same as %d %d", edge.a,edge.b, jedges[jEdgeNo].a,jedges[jEdgeNo].b ) ;
              break ;
            }
          }
        }
        
        if( isHorizonEdge )  perimeter.push_back( edge ) ;
      }
    }
    
    
    //info( "pti could see %d faces, perimeter has %d edges", facesThatPtCanSee.size(), perimeter.size() ) ;
    // we then walk the PERIMETER
    // since the tris were all wound ccw, yo actuall just wind tris in ANY ORDER,
    // fan them out from (newpoint, edge.a, edge.b ) ;
    for( int i = 0 ; i < perimeter.size() ; i++ )
      addTri( pti, perimeter[i].a, perimeter[i].b ) ;
    
    // REMOVE ALL OF THE OLD FACES
    // can either delete or just _don't copy_
    //vector<int> newIndices ;
    for( int i = 0 ; i < facesThatPtCanSee.size() ; i++ ) // Each ONE is a tri.
    {
      // delete __3__ entries from indices starting from facesThatPtCanSee[i]
      indices.erase( indices.begin() + facesThatPtCanSee[i],
                     indices.begin() + facesThatPtCanSee[i]+3 ) ; // say deletes 9,10 and 11
    }
  }
  
  void getFinalPts()
  {
    // Quickly filter the nonunique indices
    set<int> uniqueIndices( indices.begin(), indices.end() ) ;
    for( set<int>::iterator iter = uniqueIndices.begin() ; iter != uniqueIndices.end() ; ++iter )
      finalPts.push_back( verts[*iter] ) ;
    
    for( int i = 0 ; i < indices.size() ; i+=3 )
    {
      // Now keep the normals, for SAT tests.
      Triangle tri( verts[indices[i]], verts[indices[i+1]], verts[indices[i+2]] ) ;
      finalTris.push_back( tri ) ;
      
      // If we don't already have a normal like that,
      bool had=0;
      for( int j = 0 ; j < finalNormals.size(); j++ )
        // For SAT testing, lack of overlap in direction `normal` will
        // also be lack of overlap in direction `-normal`, no need to check both.
        if( finalNormals[j].isNear(  tri.plane.normal ) ||
            finalNormals[j].isNear( -tri.plane.normal ) ) {
          had=1;
          break;
        }
      if(!had)
        finalNormals.push_back( tri.plane.normal ) ;
    }
  }
  
  void expandToContainAllPts()
  {
    // repeatedly:
    // 1) trim inside pts
    // 2) find furthest out pt
    // 3) poke face
    while( remIndices.size() )
    {
      // 1) delete any points INSIDE the hull
      // any pts inside that hull (ALL on +halfspace)
      removeRemPtsInside() ;
      
      // If you just engulfed all the remaining pts with the last expansion, you're done.
      if( !remIndices.size() )  break ;
      
      // 2) get the FURTHEST point
      // the point with the BIGGEST, (MINIMUM distance to the hull)
      Vector3f closestPtOnTri ;
      // Assume first pt is the right answer,
      int fi = 0 ; // furthestI. I use an INDEX into remIndices to explicitly delete it after.
      float furthestDistance = 0 ;
      for( int i = 0 ; i < remIndices.size() ; i++ )
      {
        Vector3f ptOnTri ;
        float dist = distanceToClosestTri( remIndices[i], ptOnTri ) ;
        if( dist == HUGE ) {
          //error( "Point %d is INSIDE the hull.",remIndices[i] ) ;
          //addDebugPoint( debugPASS1, Blue ) ;
          skip;
        }
        else if( dist > furthestDistance )
        {
          fi = i ;
          furthestDistance = dist ;
          closestPtOnTri = ptOnTri ;
        }
      }
      
      //info( "Point %d has furthest distance with %f units", remIndices[fi], furthestDistance ) ;
      //addDebugLine( debugPASS1 + verts[furthestOut], debugPASS1 + closestPtOnTri, Orange ) ;
      // 3) EXPAND THE HULL
      expandToInclude( remIndices[fi] ) ;
      
      // REMOVE that furthest pt
      remIndices.erase( remIndices.begin() + fi ) ;  // although removeInside() ;shoudl remove,
      // technically "inside" the hull,
      // floating point error is bound to fuck this up soon or later. so we manually ensure its removed.
    }
    
    getFinalPts() ;    
  }
  
  // Tri normals point OUTSIDE the hull, so,
  // if you are on the + side of any tri you are NOT INSIDE
  // Because we're trying to simplify the hull, EXTREMELY CLOSE POINTS
  // are considered OUTSIDE

  bool inside( int pti ) const {
    for( int i = 0 ; i < (int)indices.size() ; i+=3 )
    {
      // Reconstructing the plane IS expensive, but this code isn't optimized there.
      Plane plane( verts[indices[i]], verts[indices[i+1]], verts[indices[i+2]] ) ;
      
      // if the dist below is -, it means the pt is INSIDE according to this face.
      // if it is +, it means it is DEFINITELY OUTSIDE THE SHAPE since it is outside at least 1 face.
      if( plane.distanceToPoint( verts[pti] ) > tolerance ) // ITS NOT INSIDE.  The larger tolerance is, the more willing
      // I am to say "Ok, you're still inside" even though you are slightly out of line / slightly outside.
      // this eps is important for points you just added to the hull.
        return 0 ;
    }
    return 1 ; // you are inside all the planes
  }
  
  // Remove all remaining candidate pts from remIndices that
  // are now INSIDE THE HULL.
  void removeRemPtsInside()
  {
    // 0 1 2 3 4 5 6 7 8
    // delete 4, 5, 6, 8
    // 
    for( int i = (int)remIndices.size()-1 ; i >= 0 ; --i )
      if( inside( remIndices[i] ) )
      {
        //info( "Removing %d", remIndices[i] ) ;
        remIndices.erase( remIndices.begin() + i ) ;
      }
  }
  
  // Get you the distance to the CLOSEST triangle in the hull.
  // The point is outside already, now, get me the SMALLEST distance,
  float distanceToClosestTri( int pti, Vector3f& closestPtOnTri ) const {
    float minDist=HUGE ;
    for( int i = 0 ; i < indices.size() ; i+=3 )
    {
      Triangle tri( verts[indices[i]], verts[indices[i+1]], verts[indices[i+2]] ) ;
      Vector3f ptOnTri ;
      int type=0;
      float dist = tri.distanceToPoint( verts[pti], ptOnTri, type ) ;
      
      Vector4f color = Red;
      if( type==0 )  color = Green ;
      else if( type==1 ) color = Blue ;
      
      if( dist > 0.f ){
        //addDebugPoint( debugPASS1+verts[pti], color ) ;
        //addDebugLine( debugPASS1+verts[pti], debugPASS1+ptOnTri, color ) ;
      }
      // If you line up with another plane on the OTHER side,
      // the distance will be negative, and inadmissible.      
      if( dist > 0.f && dist < minDist )
      {
        minDist=dist;
        closestPtOnTri = ptOnTri ;
      }
    }
    
    //addDebugPoint( debugPASS1 + p, Red ) ;
    return minDist ;
  }
  
  bool intersectsTri( const Triangle& tri ) const {
    
    float meMin, meMax, oMin, oMax ;
    // Start with tri normal.  Only need to test 1 pt from tri, since all 3 will collapse to same pt.
    SATtest( tri.plane.normal, finalPts, meMin, meMax ) ;
    SATtest( tri.plane.normal, &tri.a, 3, oMin, oMax ) ;
    if( !overlaps( meMin, meMax, oMin, oMax ) )
      return 0 ;
  
    // Now test the hull's normals against tri's pts
    //vector<Vector3f> triPts ;
    //triPts.push_back( tri.a ) ;  triPts.push_back( tri.b ) ;  triPts.push_back( tri.c ) ;
    for( int i = 0 ; i < finalNormals.size() ; i++ )
    {
      SATtest( finalNormals[i], finalPts, meMin, meMax ) ;
      SATtest( finalNormals[i], &tri.a, 3, oMin, oMax ) ;
      if( !overlaps( meMin, meMax, oMin, oMax ) )
        return 0 ;
    }
    
    return 1 ;
  }
  
  bool intersectsHull( const Hull& o ) const {
    // Get the normals for one of the shapes,
    float meMin, meMax, oMin, oMax ;
    
    for( int i = 0 ; i < finalNormals.size() ; i++ )
    {
      SATtest( finalNormals[i], finalPts, meMin, meMax ) ;
      SATtest( finalNormals[i], o.finalPts, oMin, oMax ) ;
      if( !overlaps( meMin, meMax, oMin, oMax ) )
        return 0 ; // NO OVERLAP IN AT LEAST 1 AXIS, SO NO INTERSECTION
      // otherwise, go on with the next test
    }

    // TEST SHAPE2.normals as well
    for( int i = 0 ; i < o.finalNormals.size() ; i++ )
    {
      SATtest( o.finalNormals[i], finalPts, meMin, meMax ) ;
      SATtest( o.finalNormals[i], o.finalPts, oMin, oMax ) ;
      if( !overlaps( meMin, meMax, oMin, oMax ) )
        return 0 ; // NO OVERLAP IN AT LEAST 1 AXIS, SO NO INTERSECTION
    }
    
    // if overlap occurred in ALL AXES, then they do intersect
    return 1 ;
  }
  
  // rtcd pg 199
  // this is much more efficient than a ray-tri intn on each
  // possible BECAUSE its a convex hull
  bool intersectsRay( const Ray& ray, float &t1, float &t2 ) const {
    // assume intn is .. whole ray.
    t1=0.f,t2=1.f;
    
    // Test EVERY tri..
    for( int i = 0 ; i < finalTris.size() ; i++ )
    {
      // solve t for reaching the plane.
      // The t for reaching the plane would be (-plane.d - normal•ray.start)/(normal • ray.dir)
      float den = finalTris[i].plane.normal.dot( ray.fullLengthDir ) ;
      float dist = -finalTris[i].plane.d - finalTris[i].plane.normal.dot( ray.start ) ;
      
      // If the ray is //l to the plane, but it runs AWAY from the plane,
      // then you'll never hit the convex polyhedron with this ray.
      if( den == 0.f ) {
        if( dist > 0.f ) {
          return 0 ;
        }
      }
      else // ray not parallel to this plane,
      {
        float t = dist/den;
        
        // The denominator being < 0 means the plane normal
        // FACES the ray (dot product of plane normal and ray dir is -ve)
        if( den < 0.f )
        {
          // use the FURTHEST BACK plane hit on there
          if( t > t1 ) t1=t;
        }
        else
        {
          // den > 0.f, so ray runs in same direction as
          // plane normal ie the plane is hit from the back
          // want SMALLEST t for there.
          if( t < t2 ) t2=t;
        }
        
        // if t1 (which tracks the FURTHEST forward facing plane intn found)
        // exceeds t2 (NEAREST back facing plane intn), its a miss b/c the location on the ray
        // of a hit was further on a forward facing plane
        // than on a back facing plane.  Does it make sense to hit
        // a "back facing side" of a (convex) rock BEFORE hitting its "front facing side"? No!
        if( t1>t2 ) return 0 ;
      }
    }

    return 1 ;
  }
  
} ;


#endif