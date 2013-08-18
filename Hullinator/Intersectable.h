#ifndef INTERSECTABLE_H
#define INTERSECTABLE_H

#include "AABB.h"
#include "GLUtil.h"


// Das SAT test.
void SATtest( const Vector3f& axis, const vector<Vector3f>& ptSet, float& minAlong, float& maxAlong );

// Single pt and C style array versions
void SATtest( const Vector3f& axis, const Vector3f& pt, float& minAlong, float& maxAlong ) ;
void SATtest( const Vector3f& axis, const Vector3f* ptSet, int n, float& minAlong, float& maxAlong ) ;

// a collideable sphere
struct Sphere
{
  Vector3f c ;
  float r,r2 ;
  
  Sphere() { r=r2=1.f ; }
  
  Sphere( const Vector3f& center, float radius )
  {
    c = center ;
    r = radius ;
    r2 = r*r ;
  }
  
  bool contains( const Vector3f& pt ) const {
    // vector c to point's squared len INSIDE r2.
    return (pt - c).len2() <= r2 ;
  }
  
  bool intersectsSphere( const Sphere& o ) const {
    float dist2 = (o.c - c).len2() ;
    return dist2 <= ( r2 + o.r2 ) ;
  }
  
  // You want the squared distance between the spheres for some reason
  bool intersectsSphere( const Sphere& o, float& dist2 ) const {
    dist2 = (o.c - c).len2() ;
    return dist2 <= ( r2 + o.r2 ) ;
  }

  // b/c spheres can't be made nonuniform scale is only a float
  Sphere operator*( float scale ) const {
    return Sphere( c*scale, r*scale ) ;
  }
  Sphere operator/( float scale ) const {
    return Sphere( c/scale, r/scale ) ;
  }
  
  // provided in DynamicTriangle
  //static bool intersects( const DynamicTriangle& tri, const Vector3f& center, float r ){
    // 3 ray-tri intersections 
  //  return tri.intersects( center, r ) ;
  //}
  
  ///////////////////////////
  // SEND ME SQUARE RADII
  // STATIC FUNCTIONS
  static bool intersectsSphere( const Vector3f& center1, float rSq1, const Vector3f& center2, float rSq2 )
  {
    float dist2 = (center2 - center1).len2() ;
    return dist2 <= ( rSq1 + rSq2 ) ;
  }
  
  static bool intersectsSphere( const Vector3f& center1, float rSq1, const Vector3f& center2, float rSq2, float &dist2 )
  {
    dist2 = (center2 - center1).len2() ;
    return dist2 <= ( rSq1 + rSq2 ) ;
  }
  
  // SEND ME SQUARE RADII
  static bool intersectsSphere( const Vector3f& center1, float rSq1, const Vector3f& center2, float rSq2, Vector3f& midPt )
  {
    midPt = center2 - center1 ;   //Not really the midpoint. I re(ab)used this var.
    float dist2 = midPt.len2() ;
    midPt = center1 + midPt/2.f ; //ACTUALLY FIND THE MIDPOINT
    return dist2 <= ( rSq1 + rSq2 ) ;
  }
} ;

// Contains RAY/SPHERE, but RAY/TRIANGLE was put
// inside the Triangle class
// t BETWEEN 0 AND LEN, __NOT 0 AND 1__.
struct Ray
{
  Vector3f start, end ;
  float len ;
  Vector3f dir, fullLengthDir ;
  
  // This has got to be allowed to avoid ray construction ALWAYS having
  // recomputeDir get called.
  Ray(){}
  
  Ray( const Vector3f& iStart, const Vector3f& iEnd )
  {
    start=iStart ;
    end=iEnd ;
    recomputeDir() ;
  }
  
  Ray( const Vector3f& iStart, const Vector3f& iNormalizedDirection, float iLen ){
    start = iStart ;
    dir = iNormalizedDirection ;
    len = iLen ;
    fullLengthDir = iNormalizedDirection*len ;
    end = start + fullLengthDir ;
  }
  
  static Ray FromCenter( const Vector3f& center, const Vector3f& iDir, float halfLen ) {
    return Ray( center - iDir*halfLen, center + iDir*halfLen ) ;
  }
  
  Ray& recomputeDir() {
    fullLengthDir = end - start ; // compute the full length direction vector
    len = fullLengthDir.len() ;   // keep len
    dir = fullLengthDir/len ;     // normalized dir
    return *this ;
  }
  
  Ray& wrap( const Vector3f& worldSize ) {
    start.wrap( worldSize ) ;
    end.wrap( worldSize ) ;
    return *this ;
  }
  
  Ray& setStart( const Vector3f& newStart )
  {
    // it keeps the same END so the dir changes
    start = newStart ;
    return recomputeDir();
  }
  
  Ray& setEnd( const Vector3f& newEnd )
  {
    // it keeps the same END so the dir changes
    end = newEnd ;
    return recomputeDir() ;
  }
  
  // MOVES THE END so that the ray still starts at the same pos
  // but has this new length
  // CHANGES: end, len, fullLengthDir
  Ray& setLength( float newLen )
  {
    len=newLen;
    fullLengthDir = dir*len;
    end = start + fullLengthDir;
    return *this ;
  }
  
  // advance or move up along ray
  Ray& advanceStartAlongRay( float t )
  {
    // Change start while keeping end and dir the same
    start += dir * t ;
    len = len - t ; // you have taken t units off len.
    
    // turn dir around, for you have overshot endpoint
    if( len < 0.f ){
      len = - len ;
      dir = - dir ;
    }

    fullLengthDir = dir*len ; // recompute this

    return *this ;
  }
  
  Ray& operator*=( const Vector3f& v ){
    start*=v;  end*=v;
    return recomputeDir() ;
  }
  Ray& operator/=( const Vector3f& v ){
    start/=v;  end/=v;
    return recomputeDir() ;
  }
  Ray& operator+=( const Vector3f& v ){
    start+=v ;  end+=v ;
    return *this ;
  }
  Ray& operator-=( const Vector3f& v ){
    start-=v ;  end-=v ;
    return *this ;
  }
  
  Ray operator/( const Vector3f& v ) const {
    Ray ray( start/v, end/v ) ;
    return ray ;
  }
  Ray operator*( const Vector3f& v ) const {
    Ray ray( start*v, end*v ) ;
    return ray ;
  }
  Ray operator+( const Vector3f& v ) const {
    Ray ray(*this) ; // copying it is cheaper than constructing a new one (no .len() operation)
    ray.start+=v, ray.end+=v ;
    return ray ;
  }
  Ray operator-( const Vector3f& v ) const {
    Ray ray(*this) ;
    ray.start-=v, ray.end-=v ;
    return ray ;
  }
  
  // t should be between 0 and len.
  // if you're using NORMALIZED (0..1) values
  // then manually do start+fullLengthDir*t.
  inline Vector3f at( float t ) const {
    return start + dir*t ;
  }
  
  inline Vector3f at01( float t ) const {
    return start + fullLengthDir*t ;
  }
  
  // 0, 1 or 2 hits.
  int intersectsSphere( const Sphere& sphere, Vector3f& closerPt, Vector3f& fartherPt ) const
  {
    Vector3f f = start - sphere.c ;

    //float a = 1.f ; // dir.dot(dir); // dir is normalized so it will always be a 1.0f
    float b = 2.f*f.dot( dir ) ;
    float c = f.dot( f ) - sphere.r2 ;
    
    float disc = b*b - 4.f*c ;
    if( disc < 0.f )  return 0 ;
    
    disc = sqrtf( disc ) ;
    float t1 = (-b-disc) / 2.f ; // t1 is ALWAYS SMALLER than t2
    float t2 = (-b+disc) / 2.f ;
    
    //// 4x HIT cases:
    //       -o->       --|-->  |            |  --|->
    // Impale(t1,t2), Poke(t1,t2>len), ExitWound(t1<0, t2), 
    //
    // | -> |
    // CompletelyInside(t1<0, t2>len)
    if( isBetweenOrdered( t1, 0.f, len ) )
    {
      // Poke or, Impale cases
      closerPt = at( t1 ) ;        // t1 hits.
      if( isBetweenOrdered( t2, 0.f, len ) ) {// impale
        fartherPt = at( t2 ) ;
        return 2 ;
      }
      else { // poke
        fartherPt = closerPt ; // the "farther pt" is the same as the closer pt (only 1 intn)
        return 1 ;
      }
    }
    
    else if( isBetweenOrdered( t2, 0.f, len ) ) // only t2 valid, ExitWound, since not impale
    {
      // t1 MUST BE < 0.f here ( t1 happened BEFORE START OF RAY ), you
      // are starting INSIDE the sphere.
      closerPt=fartherPt=at( t2 ) ;
      return 1 ;
    }
    else if( t1 < 0.f && t2 > len ) // CompletelyInside
    {
      // t1 occurred behind start of ray, and t2 occurred BEYOND end of ray
      // technically YOU DID NOT HIT THE SPHERE, but you are inside it.
      // So I return TRUE here and give you the start and end of you ray.
      closerPt = start ;
      fartherPt = end ;
      return 1 ;
    }
    
    // 2x MISS cases:
    //       ->  o                     o ->              
    // FallShort (t1>len,t2>len), Past (t1<0,t2<0)
    return 0 ;
  }
  
  // Yes or no answer
  bool intersectsSphere( const Sphere& sphere ) const
  {
    //If E is the starting point of the ray,
    //.. and L is the end point of the ray,
    //.. and C is the center of sphere you're testing against
    //.. and r is the radius of that sphere
    Vector3f f = start - sphere.c ; // vector from center sphere to ray start
    
    //float a = 1.f ; // dir.dot(dir); // dir is normalized so it will always be a 1.0f
    float b = 2.f*f.dot( dir ) ;
    float c = f.dot( f ) - sphere.r2 ;
    
    float disc = b*b - 4.f*c ; //*a ;
    if( disc < 0.f )  return 0 ;

    disc = sqrtf( disc ) ;
    float t1 = (-b-disc) / 2.f ; //originally float t1 = (-b-disc)/(2.f*a);
    float t2 = (-b+disc) / 2.f ;
    
    //// 4x HIT cases:
    //       -o->       --|-->  |            |  --|->
    // Impale(t1,t2), Poke(t1,t2>len), ExitWound(t1<0, t2), 
    //
    // | -> |
    // CompletelyInside(t1<0, t2>len)
    
    // 2x MISS cases:
    //       ->  o                     o ->              
    // FallShort (t1>len,t2>len), Past (t1<0,t2<0)
    
    return( (t1 >= 0.f && t1 <= len) || // Poke, Impale
            (t2 >= 0.f && t2 <= len) || // ExitWound
            (t1 < 0.f  && t2 > len) // CompletelyInside
          ) ;
  }
  
  // Positive if hit, negative if fail.
  // Returns the forward distance from the
  // ray origin to the penetration point.
  float intersectsSphereDistance( const Sphere& sphere ) const
  {
    Vector3f f = start - sphere.c ; // vector from center sphere to ray start
    
    float a = 1.f ; // dir.dot(dir); // dir is normalized so it will always be a 1.0f
    float b = 2.f*f.dot( dir ) ;
    float c = f.dot( f ) - sphere.r2 ;
    
    float disc = b*b - 4.f*a*c ;
    if( disc >= 0.f )
    {
      disc = sqrtf( disc ) ;
      float t1 = (-b-disc)/(2.f*a);
      float t2 = (-b+disc)/(2.f*a);
      
      if( t1 < 0.f && t2 > len )
      {
        // t1 has to move backward to hit,
        // t2 has to move forward to hit
        // THIS HAPPENS A LOT when you shoot a missile and you're already inside
        // the bv of your target.
        //info( "RAY COMPLETELY INSIDE SPHERE" ) ;
        
        // the t returned is supposed to be ON THE SURFACE OF THE SPHERE,
        // so you could push it out to where the exit point would be..
        // but that would be bad.  maybe just return ORIGIN of ray (0)
        return 0.f ;
      }
      
      else if( t1 >= 0.f && t1 <= len )
      {
        // t1 hit and t1 is closer than t2 (always)
        return t1 ;
      }
      
      else if( t2 >= 0.f && t2 <= len )
      {
        // t2 hit, exit wound
        return t2 ;
      }
    }
    
    // fail
    return -1.0f ;
  }
  
  //float intersectsSphereDistance( const Sphere& sphere ) const {
  //  return intersectsSphereDistance( sphere.c, sphere.r ) ;
  //}
  
  // t between 0 and 1
  inline float normalizedDistanceToPoint( const Vector3f& pt ) const {
    // http://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html
    // The 2nd factor of length NORMALIZES the t, so its range is between 0 and 1.
    return - ( ( start - pt ).dot( fullLengthDir ) / fullLengthDir.len2() ) ;
  }
  
  float normalizedDistanceToPoint( const Vector3f& pt, Vector3f& ptOnLine ) const {
    float t = normalizedDistanceToPoint( pt ) ;
    ptOnLine = at( t*len ) ; // do not change t
    return t ;
  }
  
  // the REAL distance
  float distanceToPoint( const Vector3f& pt ) const {
    return len * normalizedDistanceToPoint( pt ) ;
  }
  
  // Gets you the ray's |_ distance to the point.
  float distanceToPoint( const Vector3f& pt, Vector3f& ptOnLine ) const {
    float t = normalizedDistanceToPoint( pt ) ;
    t *= len ;
    ptOnLine = at( t ) ;
    return t ;
  }
  
  
} ;





enum PlaneSide
{
  // You are definitely behind the plane
  Behind     = -1,

  // You are IN the plane (single point), or
  // STRADDLING the plane (multiple points)
  Straddling =  0,

  // You are definitely in front of the plane.
  InFront    =  1
} ;

struct Plane
{
  // the plane normal
  Vector3f normal ;

  // d as in Ax+By+Cz+D=0. Note how I move D to the LEFT SIDE.
  // d is distance to push plane ALONG NORMAL
  // to get back to origin.  -d is the distance from the origin, then.
  // To get a point from the origin straight to the plane, use normal*-d.
  float d ;
  
  // Default plane (0,1,0)
  Plane() : normal(0,1,0)
  {
  }

  // WIND THE TRIANGLE CCW!
  Plane( const Vector3f& A, const Vector3f& B, const Vector3f& C ) {
    recomputePlane(A,B,C);
  }
  
  Plane( const Vector3f& iNormal, float iDistanceFromOrigin )
  {
    normal = iNormal ;
    d = -iDistanceFromOrigin ; // d is distance to push plane ALONG NORMAL
    // to get back to origin.  so you assign -d.
  }
  
  Plane( const Vector3f& iNormal, const Vector3f& ptInPlane )
  {
    normal = iNormal ;
    
    // The sign gets flipped on d, because d is actually the distance
    // to translate A CANDIDATE POINT in the direction of the normal
    // to see if it passes through the plane described by `normal` that
    // passes thru the originl
    d = - normal.dot( ptInPlane ) ; // Ax+By+Cz+d=0,
    // and normal.dot(Pt) is (Ax+By+Cz).  Thus d=-(Ax+By+Cz),
    // or d = -normal.dot(Pt).
  }

  // 0 for 'x', 1 for 'y', 2 for 'z'
  Plane( int axisIndex, float perpendicularDistance )
  {
    d = -perpendicularDistance ;
    normal.elts[ axisIndex ] = 1 ; // rest are 0 (set by def ctor)
  }
  
  // If you construct a plane from 3 collinear pts, the plane construction fails
  bool isDegenerate() const {
    return normal.allzero() ;
  }
  
  void recomputePlane( const Vector3f& A, const Vector3f& B, const Vector3f& C )
  {
    // the normal is just the cross of 
    // 2 vectors in the plane, normalized
    Vector3f BA = A - B ;
    Vector3f BC = C - B ;

    normal = BC.cross( BA ) ;

    if( normal.allzero() )
    {
      //error( "Triangle has 3 vertices that are collinear, (ie two vertices may be the same) the normal is 0" ) ;
      d=0;// zero normal means d=0 as well
      return ;
      //__debugbreak() ;
    }
    
    // now normalize this.
    normal.normalize() ;

    // http://en.wikipedia.org/wiki/Line-plane_intersection
    // use the normal to compute the d
    d = - normal.dot( A ) ;
  }

  // Returns true if the 3 points are on the same LINE
  static bool areCollinear( const Vector3f& A, const Vector3f& B, const Vector3f& C )
  {
    Plane p( A,B,C ) ;
    if( p.normal.len2() == 0.f )
      return true ;
    else
      return false ;
  }

  // classifies what side of the plane / triangle you are on
  // (+side, 0 on plane, -side)
  // ax + by + cz + d = { 0  point on plane
  //                    { +  point "in front" of plane, (ie on side of normal)
  //                    { -  point "behind" plane (ie on side that normal is NOT on).
  //inline real side( const Vector& p ) const {
  //  return (normal • p) + d ;
  //}

  // Could experiment with thickness and see if it
  // makes this better, for whatever reason.
  inline PlaneSide iSide( const Vector3f& p ) const
  {
    float dist = distanceToPoint( p ) ;
    
    // first see if it's within some epsilon of 0
    // the bigger the eps, the fatter the plane
    if( isNear( dist, 0.0f, EPS_MIN ) )  return PlaneSide::Straddling ;
    else if( dist > 0 ) return PlaneSide::InFront ;
    else return PlaneSide::Behind ;
  }
  
  // Just convenient functions to get which side of plane you're on
  // pSide is +Side, nSide is -side
  inline bool pSide( const Vector3f& p ) const { return distanceToPoint( p ) > 0.f ; }
  inline bool nSide( const Vector3f& p ) const { return distanceToPoint( p ) < 0.f ; }
  
  // 0: ON PLANE EXACTLY (almost never happens)
  // +: >0: IN FRONT of plane (on normal side)
  // -: <0: BEHIND plane (ON OPPOSITE SIDE OF NORMAL)
  inline float distanceToPoint( const Vector3f& p ) const {
    return normal.dot( p ) + d ;
  }
  
  Vector3f projectPointIntoPlane( const Vector3f& p ) const {
    // take p and MOVE IT BACK `distanceToPoint` units
    // in the direction of the normal.
    return p - normal * distanceToPoint( p ) ;
  }

  // You want the distance back too
  Vector3f projectPointIntoPlane( const Vector3f& p, float &dist ) const {
    return p - normal * (dist=distanceToPoint( p )) ;
  }

  // TRUE if all points on same side,
  // FALSE if at least one point on a different side
  bool sameSide( const vector<Vector3f>& pts ) const
  {
    int sign = iSide( pts[0] ) ; 
    for( int i = 1 ; i < pts.size() ; i++ )
      if( sign != iSide( pts[i] ) )
        return false ;
    return true ;
  }
  
  bool sameSide( const Vector3f* pts, int n ) const
  {
    int sign = iSide( pts[0] ) ; 
    for( int i = 1 ; i < n ; i++ )
      if( sign != iSide( pts[i] ) )
        return false ;
    return true ;
  }
  
  // UNTESTED
  bool isIntersectsRay( const Ray& ray ) const {
    // The signed distance must be different to hit the plane
    // this is 2 dots and 2 adds and 6 boolean evals and 5 &&/|| logical operations
    // it's used for when YOU DON'T WANT t,
    return signDiffers( distanceToPoint( ray.start ), distanceToPoint( ray.end ) ) ;
  }

  // the `t` for which the plane intersects the ray 
  float intersectsRay( const Ray& ray ) const
  {
    // this is the cosine of the angle between the plane normal
    // and the ray direction.  It tells you, does the ray
    // run away from the plane or towards it?  If this value is
    // 0: the ray runs || PARALLEL to the PLANE (normal |_ ray.dir)
    // >0: the ray runs AWAY from the plane, so there is only a hit if 
    //     the ray start is behind the plane.
    // <0: the ray runs TOWARDS the plane, so there is only a hit if
    //     the ray starts IN FRONT OF the plane.
    float denom = normal.dot( ray.dir ) ;
    
    // ray is orthogonal to plane normal, and never hits plane
    if( denom == 0.f )
      return HUGE ;  // representing infinity.  this will be OOB the ray of course.
      // But if you DON'T have this if statement then the result below will still WORK,
      // division by 0 gives float INF, which will work reliably.
    
    float t = - (distanceToPoint(ray.start) / denom) ;
    
    return t ;
  }
  
  // yes/no intersection, barycentric coordinates of 
  // intersection, and point of intersection
  // The reason this is all one method and not broekn up is
  // some variables from plane intersection (denom) are REUSED
  // in the barycentric coordinates part.
  bool intersectsRay( const Ray& ray, Vector3f& p ) const
  {
    // first get t for which ray intersects plane
    float t = intersectsRay( ray ) ;
    
    // t<0: intn behind start point (ray shoots away from plane)
    // t>ray.len: intn beyond ray end point
    if( t < 0 || t > ray.len )  return 0 ;
    
    // at this pt, definitely HIT PLANE
    p = ray.at( t ) ; // this is where you hit the plane
    
    return 1 ;
  }
  
  // I give you a ray defining your ray plane intn.
  bool intersectsPlaneUnstable( const Plane& o, Ray &ray ) const {
    ray.dir = normal.cross( o.normal ) ; //9op
    if( ray.dir.allzero() )  return 0 ; // NO INTN, planes are coplanar
    
    ray.dir.normalize() ; // 9op
    
    // find a pt on the ray, that is on both planes.
    Vector3f pt ;
    
    // Simultaneously solve:
    // A1x + B1y + C1z + D1 = 0
    // A2x + B2y + C2z + D2 = 0
    
    // 1. set pt.z=0
    pt.z=0.f;
    
    // Now have 2 eqns in 2 unknowns, instead of 2 eqns in 3 unknowns
    // A1x + B1y + D1 = 0
    // A2x + B2y + D2 = 0   *  -A1/A2  to eliminate x and solve for y

    //  A1x + B1y + D1 = 0
    // -A1x + (-A1/A2)*B2y + (-A1/A2)*D2 = 0   *  -A1/A2  to eliminate x and solve for y
    //  B1y + D1 + (-A1/A2)*B2y + (-A1/A2)*D2 = 0
    
    // To solve for y:
    // B1y - (A1/A2)*B2y = -D1 +(A1/A2)*D2
    // y( B1 - (A1/A2)*B2 ) = -D1 +(A1/A2)*D2
    // y = (-D1 +(A1/A2)*D2) / ( B1 - (A1/A2)*B2 ) ; ////////////SOLVES Y
    
    // Then
    // x = ( -D1 - B1*y ) / A1    //////////SOLVES X
    
    // The best variable to 0 is the one that has near 0 coefficients in C1,C2 anyways,
    // but analysis of this (if( C1 near 0 and C2 near 0 ) then set z=0) is expensive compared to just solving.
    float a1a2 = normal.x/o.normal.x ; // if o.normal.x == 0 then the solution blows up and gives you infinities.
    // that's why this function is marked "unstable".
    
    pt.y = (-d + a1a2*o.d) / ( normal.y - a1a2*o.normal.y ) ;
    pt.x = (-d - normal.y*pt.y ) / normal.x ; // 18op
    
    //36 op so far
    
    // Really just push pt 500 units either way to practically be an infinite line.
    ray.start = pt + ray.dir * -500.f ;
    ray.end = pt + ray.dir * 500.f ;
    ray.fullLengthDir = ray.dir*1000.f ;
    ray.len = 1000.f ; // really its infinite, but 
    return 1 ;
  }
  
  // gets you the ray of intersection 
  //Ray intersectsPlane( const Plane& o ) const {  }
  
  // Intersection of 3 planes, Graphics Gems 1 pg 305
  static Vector3f getIntersection( const Plane& plane1, const Plane& plane2, const Plane& plane3 ) {
    float det = Matrix3f::det( plane1.normal, plane2.normal, plane3.normal ) ; // 17op
    
    // Parallel planes, no intn.
    if( det == 0.f ) {
      error( "No intersection, 3 planes are //l" ) ;
      return 0 ; //could return inf or whatever
    }
    
    // P_intersection = (( point_on1 • n1 )( n2 × n3 ) + ( point_on2 • n2 )( n3 × n1 ) + ( point_on3 • n3 )( n1 × n2 )) / det(n1,n2,n3)
    // which is just
    // P_intersection = (( -d1 )( n2 × n3 ) + ( -d2 )( n3 × n1 ) + ( -d3 )( n1 × n2 )) / det(n1,n2,n3)
    
    // 3 pts in those 3 planes, just from the origin to the plane
    //Vector3f p1 = plane1.normal * -plane1.d ;
    //Vector3f p2 = plane2.normal * -plane2.d ;
    //Vector3f p3 = plane3.normal * -plane3.d ;
    return ( plane2.normal.cross( plane3.normal )*-plane1.d +
             plane3.normal.cross( plane1.normal )*-plane2.d + 
             plane1.normal.cross( plane2.normal )*-plane3.d ) / det ; // 31 op
    // 48 op
  }
  
  static bool getIntersection( const Plane& plane1, const Plane& plane2, Ray& ray ) {
    Plane plane3( plane1.normal.cross( plane2.normal ).normalize(), 0.f ) ;
    if( plane3.isDegenerate() ) {
      error( "No intersection, 2 planes are //l" ) ;
      return 0 ;
    }
    
    Vector3f pt = getIntersection( plane1, plane2, plane3 ) ;
    ray.dir=plane3.normal;
    ray.start = pt ; //+ ray.dir * -500.f ;
    ray.end = pt + ray.dir * 1000.f ;
    ray.fullLengthDir = ray.dir*1000.f ;
    ray.len = 1000.f ; // really its infinite, but 
    return 1 ;
  }
  
  inline bool intersectsPlane( const Plane& o, Ray& ray ) const {
    return Plane::getIntersection( *this, o, ray ) ;
  }
  
  // You want the dist back
  inline bool intersectsSphere( const Sphere& sphere, float& dist ) const {
    // if |_ distance to sphere center is FARther than radius, no hit
    return (dist=fabsf( distanceToPoint( sphere.c ) )) <= sphere.r ;
  }

  inline bool intersectsSphere( const Sphere& sphere ) const {
    return fabsf( distanceToPoint( sphere.c ) ) <= sphere.r ;
  }
  
} ;











struct PrecomputedTriangle ;

struct Triangle
{
  Vector3f a,b,c ;
  Vector3f centroid ;
  Plane plane ; // I often need the plane.
  
  Triangle() {}
  
  // If you go ahead an construct the tirangle (instead of using the static fns)
  // then I'm goig to assume you need the plane.
  Triangle( const Vector3f& ia, const Vector3f& ib, const Vector3f& ic ) :
    a(ia),b(ib),c(ic), plane( a,b,c )
  {
    computeCentroid() ;
  }
  
  // T MUST have a .pos component for this to compile.
  template <typename T>
  Triangle( vector<T>& data, int firstTriNo ) :  // what # triangle is the first one
    a( data[firstTriNo].pos ), b( data[firstTriNo+1].pos ), c( data[firstTriNo+2].pos ),
    
    // I can do this because a,b,c get init first due to a,b,c being BEFORE plane in member listing (above)
    // See http://stackoverflow.com/questions/1242830/constructor-initialization-list-evaluation-order
    plane( a,b,c )
  {
    computeCentroid() ;
  }
  
  Triangle( const PrecomputedTriangle& ptri ) ;
  
  inline void println() const {
    a.print(), b.print( ", " ), c.println( ", " ) ;
  }
  
  Sphere findCircumsphere() {
    // http://mathworld.wolfram.com/Circumcircle.html (useless)
    // See http://mathworld.wolfram.com/Circumsphere.html (useless)
    //
    // These are shewchuk's expressions from http://www.ics.uci.edu/~eppstein/junkyard/circumcenter.html
    //     |                                                           |
    //     | |c-a|^2 [(b-a)x(c-a)]x(b-a) + |b-a|^2 (c-a)x[(b-a)x(c-a)] |
    //     |                                                           |
    // r = -------------------------------------------------------------,
    //                     2 | (b-a)x(c-a) |^2
    
    //     ( abXac.cross(ab)*ac.len2() + ab.len2()*ac.cross( abXac )).len() 
    // r = -------------------------------------------------------------
    //                     2.f*ab.cross(ac).len2()
    
    
    //
    //         |c-a|^2 [(b-a)x(c-a)]x(b-a) + |b-a|^2 (c-a)x[(b-a)x(c-a)]
    // m = a + ---------------------------------------------------------.
    //                           2 | (b-a)x(c-a) |^2
    // m is the circumcenter
    
    // abXac = ab.cross(ac) ;
    // m = a + ac.len2() * ( abXac ).cross(ab) + ab.len2() * ac.cross( abXac ) / 2.f*abXac.len2() ;
    //                           
    
    Vector3f ac = c - a ;
    Vector3f ab = b - a ;
    Vector3f abXac = ab.cross( ac ) ;
    
    // this is the vector from a TO the circumsphere center
    Sphere circumsphere ;
    circumsphere.c = (abXac.cross( ab )*ac.len2() + ac.cross( abXac )*ab.len2()) / (2.f*abXac.len2()) ;
    circumsphere.r2 = circumsphere.c.len2() ;
    circumsphere.r = sqrtf( circumsphere.r2 ) ;
    
    circumsphere.c += a ; // now this is the actual 3space location
    
    return circumsphere ;
  }
  inline Vector3f& computeCentroid() {
    centroid = ( a + b + c ) / 3.f ;
    return centroid ;
  }
  inline Vector3f randomPointInsideTri() const {
    Vector3f bary = Vector3f::randBary() ;
    return a*bary.x + b*bary.y + c*bary.z ;
  }
  inline float area() const {
    return ( (b - a).cross( c - a ) ).len() / 2.f ;
  }
  inline float minEdgeLen() const {
    return min3( (b - a).len(), (c - a).len(), (c - b).len() ) ;
  }
  
  // barycentric coordinates robust check
  bool pointInside( const Vector3f& p, Vector3f& bary ) const
  {
    // christer ericsson's barycentric coordinates
    //            AB        AC        AP
    Vector3f v0 = b-a, v1=c-a, v2=p-a ; // 9
    // Can cache: v0, v1, d00, d01, d11.
    float d00 = v0.dot(v0); //5
    float d01 = v0.dot(v1); //5
    float d11 = v1.dot(v1); //5
    
    float d20 = v2.dot(v0); //5
    float d21 = v2.dot(v1); //5, 34 total
    float denom = d00 * d11 - d01 * d01; //3
    bary.y = (d11 * d20 - d01 * d21) / denom; //4 how much AB 46
    bary.z = (d00 * d21 - d01 * d20) / denom; //4 how much AC 50
    bary.x = 1.f - bary.y - bary.z; //2 how much (??) 
    
    return ( bary.y >= 0.f && bary.z >= 0.f && bary.x >= 0.f ) ;//5
    // 51 total
    
    // bary.x -ve if either v or w is greater than 1
    // so we don't have to check the upper limit of 1
  }

  // You don't want the barycentric coordinates back
  inline bool pointInside( const Vector3f& p ) const {
    Vector3f bary ;    return pointInside( p, bary ) ;
  }
  
  // Finds you the closest point on the triangle to your 3space point.
  // returns SIGNED DISTANCES, so you can simultaneously decide if pt is
  // in front or behind tri
  float signedDistanceToPoint( const Vector3f& pt, Vector3f& closestPtOnTri ) const //, int& type ) const
  {
    //type=0;
    float dist ;  Vector3f bary ;
    if( pointInside( closestPtOnTri=plane.projectPointIntoPlane( pt, dist ), bary ) )
    {
      //addDebugPoint( closestPtOnTri, Magenta ) ; //FACE
      return dist ;
    }
    
    //type=1; // a corner
    int maxBary = bary.maxIndex() ;
    int oBary1 = OTHERAXIS1( maxBary ), oBary2 = OTHERAXIS2( maxBary ) ;
    if( bary.elts[ oBary1 ] < 0.f && bary.elts[ oBary2 ] < 0.f )
    {
      closestPtOnTri = (&a)[ maxBary ] ;
      //addDebugPoint( closestPtOnTri, Yellow ) ; //CORNER
      return signum(dist)*distance1( pt, closestPtOnTri ) ;
    }
    
    //type=2;
    int minBary = bary.minIndex() ;
    oBary1 = OTHERAXIS1( minBary ), oBary2 = OTHERAXIS2( minBary ) ;
    Ray edge( (&a)[oBary1], (&a)[oBary2] ) ;
    float t = edge.distanceToPoint( pt, closestPtOnTri ) ;
    
    // If the t you got was out of range of the edge, then you still fall
    // on a corner
    if( t < 0 ) {
      closestPtOnTri = (&a)[oBary1] ;
      //addDebugPoint( closestPtOnTri, Cyan ) ; //EDGE
      return signum(dist)*distance1( pt, closestPtOnTri ) ;
    }
    else if( t >= edge.len ) {
      closestPtOnTri = (&a)[oBary2] ;
      //addDebugPoint( closestPtOnTri, Cyan ) ;
      return signum(dist)*distance1( pt, closestPtOnTri ) ;
    }
    
    //addDebugPoint( closestPtOnTri, Cyan ) ;
    return signum(dist)*distance1( pt, closestPtOnTri ) ;
  }
  
  // Finds you the closest point on the triangle to your 3space point.
  // pg 141 rtcd.  It might be more efficient than the method I have above,
  // but for points far away from small triangles, because you hit __6__
  // if statements before finally returning a value (in tri).
  Vector3f closestPointOnTri( const Vector3f& p ) const
  {
    Vector3f ab = b - a, ac = c - a, ap = p - a ;
    float d1 = ab.dot(ap), d2 = ac.dot(ap);
    
    // This means 
    if( d1 <= 0.f && d2 <= 0.f ) {
      //addDebugPoint( a, Yellow ) ; //CORNER
      return a ;
    }
    
    Vector3f bp = p - b ;
    float d3 = ab.dot(bp);
    float d4 = ac.dot(bp);
    if( d3 >= 0.f && d4 <= d3 ) {
      //addDebugPoint( b, Yellow ) ; //CORNER
      return b ;
    }
    
    float vc = d1*d4 - d3*d2 ;
    if( vc <= 0.f && d1 >= 0.f && d3 <= 0.f ) {
      float v = d1 / (d1 - d3) ;
      Vector3f closestPtOnTri = a + ab*v ;
      //addDebugPoint( closestPtOnTri, Orange ) ;
      return closestPtOnTri ;
    }
    
    Vector3f cp = p - c ;
    float d5 = ab.dot( cp ) ;
    float d6 = ac.dot( cp ) ;
    if( d6 >= 0.f && d5 <= d6 ) {
      //addDebugPoint( c, Yellow ) ; //CORNER
      return c ;
    }
    
    float vb = d5*d2 - d1*d6 ;
    if( vb <= 0.f && d2 >= 0.f && d6 <= 0.f ) {
      float w = d2 / (d2 - d6) ;
      Vector3f closestPtOnTri = a + ac*w ;
      //addDebugPoint( closestPtOnTri, Orange ) ; // EDGE
      return closestPtOnTri ;
    }
    
    float va = d3*d6 - d5*d4 ;
    if( va <= 0.f && (d4 - d3) >= 0.f && (d5-d6) >= 0.f )
    {
      float w = (d4 - d3 ) / (d4-d3 + d5-d6) ;
      Vector3f closestPtOnTri = b + (c-b)*w ;
      //addDebugPoint( closestPtOnTri, Orange ) ; // EDGE
      return closestPtOnTri ;
    }
    
    float den = 1.f / (va+vb+vc);
    float v = vb*den;
    float w = vc*den;
    Vector3f closestPtOnTri = a + ab*v + ac*w ;
    //addDebugPoint( closestPtOnTri, Magenta ) ; //FACE
    return closestPtOnTri ;
  }
  
  // you only want the DISTANCE to the closest pt on the tri
  inline float distanceToPoint( const Vector3f& pt ) const {
    Vector3f ptOnTri = closestPointOnTri( pt ) ;
    return (pt-ptOnTri).len() ;
  }
  
  // you want both DISTANCE AND closest pt on the tri (UNSIGNED)
  inline float distanceToPoint( const Vector3f& pt, Vector3f& closestPtOnTri ) const {
    closestPtOnTri = closestPointOnTri( pt ) ;
    return (pt-closestPtOnTri).len() ;
  }
  
  // yes/no intersection, barycentric coordinates of 
  // intersection, and point of intersection
  // The reason this is all one method and not broekn up is
  // some variables from plane intersection (denom) are REUSED
  // in the barycentric coordinates part.
  bool intersectsRay( const Ray& ray, Vector3f& p, Vector3f& bary ) const
  {
    // a degenerate tri CANNOT be hit by a ray.
    if( plane.isDegenerate() )  return false ;
    
    // first get t for which ray intersects plane
    float t = plane.intersectsRay( ray ) ;
    
    // t<0: intn behind start point (ray shoots away from plane)
    // t>ray.len: intn beyond ray end point
    if( t < 0 || t > ray.len )  return false ;
    
    // at this pt, definitely HIT PLANE, could still miss triangle.
    p = ray.at( t ) ; // potentially the space location, if barycentric works out
    
    return pointInside( p, bary ) ;
  }
  
  inline bool intersectsRay( const Ray& ray, Vector3f& p ) const {
    Vector3f bary ;    return intersectsRay( ray, p, bary ) ;
  }
  
  inline bool intersectsRay( const Ray& ray ) const {
    Vector3f p, bary ;    return intersectsRay( ray, p, bary ) ;
  }
  
  // tri-tri triangle-triangle
  // see pg 173 of rtcd
  // boolean yes/no intersection.  Moller's method.
  // http://fileadmin.cs.lth.se/cs/Personal/Tomas_Akenine-Moller/pubs/tritri.pdf
  bool intersectsTri( const Triangle& tri ) const
  {
    // We need the distances here actually.  It's central to the algorithm.
    // in the case of an intersection, 2 points will have distances with the same sign,
    // WE DON'T CARE about THAT edge intersection's coords (because .
    
    // Since I need the distances, i compute and keep them here
    float dist1[3] = { tri.plane.distanceToPoint( a ), tri.plane.distanceToPoint( b ), tri.plane.distanceToPoint( c ) } ;
    
    // if all 3 pts of 1 tri are on the same side of the other tri, then
    // no intn can happen
    if( sameSign( dist1[0], dist1[1], dist1[2] ) )  return 0 ;
    
    float dist2[3] = { plane.distanceToPoint( tri.a ), plane.distanceToPoint( tri.b ), plane.distanceToPoint( tri.c ) } ;
    if( sameSign( dist2[0], dist2[1], dist2[2] ) )  return 0 ;
    
    // the paper says you don't need O.
    Vector3f D = plane.normal.cross( tri.plane.normal ).normalize() ;
    if( D.allzero() ) {
      // tris parallel planes.  very rare for 2 tris to actually
      // be coplanar AND intersect, so i consider this a miss.
      //info( "parallel tris" ) ;
      return 0 ;
    }
    
    //float p1[3] = { D.dot( a ), D.dot( b ), D.dot( c ) } ;
    // This approx actually works.
    int axis = D.fabsMaxIndex() ;
    float p1[3] = { a.elts[ axis ], b.elts[ axis ], c.elts[ axis ] } ;
    
    int diffPt1 = whichDifferent( dist1[0], dist1[1], dist1[2] ) ;
    int o1 = OTHERAXIS1( diffPt1 ), o2 = OTHERAXIS2( diffPt1 ) ;
    
    // The 2 t's we want are the ones that cross over the tri plane.
    //    1 +
    //   / \  
    //  /   \  
    // 0-----2  
    // -     -
    // In this case, diffPt1=1, and o1=2, o2=0.
    float t1[2] = {
      p1[diffPt1] + (p1[o1] - p1[diffPt1]) * dist1[diffPt1] / (dist1[diffPt1]-dist1[o1]), // diffPt1->o1
      p1[diffPt1] + (p1[o2] - p1[diffPt1]) * dist1[diffPt1] / (dist1[diffPt1]-dist1[o2])  // diffPt1->o2
    } ;
    
    // Put them in order
    if( t1[0] > t1[1] )  swap( t1[0], t1[1] ) ;
    diffPt1 = whichDifferent( dist2[0], dist2[1], dist2[2] ) ;
    o1 = OTHERAXIS1( diffPt1 ), o2 = OTHERAXIS2( diffPt1 ) ;    
    //float p2[3] = { D.dot( tri.a ), D.dot( tri.b ), D.dot( tri.c ) } ;
    float p2[3] = { tri.a.elts[ axis ], tri.b.elts[ axis ], tri.c.elts[ axis ] } ;
    float t2[2] = {
      p2[diffPt1] + (p2[o1] - p2[diffPt1]) * dist2[diffPt1] / (dist2[diffPt1]-dist2[o1]), // diffPt1->o1
      p2[diffPt1] + (p2[o2] - p2[diffPt1]) * dist2[diffPt1] / (dist2[diffPt1]-dist2[o2])  // diffPt1->o2
    } ;
    if( t2[0] > t2[1] )  swap( t2[0], t2[1] ) ;
    return overlaps( t1[0], t1[1], t2[0], t2[1] ) ;
  }
  
  // gives you the RAY along which there is intersection.
  // loi is lineOfIntersection.  If you don't need the loi, DON'T ASK FOR IT (use the other overload),
  // because the extra computations to get the exact points of tri intercut
  // is very expensive.  If the intersection failed, loi is the ray along which
  // the triangle's planes intersect.
  bool intersectsTri( const Triangle& tri, Ray& loi ) const
  {
    // 1. Dismiss as no intersection if all 3 pts of 1 tri on same side of the other.
    float dist1[3] = { tri.plane.distanceToPoint( a ), tri.plane.distanceToPoint( b ), tri.plane.distanceToPoint( c ) } ;
    if( sameSign( dist1[0], dist1[1], dist1[2] ) )  return 0 ;
    // 2. other tri
    float dist2[3] = { plane.distanceToPoint( tri.a ), plane.distanceToPoint( tri.b ), plane.distanceToPoint( tri.c ) } ;
    if( sameSign( dist2[0], dist2[1], dist2[2] ) )  return 0 ;
    
    // 3. compute the "line of intersection" between the 2 planes
    // "the planes of the triangles must be intersecting in a line. if they're not, dismiss as no intn.
    if( !plane.intersectsPlane( tri.plane, loi ) )  return 0 ;
    //addDebugLine( loi.start - loi.dir*500.f, loi.start + loi.dir*500.f, Purple ) ;
    
    // Can't use the max index approx if need exact pts on loi
    float p1[3] = { loi.dir.dot( a ), loi.dir.dot( b ), loi.dir.dot( c ) } ;
    
    int diffPt1 = whichDifferent( dist1[0], dist1[1], dist1[2] ) ;
    int o1 = OTHERAXIS1( diffPt1 ), o2 = OTHERAXIS2( diffPt1 ) ;
    
    // The 2 t's we want are the ones that cross over the tri plane.
    //    1 +
    //   / \  
    //  /   \  
    // 0-----2  
    // -     -
    // In this case, diffPt1=1, and o1=2, o2=0.
    float t1[2] = {
      p1[diffPt1] + (p1[o1] - p1[diffPt1]) * dist1[diffPt1] / (dist1[diffPt1]-dist1[o1]), // diffPt1->o1
      p1[diffPt1] + (p1[o2] - p1[diffPt1]) * dist1[diffPt1] / (dist1[diffPt1]-dist1[o2])  // diffPt1->o2
    } ;
    
    // Put them in order
    if( t1[0] > t1[1] )  swap( t1[0], t1[1] ) ;
    //addDebugPoint( loi.at( t1[0] ), Vector4f(0.25,0,1) ) ;
    //addDebugPoint( loi.at( t1[1] ), Vector4f(0.25,0,1) ) ;
    
    diffPt1 = whichDifferent( dist2[0], dist2[1], dist2[2] ) ;
    o1 = OTHERAXIS1( diffPt1 ), o2 = OTHERAXIS2( diffPt1 ) ;    
    float p2[3] = { loi.dir.dot( tri.a ), loi.dir.dot( tri.b ), loi.dir.dot( tri.c ) } ;
    //float p2[3] = { tri.a.elts[ axis ], tri.b.elts[ axis ], tri.c.elts[ axis ] } ;
    float t2[2] = {
      p2[diffPt1] + (p2[o1] - p2[diffPt1]) * dist2[diffPt1] / (dist2[diffPt1]-dist2[o1]), // diffPt1->o1
      p2[diffPt1] + (p2[o2] - p2[diffPt1]) * dist2[diffPt1] / (dist2[diffPt1]-dist2[o2])  // diffPt1->o2
    } ;
    
    if( t2[0] > t2[1] )  swap( t2[0], t2[1] ) ;
    //addDebugPoint( loi.at( t2[0] ), Vector4f(0.25,1,0) ) ;
    //addDebugPoint( loi.at( t2[1] ), Vector4f(0.25,1,0) ) ;
    
    float tt1,tt2;
    
    // Just use regular overlaps, not maxOverlaps.
    bool ints = overlaps( t1[0], t1[1], t2[0], t2[1], tt1, tt2 ) ;
    loi = Ray( loi.at( tt1 ), loi.at( tt2 ) ) ;
    //addDebugRay( loi, Yellow ) ;
    
    return ints ;
  }
  
  // 5 ray tri intersections.  actually reasonable.  it gives you the p and barycentric coordinates of the FIRST hit.
  bool intersectsTri( const Triangle& tri, Vector3f& p, Vector3f& bary ) const
  {
    //////if( isDegenerate )  return false ; // WELL, a degenerate splinter can still intersect a fat triangle
    // a degenerate (sliver) tri PRODUCING a ray CAN hit something, but it cannot be HIT BY a ray.

    // 5 ray tri intersections
    return intersectsRay( Ray( tri.a, tri.b ), p, bary ) ||
           intersectsRay( Ray( tri.b, tri.c ), p, bary ) ||
           intersectsRay( Ray( tri.c, tri.a ), p, bary ) ||
           
           // try 2 of THIS tri's sides against the other before calling it quits
           tri.intersectsRay( Ray( a, b ), p, bary ) ||
           tri.intersectsRay( Ray( b, c ), p, bary ) ;
           
           // at least 2 lines will intersect so we don't check the 6th side
           // p is the value of the intersection that hit, if any
           // and early return after 1st hit due to short cct or
  }
  
  // Triangle::intersectsSphere, triangle-sphere
  bool intersectsSphere( const Sphere& sphere ) const 
  {
    // rtcd page 168:
    // 1. test if the sphere intersects the plane of the polygon.  false if not
    if( fabsf( plane.distanceToPoint( sphere.c ) ) > sphere.r ) // if |_ distance to sphere center is FARther than radius, no hit
      return false ;
    
    // 2. 3 ray sphere (this also tests any pts abc INSIDE tri)
    if( Ray( a, b ).intersectsSphere( sphere ) ||
        Ray( b, c ).intersectsSphere( sphere ) ||
        Ray( c, a ).intersectsSphere( sphere ) )
      return true ;
      
    // 3. project sphere center into plane of polygon.  is projected pt in polygon?
    return pointInside( plane.projectPointIntoPlane( sphere.c ) ) ;
  }
  
  
  
  
  // OPERATORS
  // still pretty expensive
  Triangle& operator+=( const Vector3f & trans ) {
    a+=trans, b+=trans, c+=trans, centroid+=trans ;
    plane=Plane(a,b,c);
    return *this ;
  }
  
  
  
  
  
  //////////////////////////
  // STATIC FUNCTIONS //
  static inline Vector3f triCentroid( const Vector3f& a, const Vector3f& b, const Vector3f& c ) {
    return ( a + b + c ) / 3.f ;
  }
  
  static inline Vector3f randomPointInsideTri( const Vector3f& a, const Vector3f& b, const Vector3f& c ) {
    Vector3f bary = Vector3f::randBary() ;
    return a*bary.x + b*bary.y + c*bary.z ;
  }

  static inline float area( const Vector3f& a, const Vector3f& b, const Vector3f& c ) {
    return ( (b - a).cross( c - a ) ).len() / 2.f ;
  }

  // zero normal=degenerate triangle, either same pt, or 2 pts the same, OR even collinear
  // collinear is exceptionally rare but it happens if say for example yz same, x different for ALL 3 points
  static bool isDegenerate( const Vector3f& a, const Vector3f& b, const Vector3f& c ) {
    return ( b - a ).cross( c - a ).allzero() ;
  }

  // a, b, c should be wound CCW
  static inline Vector3f triNormal( const Vector3f& a, const Vector3f& b, const Vector3f& c )
  {
    // CCW NORMAL
    // c---b
    //  \ /
    //   a
    //return ( b - a ).normalize().cross( ( c - a ).normalize() ).normalize() ;
    // You don't need to normalize the intermediate vectors, just the result (it will have the same direction)
    
    Vector3f crossProduct = ( b - a ).cross( c - a ) ;

    // degenerate triangle detection is pretty important and can be the source of mysterious bugs.
    // this usually comes up when you have a 0 triangle. (all 0 points)    
    //if( a == b || a == c || b == c ) // Not checking for equality because it doesn't catch colinear degenerate case
    if( crossProduct.allzero() )
    {
      // degenerate
      //warning( "Looking for the normal of a degenerate triangle" ) ;
      //return Vector3f(0,1,0) ; // to avoid further problems return the y vector
      return crossProduct ; // don't normalize it will be div by 0
    }
    
    return crossProduct.normalize() ;
  }
  
} ;

Triangle operator*( const Matrix4f& matrix, const Triangle& tri ) ;
Triangle operator*( const Matrix3f& matrix, const Triangle& tri ) ;











// The same as a triangle, only it has some of the barycentric
// coordinate parameters precomputed.  This makes runtime cost
// of collisions much less.  Only usable on static geometry.
struct PrecomputedTriangle
{
  Vector3f a,b,c ;
  Vector3f nA, nB, nC ; // you can keep per-vertex normals for use in collision response
  // if you don't initialize these, don't try to use them.

  Vector3f centroid ;
  Sphere circumsphere ; // the circumsphere centroid is different from the triangle centroid
  
  Plane plane ;
  
  Vector3f ab, ac ; // PRECOMPUTED EDGES
  float d00,d01,d11,denomBary ;
  bool isDegenerate ;
  
  // I have to allow this so vectors of value-type PreTris are allowed.
  PrecomputedTriangle() {}
  
  PrecomputedTriangle( const Vector3f& ia, const Vector3f& ib, const Vector3f& ic ) :
    a(ia),b(ib),c(ic),plane( ia, ib, ic ) {
    precompute() ;
  }
  
  PrecomputedTriangle( const Vector3f& ia, const Vector3f& ib, const Vector3f& ic,
    const Vector3f& ina, const Vector3f& inb, const Vector3f& inc ) :
    a(ia),b(ib),c(ic),plane( ia, ib, ic ),
    nA(ina), nB(inb), nC(inc) {
    precompute() ;
  }
  
  PrecomputedTriangle( const Triangle& iTri ) :
    a(iTri.a),b(iTri.b),c(iTri.c),plane( iTri.plane ) {
    precompute() ;
  }
  
  inline void println() const {
    a.print(), b.print( ", " ), c.println( ", " ) ;
  }
  
  void findCircumsphere() {
    // http://mathworld.wolfram.com/Circumcircle.html (useless)
    // See http://mathworld.wolfram.com/Circumsphere.html (useless)
    //
    // These are shewchuk's expressions from http://www.ics.uci.edu/~eppstein/junkyard/circumcenter.html
    //     |                                                           |
    //     | |c-a|^2 [(b-a)x(c-a)]x(b-a) + |b-a|^2 (c-a)x[(b-a)x(c-a)] |
    //     |                                                           |
    // r = -------------------------------------------------------------,
    //                     2 | (b-a)x(c-a) |^2
    
    //     ( abXac.cross(ab)*ac.len2() + ab.len2()*ac.cross( abXac )).len() 
    // r = -------------------------------------------------------------
    //                     2.f*ab.cross(ac).len2()
    
    
    //
    //         |c-a|^2 [(b-a)x(c-a)]x(b-a) + |b-a|^2 (c-a)x[(b-a)x(c-a)]
    // m = a + ---------------------------------------------------------.
    //                           2 | (b-a)x(c-a) |^2
    // m is the circumcenter
    
    // abXac = ab.cross(ac) ;
    // m = a + ac.len2() * ( abXac ).cross(ab) + ab.len2() * ac.cross( abXac ) / 2.f*abXac.len2() ;
    //                           
    
    Vector3f ac = c - a ;
    Vector3f ab = b - a ;
    Vector3f abXac = ab.cross( ac ) ;
    
    // this is the vector from a TO the circumsphere center
    circumsphere.c = (abXac.cross( ab )*ac.len2() + ac.cross( abXac )*ab.len2()) / (2.f*abXac.len2()) ;
    circumsphere.r2 = circumsphere.c.len2() ;
    circumsphere.r = sqrtf( circumsphere.r2 ) ;
    
    circumsphere.c += a ; // now this is the actual 3space location
  }
  
  // Precomputes plane, centroid, barycentric vals
  void precompute()
  {
    // Plane precomp
    if( ( isDegenerate = plane.isDegenerate() ) )
      error( "Degenerate triangle" ) ; // issue this notice.  no precomputed tris should be degen.
    
    // centroid precomp
    centroid = (a + b + c) / 3.f ;
    
    // Barycentric precomputations
    ab = b-a, ac=c-a, 
    d00 = ab.dot(ab);
    d01 = ab.dot(ac); 
    d11 = ac.dot(ac); 
    denomBary = d00 * d11 - d01 * d01;
    
    findCircumsphere() ; // uses ab,ac
  }
  
  Vector3f getInterpolatedNormal( const Vector3f& bary ) const {
    return (nA*bary.x + nB*bary.y + nC*bary.z).normalize() ;
  }
  Vector3f at( const Vector3f& bary ) const {
    return a*bary.x + b*bary.y + c*bary.z ;
  }
  
  inline Vector3f randomPointInsideTri() const {
    Vector3f bary = Vector3f::randBary() ;
    return a*bary.x + b*bary.y + c*bary.z ;
  }
  inline float area() const {
    return ( ab.cross( ac ) ).len() / 2.f ;
  }
  inline float minEdgeLen() const {
    return min3( ab.len(), ac.len(), (c - b).len() ) ;
  }
  
  bool pointInside( const Vector3f& p, Vector3f& bary ) const
  {
    // reduced runtime cost for static meshes
    Vector3f v2=p-a ; //3
    float d20 = v2.dot(ab); //5
    float d21 = v2.dot(ac); //5
    bary.y = (d11 * d20 - d01 * d21) / denomBary ; //4
    bary.z = (d00 * d21 - d01 * d20) / denomBary ; //4
    bary.x = 1.f - bary.y - bary.z; //2
    return ( bary.y >= 0.f && bary.z >= 0.f && bary.x >= 0.f ) ;//5
    //26 total
  }
  
  inline bool pointInside( const Vector3f& p ) const {
    Vector3f bary ;    return pointInside( p, bary ) ;
  }
  
  // PrecomputedTriangle::signedDistanceToPoint Finds you the closest point on the triangle to your 3space point.
  float signedDistanceToPoint( const Vector3f& pt, Vector3f& closestPtOnTri ) const //, int& type ) const
  {
    // 1. First get the |_ distance to the triangle's plane.
    float dist ;  Vector3f bary ;
    
    // 2. that pt into plane of polygon.  is projected pt IN tri?
    if( pointInside( closestPtOnTri=plane.projectPointIntoPlane( pt, dist ), bary ) )
    {
      //addDebugPoint( closestPtOnTri, Magenta ) ; //FACE
      return dist ; // that's your distance.  the point is totally above the tri,
      // so the closest pt on thet trI IS the project pt.
    }
    
    // Otherwise, you're near an edge or corner.
    // CORNER:
    // Working from the barycentric coordinates, I get the closest point on the triangle to the hit
    // IF TWO ARE NEGATIVE: the closest point to you is ONE OF THE TRIANGLE CORNERS (the one corresponding
    // to the lone + side.)
    //type=1; // a corner
    int maxBary = bary.maxIndex() ;
    int oBary1 = OTHERAXIS1( maxBary ), oBary2 = OTHERAXIS2( maxBary ) ;
    if( bary.elts[ oBary1 ] < 0.f && bary.elts[ oBary2 ] < 0.f )
    {
      // Now the POINT is the one with maxBary
      closestPtOnTri = (&a)[ maxBary ] ; // a,b, or c exactly.
      return signum(dist)*distance1( pt, closestPtOnTri ) ;
    }
    
    // otherwise, you're on an EDGE (but still possibly a corner)
    int minBary = bary.minIndex() ; // this is the negative one.  the other 2 are +.
    oBary1 = OTHERAXIS1( minBary ), oBary2 = OTHERAXIS2( minBary ) ;
    
    Ray edge( (&a)[oBary1], (&a)[oBary2] ) ;
    float t = edge.distanceToPoint( pt, closestPtOnTri ) ;
    
    // If the t you got was out of range of the edge, then you still fall
    // on a corner
    if( t < 0 ) {
      closestPtOnTri = (&a)[oBary1] ;
      //addDebugPoint( closestPtOnTri, Cyan ) ; //EDGE
      return signum(dist)*distance1( pt, closestPtOnTri ) ;
    }
    else if( t >= edge.len ) {
      closestPtOnTri = (&a)[oBary2] ;
      //addDebugPoint( closestPtOnTri, Cyan ) ;
      return signum(dist)*distance1( pt, closestPtOnTri ) ;
    }
    
    //addDebugPoint( closestPtOnTri, Cyan ) ;
    return signum(dist)*distance1( pt, closestPtOnTri ) ;
  }
  
  // Finds you the closest point on the triangle to your 3space point.
  // pg 141 rtcd.  It might be more efficient than the method I have above,
  // but for points far away from small triangles, because you hit __6__
  // if statements before finally returning a value (in tri).
  Vector3f closestPointOnTri( const Vector3f& p ) const
  {
    Vector3f ap = p - a ;
    float d1 = ab.dot(ap), d2 = ac.dot(ap);
    
    // This means 
    if( d1 <= 0.f && d2 <= 0.f ) {
      //addDebugPoint( a, Yellow ) ; //CORNER
      return a ;
    }
    
    Vector3f bp = p - b ;
    float d3 = ab.dot(bp);
    float d4 = ac.dot(bp);
    if( d3 >= 0.f && d4 <= d3 ) {
      //addDebugPoint( b, Yellow ) ; //CORNER
      return b ;
    }
    
    float vc = d1*d4 - d3*d2 ;
    if( vc <= 0.f && d1 >= 0.f && d3 <= 0.f ) {
      float v = d1 / (d1 - d3) ;
      Vector3f closestPtOnTri = a + ab*v ;
      //addDebugPoint( closestPtOnTri, Orange ) ;
      return closestPtOnTri ;
    }
    
    Vector3f cp = p - c ;
    float d5 = ab.dot( cp ) ;
    float d6 = ac.dot( cp ) ;
    if( d6 >= 0.f && d5 <= d6 ) {
      //addDebugPoint( c, Yellow ) ; //CORNER
      return c ;
    }
    
    float vb = d5*d2 - d1*d6 ;
    if( vb <= 0.f && d2 >= 0.f && d6 <= 0.f ) {
      float w = d2 / (d2 - d6) ;
      Vector3f closestPtOnTri = a + ac*w ;
      //addDebugPoint( closestPtOnTri, Orange ) ; // EDGE
      return closestPtOnTri ;
    }
    
    float va = d3*d6 - d5*d4 ;
    if( va <= 0.f && (d4 - d3) >= 0.f && (d5-d6) >= 0.f )
    {
      float w = (d4 - d3 ) / (d4-d3 + d5-d6) ;
      Vector3f closestPtOnTri = b + (c-b)*w ;
      //addDebugPoint( closestPtOnTri, Orange ) ; // EDGE
      return closestPtOnTri ;
    }
    
    float den = 1.f / (va+vb+vc);
    float v = vb*den;
    float w = vc*den;
    Vector3f closestPtOnTri = a + ab*v + ac*w ;
    //addDebugPoint( closestPtOnTri, Magenta ) ; //FACE
    return closestPtOnTri ;
  }
  
  // you only want the DISTANCE to the closest pt on the tri
  inline float distanceToPoint( const Vector3f& pt ) const {
    Vector3f ptOnTri = closestPointOnTri( pt ) ;
    return (pt-ptOnTri).len() ;
  }
  
  // you want both DISTANCE AND closest pt on the tri (UNSIGNED)
  inline float distanceToPoint( const Vector3f& pt, Vector3f& closestPtOnTri ) const {
    closestPtOnTri = closestPointOnTri( pt ) ;
    return (pt-closestPtOnTri).len() ;
  }
  
  bool intersectsRay( const Ray& ray, Vector3f& p, Vector3f& bary ) const {
    float t = plane.intersectsRay( ray ) ;
    if( t < 0 || t > ray.len )  return false ;
    p = ray.at( t ) ;
    return pointInside( p, bary ) ;
  }
  
  inline bool intersectsRay( const Ray& ray, Vector3f& p ) const {
    Vector3f bary ;
    return intersectsRay( ray, p, bary ) ;
  }
  
  inline bool intersectsRay( const Ray& ray ) const {
    Vector3f p, bary ;
    return intersectsRay( ray, p, bary ) ;
  }
  
  // THere are 2 different things going on here.  Kisses and bites.
  // You need the distance back, and the interpolated barycentric coordinates
  // triangle-sphere
  enum TriSphereIntersectionType{
    TriSphereNOINTERSECTION,
    
    // The sphere "kisses" the plane of the triangle (it has a projected point
    // that is inside the triangle)
    TriSphereKissing,
    
    // The sphere "chomps" the triangle from the side of the triangle.
    TriSphereChompEdge,
    
    // The sphere "chomps" a corner
    TriSphereCorner
  } ;
  
  // PrecomputedTriangle::intersectsSphere
  // Triangle-sphere, but gets you the closest pt on the tri to the sphere
  // as well as the interpolated normal at that pt.
  int intersectsSphere( const Sphere& sphere, Vector3f& bary, Vector3f& closestPt, Vector3f& normalAtPt ) const {
    // rtcd page 168:
    // 1. test if the sphere intersects the plane of the polygon.  false if not
    // if |_ distance to sphere center is FARther than radius, no hit
    float dist = plane.distanceToPoint( sphere.c ) ;
    if( fabsf( dist ) > sphere.r )
      return TriSphereNOINTERSECTION ;

    // 2. project sphere center into plane of polygon.  is projected pt in polygon?
    if( pointInside( closestPt=plane.projectPointIntoPlane( sphere.c ), bary ) )
    {
      // You have to turn the normal around if you are behind the plane.
      normalAtPt = getInterpolatedNormal( bary ) * signum( dist ) ;
      //info( "Kissing intersection" ) ;
      return TriSphereKissing ;
    }
    
    // Working from the barycentric coordinates, I get the closest point on the triangle to the hit
    // IF TWO ARE NEGATIVE: the closest point to you is ONE OF THE TRIANGLE CORNERS (the one corresponding
    // to the lone + side.)
    int maxBary = bary.maxIndex() ;
    int oBary1 = OTHERAXIS1( maxBary ), oBary2 = OTHERAXIS2( maxBary ) ;
    if( bary.elts[ oBary1 ] < 0.f && bary.elts[ oBary2 ] < 0.f )
    {
      // The fix is simple: your barycentric coordinates are 
      bary.elts[ maxBary ] = 1.f,
      bary.elts[ oBary1 ] = bary.elts[ oBary2 ] = 0.f ;
      
      // Now the POINT is
      closestPt = (&a)[ maxBary ] ; // a,b, or c exactly.
      // (if maxBary is 0, then that corresponds to a)
      // alternatively you could evaluate at(bary), but that's a lot more multiplies and adds.
      
      // The tri point must be inside the sphere
      if( distance2( sphere.c, closestPt ) > sphere.r2 )
        return TriSphereNOINTERSECTION ;
      
      normalAtPt = (&nA)[ maxBary ] ; // just that normal
      //info( "Chomping intersection FROM CORNER" ) ;
      return TriSphereCorner ;
    }
    
    int minBary = bary.minIndex() ; // this is the negative one.  the other 2 are +.
    oBary1 = OTHERAXIS1( minBary ), oBary2 = OTHERAXIS2( minBary ) ;
    
    // OR you could just use the |_ distance (assuming only ONE is negative) between the ray
    // from the 2 pts there.
    Ray edge( (&a)[oBary1], (&a)[oBary2] ) ;
    float t = edge.normalizedDistanceToPoint( sphere.c, closestPt ) ;
    
    // Very important check: if you are OOB the ray then nail you to the vertex.
    // t GETS CLAMPED HERE, so use in fetching interpolated normal below remains correct.
    if( t < 0.f ) { t=0.f, closestPt=(&a)[oBary1] ; }
    else if( t > 1.f ) { t=1.f, closestPt=(&a)[oBary2]; }
    
    //printf( "Normalized t %f\n", t ) ;
    if( distance2( sphere.c, closestPt ) > sphere.r2 )
      return TriSphereNOINTERSECTION ;
    
    // hit.
    //info( "Chomping intersection FROM SIDE" ) ;
    
    // The barycentric coordinates of the hit is actually related to t.
    // t is how far along the ray we was.
    bary.elts[minBary] = 0.f ;
    bary.elts[oBary1] = t ;
    bary.elts[oBary2] = 1.f-t;
    
    normalAtPt = getInterpolatedNormal( bary ) ;

    return TriSphereChompEdge ;
    //else      printf( "Pt too far from edge (%f)\n", t*edge.len, r ) ;
  }
  
  inline bool intersectsSphere( const Sphere& sphere ) const {
    Vector3f bary, closestPt, normalAtPt ;
    return intersectsSphere( sphere, bary, closestPt, normalAtPt ) ;
  }
  
  
} ;

PrecomputedTriangle operator*( const Matrix4f& matrix, const PrecomputedTriangle& tri ) ;
PrecomputedTriangle operator*( const Matrix3f& matrix, const PrecomputedTriangle& tri ) ;

PrecomputedTriangle operator+( const Vector3f& disp, const PrecomputedTriangle& tri ) ;

PrecomputedTriangle operator+( const PrecomputedTriangle& tri, const Vector3f& disp ) ;








// Originally this was called VIEWINGPLANE, because it represented
// an EYE BEHIND A FLAT VIEWING PLANE.
// Really this represents a VIEW FRUSTUM however, with an EYE LOCATION included.
// This represents a QUAD
// A frustum.
struct Frustum
{
  // looking INTO the near plane:
  // farB  farA
  //   \    / 
  //    b--a
  //    |  |
  //    c--d
  //   /    \  
  // farC  farD
  
  Vector3f a,b,c,d ;  // ACTUAL SPACE perspective/oriented of the near plane
  
  Vector3f farA,farB,farC,farD ;
  
private:
  Vector3f pa,pb,pc,pd, pfa,pfb,pfc,pfd ;// perspective located, these are used to find
  // a,b,c,d based on some persp() call
  
  // The matrix used to orient the frustum
  // the viewFwd takes a rectangular prism along +z
  // and swings/expands it so it is the actual view frustum described
  // in world space.
  Matrix4f viewFwd ;
  
  // PerspectiveProj and viewBackward are the INVERSE
  // of the frustum transformation.. that is they mimick what
  // the GPU does and takes a WORLD SPACE COORD and places it in
  // the canonical view volume.
  // This is used to determine frustum intersection
  Matrix4f perspectiveProj, viewBackward ;
  
  vector<Plane> planes ;
  
  // If not using true corners the 4 corners at the NEAR PLANE
  // are merged into ONE SINGLE CORNER at the EYE.
  // When not using true corners, the near plane is SKIPPED in intersection testing,
  // only the 5 other planes are used.
  bool useTrueCorners ;

public:
  vector<Vector3f> corners ;

  Vector3f eye ;
  
  // rows and columns in the pixel grid
  int rows, cols ;

  // distances to the near and far planes
  float nearPlane, // how far back the eye is from the near plane
    farPlane ; // distance rays travel before they are "too far" for the eye to see/resolve
  
  // For constructing a glview from these vars
  float fov, aspectRatio ;
  
  Frustum( const Matrix4f& modelViewProjection )
  {
    useTrueCorners=0;
    // you can construct the frustum planes from the mvp matrix.
    // 
    
    fov = M_PI_4 ;
    aspectRatio=1;
  }
  
  // rows and cols are the PIXELS along the near plane. for ray casting.
  Frustum( int pixelRows, int pixelCols ) /// , bool iyFromTop )
  {
    useTrueCorners=0;
    
    rows = pixelRows ;
    cols = pixelCols ;
    //yFromTop=iyFromTop ;

    // initialize the corners with default values
    a = Vector3f( 1, 1, 0 ) ;
    b = Vector3f(-1, 1, 0 ) ;
    c = Vector3f(-1,-1, 0 ) ;
    d = Vector3f( 1,-1, 0 ) ;
    
    fov = M_PI_4 ;
    aspectRatio=1;
  }

  // Sets the near plane distance and ray length
  // and also SIZE of near plane.
  // if the near plane is too small, you will get
  // a nearly orthographic-looking projection
  void persp( float fovyRadians, float aspect, float nearPlaneDist, float farPlaneDist )
  {
    fov = fovyRadians ;
    aspectRatio = aspect ;
    nearPlane = nearPlaneDist ;
    farPlane  = farPlaneDist ;
    
    perspectiveProj = Matrix4f::persp( fovyRadians, aspect, nearPlane, farPlane ) ;

    // compute the SIZE of the near plane
    // tan(fovy)=o/a, nearPlane is opp, HEIGHT is adj
    // a=o/tan(fovy)
    float tanHalfFovy = tanf( fovyRadians/2.f ) ;
    float halfNearPlaneHeight = nearPlane * tanHalfFovy ; // half near plane width

    // aspect=w/h
    float halfNearPlaneWidth = halfNearPlaneHeight*aspect ;
    
    // Viewing plane:
    // b--a
    // |  |
    // c--d
    
    // First I start the eye at (0,0,0)
    eye = Vector3f(0,0,0);
    
    // the near plane is facing down +z IF YOU DON'T CHANGE THE HANDEDNESS
    // OF THE VIEWFWD TRANSFORM.
    pa.z=pb.z=pc.z=pd.z= -nearPlane ;
    
    pa.x = pd.x =  halfNearPlaneWidth ;
    pb.x = pc.x = -halfNearPlaneWidth ;
    
    pa.y = pb.y =  halfNearPlaneHeight ;
    pc.y = pd.y = -halfNearPlaneHeight ;
    
    
    //info( "Near plane width %.2f, h=%.2f", h_nearPW, h_nearPH ) ;
    float halfFarPlaneHeight = farPlane * tanHalfFovy ;
    float halfFarPlaneWidth = halfFarPlaneHeight*aspect ;
    
    pfa.z=pfb.z=pfc.z=pfd.z= -farPlane ;
    pfa.x = pfd.x =  halfFarPlaneWidth ;
    pfb.x = pfc.x = -halfFarPlaneWidth ;
    
    pfa.y = pfb.y =  halfFarPlaneHeight ;
    pfc.y = pfd.y = -halfFarPlaneHeight ;
    
  }
  
  void changeFovBy( float dFov )
  {
    fov += dFov ;
    
    // fov must always be between PI/4 and PI/2
    ::clamp( fov, M_PI_4, M_PI_2 ) ;
    
    // You have to reconstruct the frustum now.  pa,pb etc change.
    persp( fov, aspectRatio, nearPlane, farPlane ) ;
  }
  
  void recomputePlanes()
  {
    // these are all constructed facing OUT
    planes.clear() ;
    
    if( useTrueCorners )
      planes.push_back( Plane( a,b,c ) ) ; // NEAR PLANE
    planes.push_back( Plane( farA,farD,farC ) ) ; // FAR PLANE FACES OTHER WAY

    planes.push_back( Plane( d,farD,farA ) ) ; // left
    planes.push_back( Plane( a,farA,farB ) ) ; // top
    planes.push_back( Plane( b,farB,farC ) ) ; // right
    planes.push_back( Plane( c,farC,farD ) ) ; // bottom
    
  }
  
  
  void recomputeFrustumCorners()
  {
    // Move the viewing plane.
    a = viewFwd * pa ;
    b = viewFwd * pb ;
    c = viewFwd * pc ;
    d = viewFwd * pd ;
    
    farA = viewFwd * pfa ;
    farB = viewFwd * pfb ;
    farC = viewFwd * pfc ;
    farD = viewFwd * pfd ;
    
    // cache these. corny.
    corners.clear() ;
    if( useTrueCorners )
    {
      corners.push_back( a ) ;
      corners.push_back( b ) ;
      corners.push_back( c ) ;
      corners.push_back( d ) ;
    }
    else
      corners.push_back( eye ) ;
    
    corners.push_back( farA ) ;
    corners.push_back( farB ) ;
    corners.push_back( farC ) ;
    corners.push_back( farD ) ;
    
    recomputePlanes() ;
  }
  
  // Unlike rasterization, orienting the viewing plane is a 
  // FORWARD transformation.
  void orient( const Vector3f& iEye, const Vector3f& look, const Vector3f& up )
  {
    eye = iEye ;
    viewFwd = Matrix4f::LookAtFORWARD( eye, look, up ) ; // forward transformation
    recomputeFrustumCorners() ;
  }
  
  void orient( const Vector3f& iEye, const Vector3f& right, const Vector3f& up, const Vector3f& forward )
  {
    eye = iEye ;
    
    viewFwd = Matrix4f::TransformToFaceChangeHandedness( right,up,forward ) ; // forward transformation
    viewFwd.translate(eye);
    
    recomputeFrustumCorners() ;
  }
  
  void orient( const Matrix4f& viewFORWARD )
  {
    viewFwd = viewFORWARD ;
    eye = viewFORWARD.getTranslation() ;
    
    recomputeFrustumCorners() ;
  }
  
  





  // gets you the physical space vector location of a pixel
  // ROWS AND COLS COUNT LEFT-TO-RIGHT, COLS BOTTOM TO TOP
  Vector3f getPixelLocation( int pRow, int pCol ) const
  {
    // find the vector representing pixel row/col
    // on the viewing plane
    // b--a
    // |  |
    // c--d

    Vector3f cd = d - c ; // right
    Vector3f cb = b - c ; // up

    float percRow = ( pRow + 0.5f )/ rows ;
    float percCol = ( pCol + 0.5f )/ cols ;

    //point on viewing plane ray is going thru
    Vector3f v = c + cb*percRow + cd*percCol ;
    return v ;
  }
  
  Vector3f getPixelLocationTOP( int pRow, int pCol ) const
  {
    // find the vector representing pixel row/col
    // on the viewing plane
    // b--a
    // |  |
    // c--d

    Vector3f ba = a - b ; // right
    Vector3f bc = c - b ; // down

    float percRow = ( pRow + 0.5f )/ rows ;
    float percCol = ( pCol + 0.5f )/ cols ;

    //point on viewing plane ray is going thru
    Vector3f v = b + bc*percRow + ba*percCol ;
    return v ;
  }

  Ray getRay( int pRow, int pCol ) const
  {
    Vector3f pixelLoc = getPixelLocation( pRow, pCol ) ;
    Vector3f eyeToPixel = (pixelLoc - eye).normalize() ;
    return Ray( eye, eyeToPixel, farPlane ) ;
  }
  
  
  // can't I just do perspective and
  // world transformation on pt and
  // return if pt is in the box (-1,-1,-1),(+1,+1,+1)?
  
  // SKIPS the near plane, because it usually is NOT important,
  // it excludes like less than a unit of space in front of
  // the camera, PLUS it introduces flickering artifacts
  // sometimes
  bool contains( const Vector3f& pt ) const
  {
    // If the point is 
    for( int i = 0 ; i < planes.size() ; i++ )
    {
      if( planes[i].distanceToPoint( pt ) > 0 )
      {
        return false ;
      }
    }
    return true ;
  }
  
  AABB getAABB() const{
    AABB box ;
    for( const Vector3f& pt : corners )
      box.bound( pt ) ;
    return box ;
  }
  
  //Frustum::intersectsAABB
  bool intersectsAABB( const AABB& aabb ) const
  {
    // SAT:
    // project onto 5+3 axes:
    // (frustum 6-1 and cube 6-3)
     
    // 3 cube axes
    float cubeMin,cubeMax,frustumMin,frustumMax;
    
    for( int i = 0 ; i < AABB::SATAxes.size() ; i++ )
    {
      //SATtest( AABB::SATAxes[i], corners, frustumMin, frustumMax ) ;
      // actually can optimize this a bit by picking out the min/max elt
      frustumMin=HUGE,frustumMax=-HUGE;
      for( int j = 0 ; j < corners.size() ; j++ )
      {
        if( corners[j].elts[i] < frustumMin )  frustumMin=corners[j].elts[i];
        if( corners[j].elts[i] > frustumMax )  frustumMax=corners[j].elts[i];
      }
      
      // we "cheat" here and just pick out the correct index.  b/c
      // SATAxes[0] is the x axis, so we can just pick out the max/min
      // from the AABB max/min members.
      cubeMin=aabb.min.elts[i] ;
      cubeMax=aabb.max.elts[i] ;
      
      // FAIL! there is not an overlap in at least 1 axis.
      // SAT says NO INTERSECTION.
      if( !overlaps( frustumMin, frustumMax, cubeMin, cubeMax ) )
      {
        return 0 ;
      }
    }
    
    // Ok, if the cheaper (cube) tests failed, then test the 5 unique
    // sides of the frustum (I consider the frustum as a square based pyramid, I don't test the near plane)
    // if you're using true corners, SKIP the near plane 
    for( int i = useTrueCorners ; i < planes.size() ; i++ )
    {
      Vector3f axis=planes[i].normal ;
      
      // because the axis is NOT axis aligned, there is a more thorough test needed for the cube
      // (an actual projection)
      SATtest( axis, aabb.corners, cubeMin, cubeMax ) ;
      SATtest( axis, corners, frustumMin, frustumMax ) ;
      if( !overlaps( frustumMin, frustumMax, cubeMin, cubeMax ) )
      {
        return 0 ;
      }
    }
    
    // there are overlaps in ALL axes, so the objects intersect.
    return 1 ;
  }
  
  //Frustum::intersectsSphere
  bool intersectsSphere( const Sphere& sphere ) const
  {
    // Get the closest point on the frustum to the sphere.
    // (consider the frustum as a convex hull and just construct tris)
    
    // OR check i'm within sphere.r of each plane
    for( int i = useTrueCorners ; i < planes.size() ; i++ )
    {
      float dist = planes[i].distanceToPoint( sphere.c ) ;
      
      // If the sphere is way outside one of the planes, it doesn't hit the frustum.
      if( dist > sphere.r )  return 0 ;
    }
    
    return 1 ; // sphere hits the frustum.
  }
  
  void drawPermDebugLines()
  {
    // Draws the near plane
    addPermDebugLine( a, Vector4f(1,0,0,1), b, Vector4f(1,1,0,1) );
    addPermDebugLine( b, Vector4f(1,1,0,1), c, Vector4f(1,0,1,1) );
    addPermDebugLine( c, Vector4f(1,0,1,1), d, Vector4f(1,1,1,1) );
    addPermDebugLine( d, Vector4f(1,1,1,1), a, Vector4f(1,0,0,1) );
    
    // draws the far plane.
    addPermDebugLine( farA, Vector4f(1,0,0,1), farB, Vector4f(1,1,0,1) );
    addPermDebugLine( farB, Vector4f(1,1,0,1), farC, Vector4f(1,0,1,1) );
    addPermDebugLine( farC, Vector4f(1,0,1,1), farD, Vector4f(1,1,1,1) );
    addPermDebugLine( farD, Vector4f(1,1,1,1), farA, Vector4f(1,0,0,1) );
    
    addPermDebugLine( a, Vector4f(1,0,0,1), farA, Vector4f(1,0,0,1) );
    addPermDebugLine( b, Vector4f(1,1,0,1), farB, Vector4f(1,1,0,1) );
    addPermDebugLine( c, Vector4f(1,0,1,1), farC, Vector4f(1,0,1,1) );
    addPermDebugLine( d, Vector4f(1,1,1,1), farD, Vector4f(1,1,1,1) );
  }
  
  void drawDebug()
  {
    // b--a
    // |  |
    // c--d
    /// DRAW THE PLANES AND THEIR NORMALS.
    addDebugQuadSolid( a,b,c,d, Orange*Vector4f(1,1,1,0.5) ) ;
    addDebugQuadSolid( farA,farD,farC,farB, Orange ) ;
    addDebugQuadSolid( d,farD,farA,a, Purple*Vector4f(1,1,1,0.5) ) ;
    addDebugQuadSolid( a,farA,farB,b, Blue*Vector4f(1,1,1,0.5) ) ;
    addDebugQuadSolid( b,farB,farC,c, Yellow*Vector4f(1,1,1,0.5) ) ;
    addDebugQuadSolid( c,farC,farD,d, Red*Vector4f(1,1,1,0.5) ) ;
    
    //if(useTrueCorners){ // b/c uses numeric access into 5.
    //addDebugLine( a, a + planes[0].normal*10, Orange ) ;
    //addDebugLine( farA, farA + planes[1].normal*10, Orange ) ;
    //addDebugLine( d, d + planes[2].normal*10, Purple ) ;
    //addDebugLine( a, a + planes[3].normal*10, Blue ) ;
    //addDebugLine( b, b + planes[4].normal*10, Yellow ) ;
    //addDebugLine( c, c + planes[5].normal*10, Red ) ;
    //}
  }
  
  
  void drawPermDebug()
  {
    addPermDebugQuadSolid( a,b,c,d, Orange*Vector4f(1,1,1,0.5) ) ;
    addPermDebugQuadSolid( farA,farD,farC,farB, Orange ) ;
    addPermDebugQuadSolid( d,farD,farA,a, Purple*Vector4f(1,1,1,0.5) ) ;
    addPermDebugQuadSolid( a,farA,farB,b, Blue*Vector4f(1,1,1,0.5) ) ;
    addPermDebugQuadSolid( b,farB,farC,c, Yellow*Vector4f(1,1,1,0.5) ) ;
    addPermDebugQuadSolid( c,farC,farD,d, Red*Vector4f(1,1,1,0.5) ) ;
  }
} ;



struct TriangleSphereIntersection
{
  // The IMAGE of the sphere doing the intn (need due to wrap)
  //Vector3f sphereCenter ;
  
  // The triangle performing the intn
  //PrecomputedTriangle *tri ;
  
  // The closestPt on TRIANGLE to the sphere center, in other words
  // the DEEPEST PENETRATED point on the triangle into the sphere.
  Vector3f triClosestPt, bary, interpolatedSurfaceNormal ;
  
  // If you want to push the sphere out of the tri, this pt needs to move OUT.
  //Vector3f deepestPtOnSphereIntoTri ;
  
  //Vector3f sphereCentroidToTriCentroid ;
  // Used to select the CLOSEST tri to push back on.
  float distSphereCentroidToTriCentroid ;
  
  // The penetration vector is the correct distance to push the sphere
  // SO IT EXITS THE TRIANGLE IT IS PENETRATING.
  // It DOES NOT use the tri normal because of side triangle "chomps" (intns)
  Vector3f penetration ;
  
  // Distance from PT to SPHERE EDGE (penetration depth)
  float penetrationDepth ;
} ;




#endif
