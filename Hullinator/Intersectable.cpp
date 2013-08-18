#include "Intersectable.h"

// Das SAT test.
void SATtest( const Vector3f& axis, const vector<Vector3f>& ptSet, float& minAlong, float& maxAlong )
{
  minAlong=HUGE, maxAlong=-HUGE;
  for( int i = 0 ; i < ptSet.size() ; i++ )
  {
    // just dot it to get the min/max along this axis.
    float dotVal = ptSet[i].dot( axis ) ;
    if( dotVal < minAlong )  minAlong=dotVal;
    if( dotVal > maxAlong )  maxAlong=dotVal;
  }
}

// Single pt and C style array versions
void SATtest( const Vector3f& axis, const Vector3f& pt, float& minAlong, float& maxAlong ) {
  minAlong=HUGE, maxAlong=-HUGE;
  float dotVal = pt.dot( axis ) ;
  if( dotVal < minAlong )  minAlong=dotVal;
  if( dotVal > maxAlong )  maxAlong=dotVal;
}
void SATtest( const Vector3f& axis, const Vector3f* ptSet, int n, float& minAlong, float& maxAlong ) {
  minAlong=HUGE, maxAlong=-HUGE;
  for( int i = 0 ; i < n ; i++ )
  {
    float dotVal = ptSet[i].dot( axis ) ;
    if( dotVal < minAlong )  minAlong=dotVal;
    if( dotVal > maxAlong )  maxAlong=dotVal;
  }
}

// GLobal function defining Matrix4f*Triangle.
Triangle operator*( const Matrix4f& matrix, const Triangle& tri ) {
  return Triangle( matrix*tri.a, matrix*tri.b, matrix*tri.c ) ;
}
Triangle operator*( const Matrix3f& matrix, const Triangle& tri ) {
  return Triangle( matrix*tri.a, matrix*tri.b, matrix*tri.c ) ;
}

PrecomputedTriangle operator*( const Matrix4f& matrix, const PrecomputedTriangle& tri ) {
  return PrecomputedTriangle( matrix*tri.a, matrix*tri.b, matrix*tri.c ) ;
}

PrecomputedTriangle operator*( const Matrix3f& matrix, const PrecomputedTriangle& tri ) {
  return PrecomputedTriangle( matrix*tri.a, matrix*tri.b, matrix*tri.c ) ;
}

PrecomputedTriangle operator+( const PrecomputedTriangle& tri, const Vector3f& disp ){ 
  return PrecomputedTriangle( tri.a+disp, tri.b+disp, tri.c+disp ) ;
}
PrecomputedTriangle operator+( const Vector3f& disp, const PrecomputedTriangle& tri ){ 
  return PrecomputedTriangle( tri.a+disp, tri.b+disp, tri.c+disp ) ;
}

Triangle::Triangle( const PrecomputedTriangle& ptri ) :
a(ptri.a),b(ptri.b),c(ptri.c), plane( a,b,c )
{
  computeCentroid() ;
}
