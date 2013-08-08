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
  // just dot it to get the min/max along this axis.
  float dotVal = pt.dot( axis ) ;
  if( dotVal < minAlong )  minAlong=dotVal;
  if( dotVal > maxAlong )  maxAlong=dotVal;
}
void SATtest( const Vector3f& axis, const Vector3f* ptSet, int n, float& minAlong, float& maxAlong ) {
  minAlong=HUGE, maxAlong=-HUGE;
  for( int i = 0 ; i < n ; i++ )
  {
    // just dot it to get the min/max along this axis.
    float dotVal = ptSet[i].dot( axis ) ;
    if( dotVal < minAlong )  minAlong=dotVal;
    if( dotVal > maxAlong )  maxAlong=dotVal;
  }
}
