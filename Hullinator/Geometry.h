#ifndef GEOMETRY_H
#define GEOMETRY_H

#include "GLUtil.h"
#include "Intersectable.h"

struct Geometry
{
  // makes a spherical mesh from an icosahedron.
  static void addSphere( vector<Vector3f>& verts, float r )
  {
    //http://en.wikipedia.org/wiki/Icosahedron
    //(0, ±1, ±φ)
    //(±1, ±φ, 0)
    //(±φ, 0, ±1)
    //where φ = (1 + √5) / 2 
    const static float t = ( 1.f + sqrtf( 5.f ) ) / 2.f ;
    const static float L = sqrtf( 2.f / (5.f+sqrtf(5.f)) ) ;
    const static float S = t*L;
    Vector3f v[12]; // 12 base verts
    
    for( int i = 0 ; i < 4; i++ )
      //v[ i ] = Vector( 0, -(i&2), -(i&1)*t ) ; 
      v[ i ] = Vector3f( 0, i&2?-L:L, i&1?-S:S ) * r ;

    for( int i = 4 ; i < 8; i++ )
      //v[ i ] = Vector( -(i&2), -(i&1)*t, 0 ) ; 
      v[ i ] = Vector3f( i&2?-L:L, i&1?-S:S, 0 ) * r ;

    for( int i = 8 ; i < 12; i++ )
      //v[ i ] = Vector( -(i&1)*t, 0, -(i&2) ) ; 
      v[ i ] = Vector3f( i&1?-S:S, 0, i&2?-L:L ) * r ;
      
    // these are the faces.
    addTri( verts, v[0], v[2], v[8] ) ;
    addTri( verts, v[0], v[8], v[4] ) ;
    addTri( verts, v[0], v[4], v[6] ) ;
    addTri( verts, v[0], v[6], v[9] ) ;
    addTri( verts, v[0], v[9], v[2] ) ;

    addTri( verts, v[2], v[7], v[5] ) ;
    addTri( verts, v[2], v[5], v[8] ) ;
    addTri( verts, v[2], v[9], v[7] ) ;
      
    addTri( verts, v[8], v[5], v[10] ) ;
    addTri( verts, v[8], v[10], v[4] ) ;
    
    addTri( verts, v[10], v[5], v[3] ) ;
    addTri( verts, v[10], v[3], v[1] ) ;
    addTri( verts, v[10], v[1], v[4] ) ;
    
    addTri( verts, v[1], v[6], v[4] ) ;
    addTri( verts, v[1], v[3], v[11] ) ;
    addTri( verts, v[1], v[11], v[6] ) ;

    addTri( verts, v[6], v[11], v[9] ) ;

    addTri( verts, v[11], v[3], v[7] ) ;
    addTri( verts, v[11], v[7], v[9] ) ;

    addTri( verts, v[3], v[5], v[7] ) ;
    
  }
  
  template <typename T> static void addSphere( vector<T>& verts, const Vector3f& center, float r, const Vector4f& color )
  {
    const static float t = ( 1.f + sqrtf( 5.f ) ) / 2.f ;
    const static float L = sqrtf( 2.f / (5.f+sqrtf(5.f)) ) ;
    const static float S = t*L;
    
    T v[12]; // 12 base verts
    for( int i = 0 ; i < 4; i++ )
      v[ i ].normal = v[ i ].pos = Vector3f( 0, i&2?-L:L, i&1?-S:S ) ;  // do not scale, normal assigned to save value
    for( int i = 4 ; i < 8; i++ )
      v[ i ].normal = v[ i ].pos = Vector3f( i&2?-L:L, i&1?-S:S, 0 ) ;
    for( int i = 8 ; i < 12; i++ )
      v[ i ].normal = v[ i ].pos = Vector3f( i&1?-S:S, 0, i&2?-L:L ) ;
    for( int i = 0 ; i < 12 ; i++ )
    {
      v[ i ].color = color ;
      v[ i ].pos *= r ; // this is here b/c so the normals are still unit above
      v[ i ].pos += center ; // offset after scaling
    }
    
    // these are the faces.
    addTri( verts, v[0], v[2], v[8] ) ;
    addTri( verts, v[0], v[8], v[4] ) ;
    addTri( verts, v[0], v[4], v[6] ) ;
    addTri( verts, v[0], v[6], v[9] ) ;
    addTri( verts, v[0], v[9], v[2] ) ;

    addTri( verts, v[2], v[7], v[5] ) ;
    addTri( verts, v[2], v[5], v[8] ) ;
    addTri( verts, v[2], v[9], v[7] ) ;
      
    addTri( verts, v[8], v[5], v[10] ) ;
    addTri( verts, v[8], v[10], v[4] ) ;
    
    addTri( verts, v[10], v[5], v[3] ) ;
    addTri( verts, v[10], v[3], v[1] ) ;
    addTri( verts, v[10], v[1], v[4] ) ;
    
    addTri( verts, v[1], v[6], v[4] ) ;
    addTri( verts, v[1], v[3], v[11] ) ;
    addTri( verts, v[1], v[11], v[6] ) ;

    addTri( verts, v[6], v[11], v[9] ) ;

    addTri( verts, v[11], v[3], v[7] ) ;
    addTri( verts, v[11], v[7], v[9] ) ;

    addTri( verts, v[3], v[5], v[7] ) ;
  }
  
  template <typename T> static void addSphereSubdiv( vector<T>& verts, const Vector3f& center, float r, const Vector4f& color,
    int subdivs )
  {
    const static float t = ( 1.f + sqrtf( 5.f ) ) / 2.f ;
    const static float L = sqrtf( 2.f / (5.f+sqrtf(5.f)) ) ;
    const static float S = t*L;
    
    T v[12]; // 12 base verts
    for( int i = 0 ; i < 4; i++ )
      v[ i ].normal = v[ i ].pos = Vector3f( 0, i&2?-L:L, i&1?-S:S ) ;  // do not scale, normal assigned to save value
    for( int i = 4 ; i < 8; i++ )
      v[ i ].normal = v[ i ].pos = Vector3f( i&2?-L:L, i&1?-S:S, 0 ) ;
    for( int i = 8 ; i < 12; i++ )
      v[ i ].normal = v[ i ].pos = Vector3f( i&1?-S:S, 0, i&2?-L:L ) ;
    for( int i = 0 ; i < 12 ; i++ )
    {
      v[ i ].color = color ;
      v[ i ].pos *= r ; // this is here b/c so the normals are still unit above
    }
    
    // these are the faces.
    int n = 0;
    n+=addTriSubdiv( verts, v[0], v[2], v[8], r, subdivs ) ;
    n+=addTriSubdiv( verts, v[0], v[8], v[4], r, subdivs ) ;
    n+=addTriSubdiv( verts, v[0], v[4], v[6], r, subdivs ) ;
    n+=addTriSubdiv( verts, v[0], v[6], v[9], r, subdivs ) ;
    n+=addTriSubdiv( verts, v[0], v[9], v[2], r, subdivs ) ;

    n+=addTriSubdiv( verts, v[2], v[7], v[5], r, subdivs ) ;
    n+=addTriSubdiv( verts, v[2], v[5], v[8], r, subdivs ) ;
    n+=addTriSubdiv( verts, v[2], v[9], v[7], r, subdivs ) ;
      
    n+=addTriSubdiv( verts, v[8], v[5], v[10], r, subdivs ) ;
    n+=addTriSubdiv( verts, v[8], v[10], v[4], r, subdivs ) ;
    
    n+=addTriSubdiv( verts, v[10], v[5], v[3], r, subdivs ) ;
    n+=addTriSubdiv( verts, v[10], v[3], v[1], r, subdivs ) ;
    n+=addTriSubdiv( verts, v[10], v[1], v[4], r, subdivs ) ;
    
    n+=addTriSubdiv( verts, v[1], v[6], v[4], r, subdivs ) ;
    n+=addTriSubdiv( verts, v[1], v[3], v[11], r, subdivs ) ;
    n+=addTriSubdiv( verts, v[1], v[11], v[6], r, subdivs ) ;

    n+=addTriSubdiv( verts, v[6], v[11], v[9], r, subdivs ) ;

    n+=addTriSubdiv( verts, v[11], v[3], v[7], r, subdivs ) ;
    n+=addTriSubdiv( verts, v[11], v[7], v[9], r, subdivs ) ;

    n+=addTriSubdiv( verts, v[3], v[5], v[7], r, subdivs ) ;
    
    for( int i = (int)verts.size() - n*3 ; i < verts.size() ; i++ )
      verts[i].pos += center ;  // offset after scaling
  }


  
  // Used by the wireframe version
  template <typename T>
  static void makeIcosahedronVerts( T* v, float r )
  {
    //http://en.wikipedia.org/wiki/Icosahedron
    //(0, ±1, ±φ)
    //(±1, ±φ, 0)
    //(±φ, 0, ±1)
    //where φ = (1 + √5) / 2
    const static float t = ( 1.f + sqrtf( 5.f ) ) / 2.f ;
    const static float L = sqrtf( 2.f / (5.f+sqrtf(5.f)) ) ;
    const static float S = t*L;
    
    for( int i = 0 ; i < 4; i++ )
      v[ i ].pos = Vector3f( 0, i&2?-L:L, i&1?-S:S ) * r ;
    for( int i = 4 ; i < 8; i++ )
      v[ i ].pos = Vector3f( i&2?-L:L, i&1?-S:S, 0 ) * r ;
    for( int i = 8 ; i < 12; i++ )
      v[ i ].pos = Vector3f( i&1?-S:S, 0, i&2?-L:L ) * r ;
  }
  
  static void makeIcosahedronVerts( Vector3f* v, float r )
  {
    //http://en.wikipedia.org/wiki/Icosahedron
    //(0, ±1, ±φ)
    //(±1, ±φ, 0)
    //(±φ, 0, ±1)
    //where φ = (1 + √5) / 2
    
    // L&S are shortened versions of φ and 1 respectively.
    const static float t = ( 1.f + sqrtf( 5.f ) ) / 2.f ; // ~1.62
    const static float L = sqrtf( 2.f / (5.f+sqrtf(5.f)) ) ; // ~0.53, shorten "1" so √( φ² + L² ) = 1
    const static float S = t*L; // ~0.85, shorter φ
    for( int i = 0 ; i < 4; i++ )
      v[ i ] = Vector3f( 0, i&2?-L:L, i&1?-S:S ) * r ;
    for( int i = 4 ; i < 8; i++ )
      v[ i ] = Vector3f( i&2?-L:L, i&1?-S:S, 0 ) * r ;
    for( int i = 8 ; i < 12; i++ )
      v[ i ] = Vector3f( i&1?-S:S, 0, i&2?-L:L ) * r ;
  }

  static void addSphereLines( vector<Vector3f>& verts, const Vector3f& center, float r )
  {
    Vector3f v[12]; // 12 base verts
    makeIcosahedronVerts( v, r ) ;
    
    for( int i = 0 ; i < 12 ; i++ )
      v[ i ] += center ; // offset after scaling
      
    // draw 30 lines!
    addEdge( verts, v[0], v[2] ) ;  addEdge( verts, v[2], v[8] ) ;
    addEdge( verts, v[0], v[8] ) ;  addEdge( verts, v[4], v[8] ) ;
    addEdge( verts, v[0], v[4] ) ;  addEdge( verts, v[4], v[6] ) ;
    addEdge( verts, v[0], v[6] ) ;  addEdge( verts, v[6], v[9] ) ;
    addEdge( verts, v[0], v[9] ) ;  addEdge( verts, v[2], v[9] ) ;
    
    addEdge( verts, v[2], v[7] ) ;  addEdge( verts, v[7], v[9] ) ;
    addEdge( verts, v[2], v[5] ) ;  addEdge( verts, v[5], v[7] ) ;
    addEdge( verts, v[5], v[8] ) ;  addEdge( verts, v[3], v[7] ) ;
    addEdge( verts, v[3], v[5] ) ;  addEdge( verts, v[3], v[10] ) ;
    addEdge( verts, v[5], v[10] ) ; addEdge( verts, v[8], v[10] ) ;
    
    addEdge( verts, v[4], v[10] ) ; addEdge( verts, v[1], v[10] ) ;
    addEdge( verts, v[1], v[3] ) ;  addEdge( verts, v[1], v[11] ) ;
    addEdge( verts, v[3], v[11] ) ; addEdge( verts, v[7], v[11] ) ;
    addEdge( verts, v[1], v[4] ) ;  addEdge( verts, v[1], v[6] ) ;
    addEdge( verts, v[6], v[11] ) ; addEdge( verts, v[9], v[11] ) ;
  }
  
  template <typename T>
  static void addSphereLines( vector<T>& verts, const Vector3f& center, float r, const Vector4f& color )
  {
    T v[12]; // 12 base verts
    makeIcosahedronVerts( &v[0], r ) ;
    for( int i = 0 ; i < 12 ; i++ )
    {
      v[ i ].color = color ;
      v[ i ].pos += center ;
    }
    
    // draw 30 lines!
    addEdge( verts, v[0], v[2] ) ;  addEdge( verts, v[2], v[8] ) ;
    addEdge( verts, v[0], v[8] ) ;  addEdge( verts, v[4], v[8] ) ;
    addEdge( verts, v[0], v[4] ) ;  addEdge( verts, v[4], v[6] ) ;
    addEdge( verts, v[0], v[6] ) ;  addEdge( verts, v[6], v[9] ) ;
    addEdge( verts, v[0], v[9] ) ;  addEdge( verts, v[2], v[9] ) ;
    
    addEdge( verts, v[2], v[7] ) ;  addEdge( verts, v[7], v[9] ) ;
    addEdge( verts, v[2], v[5] ) ;  addEdge( verts, v[5], v[7] ) ;
    addEdge( verts, v[5], v[8] ) ;  addEdge( verts, v[3], v[7] ) ;
    addEdge( verts, v[3], v[5] ) ;  addEdge( verts, v[3], v[10] ) ;
    addEdge( verts, v[5], v[10] ) ;  addEdge( verts, v[8], v[10] ) ;
    
    addEdge( verts, v[4], v[10] ) ;  addEdge( verts, v[1], v[10] ) ;
    addEdge( verts, v[1], v[3] ) ;  addEdge( verts, v[1], v[11] ) ;
    addEdge( verts, v[3], v[11] ) ;  addEdge( verts, v[7], v[11] ) ;
    addEdge( verts, v[1], v[4] ) ;  addEdge( verts, v[1], v[6] ) ;
    addEdge( verts, v[6], v[11] ) ;  addEdge( verts, v[9], v[11] ) ;
    
    // Cost is 60 VERTS = 60*(3 float pos + 4 float color) = 420 floats
    
    // If you use index buffers, then cost is only
    // 12 verts = 84 floats, + 60 indices (shorts)= eq 84 + 30 float size = 114.
    
  }

  // For the vertex types that do not support normals
  template <typename T>
  static void addSphereNoNormal( vector<T>& verts, float r, const Vector4f& color )
  {
    T v[12];
    makeIcosahedronVerts( v, r ) ;
      
    for( int i = 0 ; i < 12 ; i++ )
      v[ i ].color = color ;
      
    // these are the faces.
    addTri( verts, v[0], v[2], v[8] ) ;
    addTri( verts, v[0], v[8], v[4] ) ;
    addTri( verts, v[0], v[4], v[6] ) ;
    addTri( verts, v[0], v[6], v[9] ) ;
    addTri( verts, v[0], v[9], v[2] ) ;

    addTri( verts, v[2], v[7], v[5] ) ;
    addTri( verts, v[2], v[5], v[8] ) ;
    addTri( verts, v[2], v[9], v[7] ) ;
      
    addTri( verts, v[8], v[5], v[10] ) ;
    addTri( verts, v[8], v[10], v[4] ) ;
    
    addTri( verts, v[10], v[5], v[3] ) ;
    addTri( verts, v[10], v[3], v[1] ) ;
    addTri( verts, v[10], v[1], v[4] ) ;
    
    addTri( verts, v[1], v[6], v[4] ) ;
    addTri( verts, v[1], v[3], v[11] ) ;
    addTri( verts, v[1], v[11], v[6] ) ;

    addTri( verts, v[6], v[11], v[9] ) ;

    addTri( verts, v[11], v[3], v[7] ) ;
    addTri( verts, v[11], v[7], v[9] ) ;

    addTri( verts, v[3], v[5], v[7] ) ;
    
  }

  template <typename T>
  static void addOctahedron( vector<T>& verts, const T& baseVertex, const Matrix4f& mat, const Vector3f& offset )
  {
    float bwx=0.055, bwy=0.055, heado=0, tailo=8;
    float midp = 2.0*tailo/3.0 ;
    T PX,NX,PY,NY,HEAD,TAIL;
    //set up the base properties
    PX=NX=PY=NY=HEAD=TAIL=baseVertex ;
    
    PX.pos = Vector3f( bwx, 0, midp ), NX.pos = Vector3f( -bwx, 0, midp ), 
    PY.pos = Vector3f( 0, bwy, midp ), NY.pos = Vector3f( 0, -bwy, midp ),
    HEAD.pos = Vector3f(0,0,-heado), TAIL.pos = Vector3f( 0,0,tailo ) ;
    
    // The offset from base center, like "shoot from right cannon"
    // means you must offset the octahedron +x a bit, and -y a bit first.
    PX.pos += offset, PY.pos += offset, NX.pos += offset, NY.pos += offset,
    HEAD.pos += offset, TAIL.pos += offset ;
    
    // Orienting the octahedron in space
    PX.pos = mat*PX.pos ;
    PY.pos = mat*PY.pos ;
    NX.pos = mat*NX.pos ;
    NY.pos = mat*NY.pos ;
    HEAD.pos = mat*HEAD.pos ;
    TAIL.pos = mat*TAIL.pos ;

    addTri( verts, TAIL, PX, PY ) ;
    addTri( verts, TAIL, PY, NX ) ;
    addTri( verts, TAIL, NX, NY ) ;
    addTri( verts, TAIL, NY, PX ) ;

    addTri( verts, HEAD, NX, PY ) ;
    addTri( verts, HEAD, PY, PX ) ;
    addTri( verts, HEAD, PX, NY ) ;
    addTri( verts, HEAD, NY, NX ) ;

  }
  
  template <typename T> static void addEdge( vector<T>& verts, const T& a, const T& b ) {
    verts.push_back( a ) ;  verts.push_back( b ) ;
  }
  
  template <typename T> static void addTri( vector<T>& verts, const T& A, const T& B, const T& C ) {
    verts.push_back( A ) ;  verts.push_back( B ) ;  verts.push_back( C ) ;
  }
  
  // You already know the normal
  template <typename T> inline static void addTriWithNormal( vector<T>& verts, const Vector3f& A, const Vector3f& B, const Vector3f& C, const Vector3f& N, const Vector4f& color ) {
    T nA,nB,nC;
    nA.pos=A,nB.pos=B,nC.pos=C;
    nA.normal=nB.normal=nC.normal=N ;
    nA.color=nB.color=nC.color=color;
    addTri( verts, nA,nB,nC ) ;
  }

  // You want me to compute the normal
  template <typename T> inline static void addTriWithNormal( vector<T>& verts, const Vector3f& A, const Vector3f& B, const Vector3f& C, const Vector4f& color ) {
    Vector3f N = Triangle::triNormal( A, B, C ) ;
    addTriWithNormal( verts, A, B, C, N, color ) ;
  }

  // Subdivs the tri you're trying to add n times.  it renormalizes the
  // avg'd positions so they sit on a sphere of same radius
  template <typename T> static int addTriSubdiv( vector<T>& verts, const T& A, const T& B, const T& C, float r, int n ) {
    if( n <= 0 )
    {
      verts.push_back( A ) ;  verts.push_back( B ) ;  verts.push_back( C ) ;
      return 1 ; // 1 tri added.
    }
    else
    {
      // don't actually add the tri until n subdivs have been done
      T AB2 = A.avgWith( B ), BC2=B.avgWith(C),CA2=C.avgWith(A);
      
      AB2.pos.setLen( r ) ;
      BC2.pos.setLen( r ) ;
      CA2.pos.setLen( r ) ;
      
      return addTriSubdiv( verts, A, AB2, CA2, r, n-1 ) +
      addTriSubdiv( verts, AB2, B, BC2, r, n-1 ) +
      addTriSubdiv( verts, CA2, BC2, C, r, n-1 ) +
      addTriSubdiv( verts, CA2, AB2, BC2, r, n-1 ) ; //center tri
    }
  }
  
  // (0,1)
  // D----C (1,1)
  // | __/|
  // |/   |
  // A----B (1,0)
  // (0,0)
  template <typename T> static void addQuad( vector<T>& verts, const T& A, const T& B, const T& C, const T& D )
  {
    addTri( verts, A, B, C ) ;
    addTri( verts, A, C, D ) ;
  }
  
  //static void addTri( vector<VertexPC>& verts, const Vector3f& A, const Vector3f& B, const Vector3f& C, const Vector4f& color ) {
  //  verts.push_back( VertexPC(A,color) ) ;
  //  verts.push_back( VertexPC(B,color) ) ;
  //  verts.push_back( VertexPC(C,color) ) ;
  //}
  
  
  template <typename T> inline static void addQuadWithNormal( vector<T>& verts, const Vector3f& A, const Vector3f& B, const Vector3f& C, const Vector3f& D, const Vector3f& N, const Vector4f& color ) {
    addTriWithNormal( verts, A, B, C, N, color ) ;
    addTriWithNormal( verts, A, C, D, N, color ) ;
  }
  
  template <typename T> inline static void addQuadWithNormal( vector<T>& verts, const Vector3f& A, const Vector3f& B, const Vector3f& C, const Vector3f& D, const Vector4f& color ) {
    addTriWithNormal( verts, A, B, C, color ) ;
    addTriWithNormal( verts, A, C, D, color ) ;
  }

  template <typename T> inline static void addPentagonWithNormal( vector<T>& verts, 
    const Vector3f& A, const Vector3f& B, const Vector3f& C, const Vector3f& D, const Vector3f& E, const Vector4f& color ) {
    addTriWithNormal( verts, A, B, C, color ) ;
    addTriWithNormal( verts, A, C, D, color ) ;
    addTriWithNormal( verts, A, D, E, color ) ;
  }
  
  template <typename T> inline static void addHexagonWithNormal( vector<T>& verts, 
    const Vector3f& A, const Vector3f& B, const Vector3f& C,
    const Vector3f& D, const Vector3f& E, const Vector3f& F, const Vector4f& color ) {
    addTriWithNormal( verts, A, B, C, color ) ;
    addTriWithNormal( verts, C, D, A, color ) ;
    addTriWithNormal( verts, D, F, A, color ) ;
    addTriWithNormal( verts, E, F, D, color ) ;
  }

  
  //static void addQuad( vector<VertexPC>& verts, const Vector3f& A, const Vector3f& B, const Vector3f& C, const Vector3f& D, const Vector4f& color )
  //{
  //  addTri( verts, A, B, C, color ) ;
  //  addTri( verts, A, C, D, color ) ;
  //}
  
  // wind the 2 faces FACING OUT ok?
  template <typename T> static void addTriPrism( vector<T>& verts,
    const Vector3f& A, const Vector3f& B, const Vector3f& C,
    const Vector3f& D, const Vector3f& E, const Vector3f& F,
    const Vector4f& color )
  {
    addTriWithNormal( verts, A, B, C, color ) ;
    addTriWithNormal( verts, D, E, F, color ) ;
    
    addTriWithNormal( verts, A, D, F, color ) ;
    addTriWithNormal( verts, A, F, B, color ) ;
    
    addTriWithNormal( verts, B, F, E, color ) ;
    addTriWithNormal( verts, B, E, C, color ) ;
    
    addTriWithNormal( verts, C, E, D, color ) ;
    addTriWithNormal( verts, C, D, A, color ) ;
  }
  
  template <typename T> static void addTet( vector<T>& verts,
    const Vector3f& A, const Vector3f& B, const Vector3f& C, const Vector3f& D,
    const Vector4f& color )
  {
    addTriWithNormal( verts, A, B, C, color ) ;
    addTriWithNormal( verts, A, D, B, color ) ;
    addTriWithNormal( verts, A, C, D, color ) ;
    addTriWithNormal( verts, B, D, C, color ) ;
  }

  // http://www.ics.uci.edu/~eppstein/projects/tetra/
  // just to see what those 5 tets stuck together in a cube look like
  // YOU CANNOT USE 5 TET PACKING FOR MARCHING TETS.  THE REASON IS
  // THE NEIGHBOURING TETRAHEDRA HAVE DIAGONALS GOING IN __OPPOSITE DIRECTIONS__ WHEN STACKED.
  // THIS IS __NOT OK__ for achieving a space filling packing because then the isosurface
  // punchthrus for adjacent cubes'o'tets will NOT be the same.
  template <typename T> static void gen5Tets( vector<T>& verts, float s, const Vector3f& center )
  {
    s/=2;
    /*
      C----G
     /|   /|
    D-A--H E
    |/   |/
    B----F
    */
    Vector3f A( -s, -s, -s ),  B( -s, -s,  s ),  C( -s,  s, -s ),  D( -s,  s,  s ),
             E(  s, -s, -s ),  F(  s, -s,  s ),  G(  s,  s, -s ),  H(  s,  s,  s ) ;
  
    A+=center,  B+=center,  C+=center,  D+=center,
    E+=center,  F+=center,  G+=center,  H+=center ;

    Geometry::addTet( verts, A, D, C, G, Vector4f( 0,0,1,0.5 ) ) ;
    Geometry::addTet( verts, E, G, F, A, Vector4f( 0,1,0,0.5 ) ) ;
    Geometry::addTet( verts, H, D, F, G, Vector4f( 0.76,0.05,0.18,0.5 ) ) ;
    Geometry::addTet( verts, F, D, A, G, Vector4f( 1,1,0,0.5 ) ) ; // MIDDLE TET
    Geometry::addTet( verts, B, D, A, F, Vector4f( 1,0,0,0.5 ) ) ;
  }

  // http://graphics.cs.ucdavis.edu/~joy/ecs177/other-notes/SixTetrahedra.html
  // This paper "Mysteries in Packing Regular Tetrahedra"
  // http://www.ams.org/notices/201211/rtx121101540p.pdf  (free atm)
  // has a diagram of the packing used here (figure 3)
  // s is the size.
  template <typename T> static void gen6Tets( vector<T>& verts, float s, const Vector3f& center )
  {
    // Notice how ALL the tets use vertex E.
    s/=2;
    /*
      C----G
     /|   /|
    D-A--H E
    |/   |/
    B----F
    */
    // the cube you lay the tets in start in the [-s/2,s/2] cube (s was divided by 2 above)
    // centered AT THE ORIGIN then you translate the entire cube.
    Vector3f A( -s, -s, -s ),  B( -s, -s,  s ),  C( -s,  s, -s ),  D( -s,  s,  s ),
             E(  s, -s, -s ),  F(  s, -s,  s ),  G(  s,  s, -s ),  H(  s,  s,  s ) ;
  
    A+=center,  B+=center,  C+=center,  D+=center,
    E+=center,  F+=center,  G+=center,  H+=center ;

    // LEFT /NX EDGE
    Geometry::addTet( verts, A, B, D, E, Vector4f(   0,   1,   1, 0.5 ) ) ; //Cyan
    Geometry::addTet( verts, A, D, C, E, Vector4f(   0,   0,   1, 0.5 ) ) ; //Blue

    // TOP
    Geometry::addTet( verts, D, G, C, E, Vector4f(   1,   0,   1, 0.5 ) ) ; //Magenta
    Geometry::addTet( verts, D, H, G, E, Vector4f(   0,   1,   0, 0.5 ) ) ; // Green

    // FRONT
    Geometry::addTet( verts, B, F, D, E, Vector4f(   1,   0,   0, 0.5 ) ) ; // red
    Geometry::addTet( verts, F, H, D, E, Vector4f(   1,   1,   0, 0.5 ) ) ; // yellow
  }
  
  
  
  
  
  
  
  
  
  
  template <typename T> static void addQuadGenUVNormal( vector<T>& verts,
    T A, T B, T C, T D,
    const Vector2f& minTex, const Vector2f& maxTex )
  {
    // (0,1)
    // D----C (1,1)
    // | __/|
    // |/   |
    // A----B (1,0)
    // (0,0)
    
    A.tex = minTex ;
    B.tex = Vector2f( maxTex.x, minTex.y );
    C.tex = maxTex ;
    D.tex = Vector2f( minTex.x, maxTex.y );
    
    // Sets the normal 
    setFaceNormalQuad( A, B, C, D, hasNormalTrait<T>() ) ;
    
    addQuad( verts, A, B, C, D ) ;
  }
  
  template <typename T> static void setFaceNormalQuad( T& A, T& B, T& C, T& D, std::true_type const & )
  {
    // Find the normal
    Vector3f triNorm = Triangle::triNormal( A.pos, B.pos, C.pos ) ;
    A.normal = B.normal = C.normal = D.normal = triNorm ;
  }
  
  template <typename T> static void setFaceNormalQuad( T& A, T& B, T& C, T& D, std::false_type const & )
  {
    // does nothing, the vertex has no normal
  }
  
  //!! This is not the best function.
  template <typename T> static void addSquare( vector<T>& verts, const Vector3f& min, const Vector3f& max, const Vector4f& color,
    const Vector2f& minTex, const Vector2f& maxTex )
  {
    T A,B,C,D ;

    // D----C
    // | __/|
    // |/   |
    // A----B 

    A.pos = Vector3f( min.x, min.y, max.z ) ; // use max's z all the time //min ;
    B.pos = Vector3f( max.x, min.y, max.z ) ; // use max's z all the time
    C.pos = max ;
    D.pos = Vector3f( min.x, max.y, max.z ) ;

    A.color=B.color=C.color=D.color= color ;
    
    addQuadGenUVNormal( verts, A, B, C, D, minTex, maxTex ) ;
  }
  
  
  
  
  
  
  
  
  
  //////////////////////
  // SOLID CUBES
  static void addCubeCenteredFacingIn( vector<VertexPNC>& cmVerts, const Vector3f& center, float s, const Vector4f& color )
  {
    s /= 2.f;
    /*
      C----G
     /|   /|
    D-A--H E
    |/   |/
    B----F

       D--H
       |  |
    D--C--G--H--D
    |  |  |  |  |
    B--A--E--F--B
       |  |
       B--F
    */
    Vector3f A( -s, -s, -s ),  B( -s, -s,  s ),  C( -s,  s, -s ),  D( -s,  s,  s ),
             E(  s, -s, -s ),  F(  s, -s,  s ),  G(  s,  s, -s ),  H(  s,  s,  s ) ;
    A+=center ;  B+=center ;  C+=center ;  D+=center ;
    E+=center ;  F+=center ;  G+=center ;  H+=center ;
    // right face PX
    Vector3f norm( -1,0,0 ) ;
    Geometry::addQuad( cmVerts, VertexPNC( E,norm,color ), VertexPNC( F,norm,color ), VertexPNC( H,norm,color ), VertexPNC( G,norm,color ) ) ; // IN
    
    // left NX
    norm.x= 1;
    Geometry::addQuad( cmVerts, VertexPNC( B,norm,color ), VertexPNC( A,norm,color ), VertexPNC( C,norm,color ), VertexPNC( D,norm,color ) ) ; // IN

    // top face PY
    norm.x=0,norm.y=-1;
    Geometry::addQuad( cmVerts, VertexPNC( C,norm,color ), VertexPNC( G,norm,color ), VertexPNC( H,norm,color ), VertexPNC( D,norm,color ) ) ; // IN
    
    // bottom NY
    norm.y= 1;
    Geometry::addQuad( cmVerts, VertexPNC( B,norm,color ), VertexPNC( F,norm,color ), VertexPNC( E,norm,color ), VertexPNC( A,norm,color ) ) ; // IN
    
    // back face PZ
    norm.y=0,norm.z=-1;
    Geometry::addQuad( cmVerts, VertexPNC( A,norm,color ), VertexPNC( E,norm,color ), VertexPNC( G,norm,color ), VertexPNC( C,norm,color ) ) ; // IN
    
    // front face NZ
    norm.z= 1;
    Geometry::addQuad( cmVerts, VertexPNC( F,norm,color ), VertexPNC( B,norm,color ), VertexPNC( D,norm,color ), VertexPNC( H,norm,color ) ) ;
  }

  // SKYCUBE AND WHATNOT
  static void addCubeCenteredFacingOut( vector<VertexPNC>& cmVerts, const Vector3f& center, float s, const Vector4f& color )
  {
    s /= 2.f;
    /*
      C----G
     /|   /|
    D-A--H E
    |/   |/
    B----F
    */
    Vector3f A( -s, -s, -s ),  B( -s, -s,  s ),  C( -s,  s, -s ),  D( -s,  s,  s ),
             E(  s, -s, -s ),  F(  s, -s,  s ),  G(  s,  s, -s ),  H(  s,  s,  s ) ;
    A+=center ;  B+=center ;  C+=center ;  D+=center ;  E+=center ;  F+=center ;  G+=center ;  H+=center ;
    // right face PX
    Vector3f norm( 1,0,0 ) ;
    Geometry::addQuad( cmVerts, VertexPNC( E,norm,color ), VertexPNC( G,norm,color ), VertexPNC( H,norm,color ), VertexPNC( F,norm,color ) ) ; // IN
    
    // left NX
    norm.x=-1;
    Geometry::addQuad( cmVerts, VertexPNC( B,norm,color ), VertexPNC( D,norm,color ), VertexPNC( C,norm,color ), VertexPNC( A,norm,color ) ) ; // IN

    // top face PY
    norm.x=0,norm.y=1;
    Geometry::addQuad( cmVerts, VertexPNC( C,norm,color ), VertexPNC( D,norm,color ), VertexPNC( H,norm,color ), VertexPNC( G,norm,color ) ) ; // IN
    
    // bottom NY
    norm.y=-1;
    Geometry::addQuad( cmVerts, VertexPNC( B,norm,color ), VertexPNC( A,norm,color ), VertexPNC( E,norm,color ), VertexPNC( F,norm,color ) ) ; // IN
    
    // back face PZ
    norm.y=0,norm.z=1;
    Geometry::addQuad( cmVerts, VertexPNC( A,norm,color ), VertexPNC( C,norm,color ), VertexPNC( G,norm,color ), VertexPNC( E,norm,color ) ) ; // IN
    
    // front face NZ
    norm.z=-1;
    Geometry::addQuad( cmVerts, VertexPNC( F,norm,color ), VertexPNC( H,norm,color ), VertexPNC( D,norm,color ), VertexPNC( B,norm,color ) ) ;
  }
  
  // by vertex.  not really a cube, its really an 8 point convex polyhedron, but i still call it a cube.
  static void addCubeFacingOut( vector<VertexPNC>& cmVerts,
    const Vector3f& A, const Vector3f& B, const Vector3f& C, const Vector3f& D,
    const Vector3f& E, const Vector3f& F, const Vector3f& G, const Vector3f& H, const Vector4f& color )
  {
    /*
      C----G
     /|   /|
    D-A--H E
    |/   |/
    B----F
    */
    // right face PX
    Vector3f norm( 1,0,0 ) ;
    Geometry::addQuad( cmVerts, VertexPNC( E,norm,color ), VertexPNC( G,norm,color ), VertexPNC( H,norm,color ), VertexPNC( F,norm,color ) ) ; // IN
    
    // left NX
    norm.x=-1;
    Geometry::addQuad( cmVerts, VertexPNC( B,norm,color ), VertexPNC( D,norm,color ), VertexPNC( C,norm,color ), VertexPNC( A,norm,color ) ) ; // IN

    // top face PY
    norm.x=0,norm.y=1;
    Geometry::addQuad( cmVerts, VertexPNC( C,norm,color ), VertexPNC( D,norm,color ), VertexPNC( H,norm,color ), VertexPNC( G,norm,color ) ) ; // IN
    
    // bottom NY
    norm.y=-1;
    Geometry::addQuad( cmVerts, VertexPNC( B,norm,color ), VertexPNC( A,norm,color ), VertexPNC( E,norm,color ), VertexPNC( F,norm,color ) ) ; // IN
    
    // back face PZ
    norm.y=0,norm.z=1;
    Geometry::addQuad( cmVerts, VertexPNC( A,norm,color ), VertexPNC( C,norm,color ), VertexPNC( G,norm,color ), VertexPNC( E,norm,color ) ) ; // IN
    
    // front face NZ
    norm.z=-1;
    Geometry::addQuad( cmVerts, VertexPNC( F,norm,color ), VertexPNC( H,norm,color ), VertexPNC( D,norm,color ), VertexPNC( B,norm,color ) ) ;
  }
  
  // TEXTURED
  template <typename T> static void addCubeTextured( vector<T>& verts, const Vector3f& min, const Vector3f& max, const Vector4f& color,
    const Vector2f& minTex, const Vector2f& maxTex,
    bool facingOut )
  {
    T A,B,C,D,E,F,G,H;
    
    //Vector3f A( min ),B( min.x, min.y, max.z ),C( min.x, max.y, min.z ),D( min.x, max.y, max.z ),
    //  E( max.x, min.y, min.z ),F( max.x, min.y, max.z ),G( max.x, max.y, min.z ),H( max );
    A.pos = min ;
    B.pos = Vector3f( min.x, min.y, max.z ) ;
    C.pos = Vector3f( min.x, max.y, min.z ) ;
    D.pos = Vector3f( min.x, max.y, max.z ) ;
    
    E.pos = Vector3f( max.x, min.y, min.z ) ;
    F.pos = Vector3f( max.x, min.y, max.z ) ;
    G.pos = Vector3f( max.x, max.y, min.z ) ;
    H.pos = max ;
    
    A.color=B.color=C.color=D.color=E.color=F.color=G.color=H.color= color ;
    
    //       y
    //     ^
    //     |
    //    C----G
    //   /|   /|
    //  D-A--H E  -> x
    //  |/   |/ 
    //  B----F  
    //  /
    // z
    
    // 6 faces
    if( facingOut )
    {
      // CCW out
      addQuadGenUVNormal( verts, F, E, G, H, minTex, maxTex ) ; //PX
      addQuadGenUVNormal( verts, A, B, D, C, minTex, maxTex ) ; //NX
      addQuadGenUVNormal( verts, G, C, D, H, minTex, maxTex ) ; //PY
      addQuadGenUVNormal( verts, F, B, A, E, minTex, maxTex ) ; //NY
      addQuadGenUVNormal( verts, B, F, H, D, minTex, maxTex ) ; //PZ
      addQuadGenUVNormal( verts, E, A, C, G, minTex, maxTex ) ; //NZ
    }
    else
    {
      // normals face IN, CCW in.
      addQuadGenUVNormal( verts, E, F, H, G, minTex, maxTex ) ; //PX
      addQuadGenUVNormal( verts, B, A, C, D, minTex, maxTex ) ; //NX
      addQuadGenUVNormal( verts, C, G, H, D, minTex, maxTex ) ; //PY
      addQuadGenUVNormal( verts, B, F, E, A, minTex, maxTex ) ; //NY
      addQuadGenUVNormal( verts, F, B, D, H, minTex, maxTex ) ; //PZ
      addQuadGenUVNormal( verts, A, E, G, C, minTex, maxTex ) ; //NZ
    }
  }
  
  template <typename T> static void addCube( vector<T>& verts, const Vector3f& min, const Vector3f& max, const Vector4f& color, bool facingOut )
  {
    Vector3f A( min ),B( min.x, min.y, max.z ),C( min.x, max.y, min.z ),D( min.x, max.y, max.z ),
             E( max.x, min.y, min.z ),F( max.x, min.y, max.z ),G( max.x, max.y, min.z ),H( max );
    
    // I assume you have normals.  if you don't what good is adding a solid cube?  it will be too hard to see.
    
    //       y
    //     ^
    //     |
    //    C----G
    //   /|   /|
    //  D-A--H E  -> x
    //  |/   |/ 
    //  B----F  
    //  /
    // z
    
    // 6 faces
    if( facingOut )
    {
      // CCW out
      addQuadWithNormal( verts, F, E, G, H, Vector3f( 1, 0, 0), color ) ; //PX
      addQuadWithNormal( verts, A, B, D, C, Vector3f(-1, 0, 0), color ) ; //NX
      addQuadWithNormal( verts, G, C, D, H, Vector3f( 0, 1, 0), color ) ; //PY
      addQuadWithNormal( verts, F, B, A, E, Vector3f( 0,-1, 0), color ) ; //NY
      addQuadWithNormal( verts, B, F, H, D, Vector3f( 0, 0, 1), color ) ; //PZ
      addQuadWithNormal( verts, E, A, C, G, Vector3f( 0, 0,-1), color ) ; //NZ
    }
    else
    {
      // normals face IN, CCW in.
      addQuadWithNormal( verts, E, F, H, G, Vector3f( 1, 0, 0), color ) ; //PX
      addQuadWithNormal( verts, B, A, C, D, Vector3f(-1, 0, 0), color ) ; //NX
      addQuadWithNormal( verts, C, G, H, D, Vector3f( 0, 1, 0), color ) ; //PY
      addQuadWithNormal( verts, B, F, E, A, Vector3f( 0,-1, 0), color ) ; //NY
      addQuadWithNormal( verts, F, B, D, H, Vector3f( 0, 0, 1), color ) ; //PZ
      addQuadWithNormal( verts, A, E, G, C, Vector3f( 0, 0,-1), color ) ; //NZ
    }
  }
  template <typename T> static void addCubeFacingOut( vector<T>& verts, const AABB& aabb, const Vector4f& color ) {
    addCubeFacingOut( verts, aabb.min, aabb.max, color ) ;
  }
  template <typename T> static void addCubeFacingIn( vector<T>& verts, const AABB& aabb, const Vector4f& color ) {
    addCubeFacingIn( verts, aabb.min, aabb.max, color ) ;
  }
  
  
  template <typename T> static void addCubeLine( vector<T>& verts, const Vector3f& min, const Vector3f& max, const Vector4f& color )
  {
    T A,B,C,D,E,F,G,H;
    
    A.pos = min ;
    B.pos = Vector3f( min.x, min.y, max.z ) ;
    C.pos = Vector3f( min.x, max.y, min.z ) ;
    D.pos = Vector3f( min.x, max.y, max.z ) ;
    
    E.pos = Vector3f( max.x, min.y, min.z ) ;
    F.pos = Vector3f( max.x, min.y, max.z ) ;
    G.pos = Vector3f( max.x, max.y, min.z ) ;
    H.pos = max ;
    
    A.color=B.color=C.color=D.color=E.color=F.color=G.color=H.color= color ;
    
    //   C----G
    //  /|   /|
    // D-A--H E
    // |/   |/
    // B----F
    
    // left nx
    addEdge( verts, A, B ) ;  addEdge( verts, B, D ) ;  addEdge( verts, D, C ) ;  addEdge( verts, C, A ) ;
    
    // right px
    addEdge( verts, E, F ) ;  addEdge( verts, F, H ) ;  addEdge( verts, H, G ) ;  addEdge( verts, G, E ) ;
    
    addEdge( verts, D, H ) ;  addEdge( verts, B, F ) ;  addEdge( verts, A, E ) ;  addEdge( verts, C, G ) ;

  }
  
  template <typename T> static void addCubeLine( vector<T>& verts, const AABB& aabb, const Vector4f& color )
  {
    addCubeLine( verts, aabb.min, aabb.max, color ) ;
  }
  
  template <typename T> static void addCubeLine( vector<T>& verts, const Vector3f& a, const Vector3f& b, 
    const Vector3f& c, const Vector3f& d, const Vector3f& e, const Vector3f& f, 
    const Vector3f& g, const Vector3f& h, const Vector4f& color )
  {
    T A,B,C,D,E,F,G,H;
    
    A.pos = a ;
    B.pos = b ;
    C.pos = c ;
    D.pos = d ;
    
    E.pos = e ;
    F.pos = f ;
    G.pos = g ;
    H.pos = h ;
    
    A.color=B.color=C.color=D.color=E.color=F.color=G.color=H.color= color ;
    
    addEdge( verts, A, B ) ;  addEdge( verts, B, C ) ;  addEdge( verts, C, D ) ;  addEdge( verts, D, A ) ;
    addEdge( verts, E, F ) ;  addEdge( verts, F, G ) ;  addEdge( verts, G, H ) ;  addEdge( verts, H, E ) ;
    addEdge( verts, A, E ) ;  addEdge( verts, B, F ) ;  addEdge( verts, C, G ) ;  addEdge( verts, D, H ) ;
  }
  
  template <typename T> static void drawFrustum( vector<T>& verts, const Frustum& f, const Vector4f& color )
  {
    // looking INTO the near plane:
    // farB  farA
    //   \    / 
    //    b--a
    //    |  |
    //    c--d
    //   /    \  
    // farC  farD
    addCubeLine( verts, f.a, f.b, f.c, f.d, f.farA, f.farB, f.farC, f.farD, color ) ;
  }

  
  // related to addCube, this function CHANGES the texcoords in verts
  // to being minTex/maxTex as specified.
  // This is needed because as you choose an itembox to create,
  // the skin has to be selected from the main texture
  template <typename T> static void cubeChangeTexcoords( vector<T>& verts,
    const Vector2f& minTex, const Vector2f& maxTex )
  {
    // now use the same visitation order (ABC, ACD)
    for( int i = 0 ; i < verts.size() ; i+=6 )
    {
      // (0,1)
      // D----C (1,1)
      // | __/|
      // |/   |
      // A----B (1,0)
      // (0,0)
      verts[i].tex = minTex ; //A
      verts[i+1].tex = Vector2f(maxTex.x,minTex.y) ;
      verts[i+2].tex = maxTex ;
      
      verts[i+3].tex = minTex ;
      verts[i+4].tex = maxTex ;
      verts[i+5].tex = Vector2f(minTex.x,maxTex.y) ;
    }
  }
  
  #if 0
  static void addCubeFacingIn( vector<VertexPC>& cmVerts, const Vector3f& center, float s, const Vector4f& color )
  {
    s /= 2.f;
    /*

      C----G
     /|   /|
    D-A--H E
    |/   |/
    B----F

       D--H
       |  |
    D--C--G--H--D
    |  |  |  |  |
    B--A--E--F--B
       |  |
       B--F

    */
    Vector3f A( -s, -s, -s ),  B( -s, -s,  s ),  C( -s,  s, -s ),  D( -s,  s,  s ),
      E(  s, -s, -s ),  F(  s, -s,  s ),  G(  s,  s, -s ),  H(  s,  s,  s ) ;
    A+=center ;  B+=center ;  C+=center ;  D+=center ;
    E+=center ;  F+=center ;  G+=center ;  H+=center ;
    // right face PX
    Geometry::addQuad( cmVerts, VertexPC( E,color ), VertexPC( F,color ), VertexPC( H,color ), VertexPC( G,color ) ) ; // IN
    
    // left NX
    Geometry::addQuad( cmVerts, VertexPC( B,color ), VertexPC( A,color ), VertexPC( C,color ), VertexPC( D,color ) ) ; // IN

    // top face PY
    Geometry::addQuad( cmVerts, VertexPC( C,color ), VertexPC( G,color ), VertexPC( H,color ), VertexPC( D,color ) ) ; // IN
    
    // bottom NY
    Geometry::addQuad( cmVerts, VertexPC( B,color ), VertexPC( F,color ), VertexPC( E,color ), VertexPC( A,color ) ) ; // IN
    
    // back face PZ
    Geometry::addQuad( cmVerts, VertexPC( A,color ), VertexPC( E,color ), VertexPC( G,color ), VertexPC( C,color ) ) ; // IN
    
    // front face NZ
    Geometry::addQuad( cmVerts, VertexPC( F,color ), VertexPC( B,color ), VertexPC( D,color ), VertexPC( H,color ) ) ;
  }
  #endif
  
  
  
} ;



#endif
