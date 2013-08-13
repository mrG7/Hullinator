//
//  Hullinator
//
//  Created by William Sherif on 8/4/13.
//
/*

  https://github.com/superwills/Hullinator
  Convex hull creation and intersection (and rays & triangles)
  version 1.0.0, Aug 7, 2013 649p

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

#ifdef _WIN32
#include <stdlib.h> // MUST BE BEFORE GLUT ON WINDOWS
#include <gl/glut.h>

#else
#include <GLUT/glut.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <Carbon/Carbon.h>

// I call them by their windows names, on mac gets renamed
#define VK_RIGHT kVK_RightArrow
#define VK_LEFT kVK_LeftArrow
#define VK_UP kVK_UpArrow
#define VK_DOWN kVK_DownArrow
#endif
#include "GLUtil.h"
#include "StdWilUtil.h"
#include "Vectorf.h"
#include "Geometry.h"
#include "Hull.h"
#include <vector>
using namespace std;

int w=768, h=768 ;// window width and height.
#include "Message.h"

static float mx, my, sbd=100.f,ptSize=1.f,lineWidth=1.f ;
bool showOriginalPoints=0, axisLinesOn=1;
vector<Vector3f> pointCloud1,pointCloud2 ;
Vector4f lightPos0, lightPos1, lightPos2, lightPos3 ;
Frustum frustum(w,h) ;
int pointsPerCloud = 35 ;

const char* ModeName[] = {
  "Hull-hull",
  "Hull-tri",
  "Tri-tri",
  "Sphere-hull",
  "Sphere-tri",
  "Sphere-AABB",
  "Hull-AABB",
  "Plane-plane-plane"
} ;
enum Mode{
  HullHull, HullTri, TriTri, 
  SphereHull, SphereTri, SphereAABB, HullAABB,
    
  PlanePlanePlane // LEAVE PlanePlanePlane LAST, its used to determine #modes
} ;
int mode = 0 ;

Hull hull1,hull2 ;
Triangle tri1(0,0,0), tri2(0,0,0),
planeTri1(0,0,0),planeTri2(0,0,0),planeTri3(0,0,0) ; // huge tris used as planes
Sphere sphere1 ;

void regenHulls()
{
  // Oh look at the beautiful syntax. hull1 is a Hull( around point cloud 1 ).
  hull1 = Hull( pointCloud1 ), hull2 = Hull( pointCloud2 ) ;
}

void newPointClouds(){
  pointCloud1.clear();
  pointCloud2.clear();
  Vector3f c1=Vector3f::random( 0, 15 ), c2=Vector3f::random(-15,0) ;
  for( int i = 0 ; i < pointsPerCloud ; i++ )
  {
    pointCloud1.push_back( c1+ Vector3f::random(-10,10) ) ;
    pointCloud2.push_back( c2+ Vector3f::random(-10,10) ) ;
  }
  regenHulls() ;
}

void shiftPointClouds() {
  for( int i = 0 ; i < pointCloud1.size() ; i++ ) {
    pointCloud1[i] += Vector3f::random(-.2,.2) ;
  }
  for( int i = 0 ; i < pointCloud2.size() ; i++ ) {
    pointCloud2[i] += Vector3f::random(-.2,.2) ;
  }
  regenHulls() ;
}

void init() // Called before main loop to set up the program
{
  newPointClouds() ;
  
  ptSize = 16.f;
  glPointSize( ptSize ) ;
  
  msg( "mode", "\"Hullo\", I am Hullinator." ) ;
  msg( "instr1", "(1) to switch modes. (2) for wireframe (toggle). (h) to for help." ) ;
  
}

void help()
{
  msg( "mode", ModeName[mode], Gray ) ;
  switch( mode )
  {
    case Mode::HullHull:
      msg( "instr1", "(m) makes new point clouds.  +/- to change # pts per cloud." ) ;
      msg( "instr2", "holding (r) jiggles the clouds. (e) shows the original points that made up the hull." ) ;
      break;
    case Mode::HullTri:
      msg( "instr1", "left/right arrows to spin tri. Also (m), (+/-)" ) ;
      break;
    case Mode::TriTri:
      msg( "instr1", "left/right arrows to spin tris. (2) for wireframe." ) ;
      break;
    case Mode::SphereHull:
      msg( "instr1", "left/right arrows to grow/shrink sphere. up/down to move in/out. Also (m), (+/-)" ) ;
      break ;
    case Mode::SphereTri:
      msg( "instr1", "left/right arrows to grow/shrink sphere" ) ;
      break ;
    case Mode::SphereAABB:
      msg( "instr1", "left/right arrows to grow/shrink sphere and also move box. (r) jiggles, (m) makes new, (+/-)" ) ;
      break ;
    case Mode::HullAABB:
      msg( "instr1", "left/right/up/down arrows to move aabb. (r) jiggles, also (m), (+/-)" ) ;
      break ;
    case Mode::PlanePlanePlane:
      msg( "instr1", "Intn is yellow pt.  CTRL+Click to fire rays at the planes." ) ;
      break;  
  }
  
  static bool firstHelp=1;
  if( firstHelp )
  {
    msg( "instr3", "CTRL+CLICK to fire rays. (c) to clear those rays." ) ;
    firstHelp=0;
  }
}

#ifdef __APPLE__
KeyMap keyStates ;

bool IS_KEYDOWN( uint16_t vKey )
{
  // http://stackoverflow.com/questions/11466294/getting-keyboard-state-using-getkeys-function
  uint8_t index = vKey / 32 ;
  uint8_t shift = vKey % 32 ;
  return keyStates[index].bigEndianValue & (1 << shift) ;
}
#else
#define IS_KEYDOWN( c ) (GetAsyncKeyState( c ) & 0x8000)
#endif

void hullHullTest()
{
  // This is a unit test, so make sure it works both ways
  if( hull1.intersectsHull( hull2 ) )  hull1.drawDebug( Red ) ;
  else  hull1.drawDebug( Vector4f(0,0,1,0.75) ) ;

  if( hull2.intersectsHull( hull1 ) )  hull2.drawDebug( Purple ) ;
  else  hull2.drawDebug( Vector4f(0,1,0,0.75) ) ;
  if( showOriginalPoints )
  {
    hull1.drawDebugOriginalPts() ;
    hull2.drawDebugOriginalPts() ;
  }
}

void hullTriTest()
{
  static float ang = 0.f;
  if( IS_KEYDOWN( VK_RIGHT ) )
    ang += 0.001f ;
  if( IS_KEYDOWN( VK_LEFT ) )
    ang -= 0.001f ;
  
  Matrix3f rot = Matrix3f::rotationY( ang ) ; // * Matrix3f::rotationX( M_PI- ang ) ;
  tri1 = Triangle( rot*Vector3f( -20,0,5 ), rot*Vector3f( 20,0,5 ), rot*Vector3f( 0,20,-5 ) ) ;
  
  // Testing the circumsphere
  //Sphere circ = tri1.findCircumsphere() ;
  //addDebugSphereSolid( circ.c, circ.r, Green ) ;
  
  Vector3f penetration ;
  if( hull1.intersectsTri( tri1, penetration ) ) {
    addDebugLine( tri1.centroid, tri1.centroid + penetration, Red ) ;
    addDebugTriSolid( tri1, Red ) ;
    hull1.drawDebug( Vector4f(1,0,1,0.75) ) ;
    
    // Resolve the interpenetration with these ghosts
    Triangle t2( tri1.a + penetration, tri1.b + penetration, tri1.c + penetration ) ;
    addDebugTriSolid( t2, Vector4f( 0,0,1,0.5f ) ) ;
    
    hull1.drawDebug( -penetration, Vector4f(0,0,1,0.5) ) ;
  }
  else {
    addDebugTriSolid( tri1, Blue ) ;
    hull1.drawDebug( Vector4f(0,0,1,0.75) ) ;
  }
}

void triTriTest()
{
  static float ang = 0.f;
  if( IS_KEYDOWN( VK_RIGHT ) )
    ang += 0.001f ;
  if( IS_KEYDOWN( VK_LEFT ) )
    ang -= 0.001f ;
  
  Matrix3f rot = Matrix3f::rotationY( ang ) ;
  tri1 = Triangle( rot*Vector3f( -20,0,5 ), rot*Vector3f( 20,0,5 ), rot*Vector3f( 0,20,-5 ) ) ;
  
  rot = Matrix3f::rotationX( ang ) ;
  //rot = Matrix3f::rotation( rot*Vector3f(1,0,0), ang ) ;
  tri2 = Triangle( rot*Vector3f( -20,10,20 ), rot*Vector3f( 20,10,20 ), rot*Vector3f( -10,10,-10 ) ) ;
  
  Ray ray ;
  if( tri1.intersectsTri( tri2,ray ) )
  {
    addDebugTriSolid( tri1, Purple ) ;
    addDebugTriSolid( tri2, Yellow ) ;
    addDebugRay( ray, Red ) ;
  }
  else
  {
    addDebugTriSolid( tri1, Blue ) ;
    addDebugTriSolid( tri2, Green ) ;
  }
}

void sphereHullTest()
{
  // move a pt around,
  static float ang=0.f;
  static float sphereDist=20.f;
  if( IS_KEYDOWN( VK_UP ) )
    sphereDist += 0.01f ;
  if( IS_KEYDOWN( VK_DOWN ) )
    sphereDist -= 0.01f ;
  
  Vector3f pt = Matrix3f::rotationY( ang+=0.0001f ) * Vector3f( sphereDist,sphereDist*sin(ang/3.f),sphereDist ) ;
  
  static float r = 3.f ;
  if( IS_KEYDOWN( VK_RIGHT ) )
    r += 0.01f ;
  if( IS_KEYDOWN( VK_LEFT ) )
    r -= 0.01f ;
  
  sphere1 = Sphere( pt, r ) ;
  Vector3f closestPtOnHull ;
  if( hull1.intersectsSphere( sphere1 ) ) {
    hull1.drawDebug( Red ) ;
    //addDebugLine( pt, closestPtOnHull, Red ) ;
    addDebugSphereSolid( pt, r, Purple ) ;
  }
  else {
    hull1.drawDebug( Vector4f(0,0,1,0.75) ) ;
    //addDebugLine( pt, closestPtOnHull, Yellow ) ;
    addDebugSphereSolid( pt, r, Blue ) ;
  }
}

void sphereTriTest()
{
  // move a pt around,
  static float ang=0.f;
  Vector3f pt = Matrix3f::rotationY( ang+=0.0001f ) * Vector3f( 20,20*sin(ang/3.f),20 ) ;
  
  Matrix3f rot = Matrix3f::rotationY( ang*10.f ) ; // * Matrix3f::rotationX( M_PI- ang ) ;
  tri1 = Triangle( rot*Vector3f( -20,0,5 ), rot*Vector3f( 20,0,5 ), rot*Vector3f( 0,20,-5 ) ) ;
  
  static float r = 3.f ;
  if( IS_KEYDOWN( VK_RIGHT ) )
    r += 0.01f ;
  if( IS_KEYDOWN( VK_LEFT ) )
    r -= 0.01f ;
  
  sphere1 = Sphere( pt,r ) ;
  
  Vector3f closestPtOnHull ;
  if( tri1.intersectsSphere( sphere1 ) ) {
    addDebugTriSolid( tri1, Red ) ;
    addDebugSphereSolid( pt, r, Purple ) ;
  }
  else {
    addDebugTriSolid( tri1, Green ) ;
    addDebugSphereSolid( pt, r, Blue ) ;
  }
}

void sphereAABBTest()
{
  float move=0.01f;
  Vector3f offsets ;
  if( IS_KEYDOWN( VK_UP ) )
    offsets.y += move;
  if( IS_KEYDOWN( VK_DOWN ) )
    offsets.y -= move;
  if( IS_KEYDOWN( VK_RIGHT ) )
    offsets.x += move;
  if( IS_KEYDOWN( VK_LEFT ) )
    offsets.x -= move;

  for( int i = 0 ; i < pointCloud2.size() ; i++ )
    pointCloud2[i] += offsets ;
  
  hull2 = Hull( pointCloud2 ) ; // you have ot regen the cloud, and so the aabb.
  static float ang=0.f;
  Vector3f pt = Matrix3f::rotationY( ang+=0.0001f ) * Vector3f( 20,20*sin(ang/3.f),20 ) ;
  
  Matrix3f rot = Matrix3f::rotationY( ang*10.f ) ; // * Matrix3f::rotationX( M_PI- ang ) ;
  tri1 = Triangle( rot*Vector3f( -20,0,5 ), rot*Vector3f( 20,0,5 ), rot*Vector3f( 0,20,-5 ) ) ;
  
  static float r = 3.f ;
  if( IS_KEYDOWN( VK_RIGHT ) )
    r += 0.01f ;
  if( IS_KEYDOWN( VK_LEFT ) )
    r -= 0.01f ;
  
  sphere1 = Sphere( pt,r ) ;
  // use the 2nd hull's aabb
  if( hull2.aabb.intersectsSphere( sphere1 ) )
  {
    addDebugSphereSolid( sphere1.c, sphere1.r, Purple ) ;
    hull2.aabb.drawDebugSolid( Red ) ;
  }
  else
  {
    addDebugSphereSolid( sphere1.c, sphere1.r, Green ) ;
    hull2.aabb.drawDebugSolid( Blue ) ;
  }
}

void hullAABBTest()
{
  float move=0.01f;
  Vector3f offsets ;
  if( IS_KEYDOWN( VK_UP ) )
    offsets.y += move;
  if( IS_KEYDOWN( VK_DOWN ) )
    offsets.y -= move;
  if( IS_KEYDOWN( VK_RIGHT ) )
    offsets.x += move;
  if( IS_KEYDOWN( VK_LEFT ) )
    offsets.x -= move;

  for( int i = 0 ; i < pointCloud2.size() ; i++ )
    pointCloud2[i] += offsets ;
  
  hull2 = Hull( pointCloud2 ) ; // you have ot regen the cloud, and so the aabb.
  
  // use the 2nd hull's aabb
  if( hull1.intersectsAABB( hull2.aabb ) )
  {
    hull1.drawDebug( Vector4f(1,0,1,0.75) ) ;
    hull2.aabb.drawDebugSolid( Red ) ;
  }
  else
  {
    hull1.drawDebug( Vector4f(0,0,1,0.75) ) ;
    hull2.aabb.drawDebugSolid( Blue ) ;
  }
}



void planePlanePlaneTest()
{
  static float a = 0.f;
  a += 0.001f;
  
  // very large triangles (planes essentially)
  planeTri1 = Triangle( Vector3f( -500, -500, 0* sinf(a) ), Vector3f( 500, -500, 25 * sinf(a) ), Vector3f( 0, 500, -30* sinf(a) ) ) ;
  addDebugTriSolid( planeTri1, Vector4f( 1,0,0,0.5f ) ) ;
  
  planeTri2 = Triangle( Vector3f( -40, -500, 500 ), Vector3f( 50, -500, -500 ), Vector3f( 5, 500, -10 ) ) ;
  addDebugTriSolid( planeTri2, Vector4f( 0,0,1,0.5f ) ) ;

  planeTri3 = Triangle( Vector3f( -500, 4, 300 ), Vector3f( 500, 7, 500 ), Vector3f( 0, 12, -500 ) ) ;
  addDebugTriSolid( planeTri3, Vector4f( 0,1,0,0.5f ) ) ;
  
  // Test plane-plane-plane intn
  Vector3f pt = Plane::getIntersection( planeTri1.plane, planeTri2.plane, planeTri3.plane ) ;
  addDebugPoint( pt, Yellow ) ; // the startpos of the ray is the intn pt of the 3 planes.

  // the direction vector i want to run a line along is along the intersection line
  // of 2 of the planes.
  Vector3f dir = planeTri1.plane.normal.cross( planeTri2.plane.normal ).normalize() ;
  addDebugLine( pt + dir*-500.f, pt + dir*500.f, Yellow ) ; // draw ray really long along intn line
  
}

void draw()
{
  // FIRST I'LL DO THE PROCESSING HERE
  #ifdef __APPLE__
  GetKeys(keyStates) ;
  #endif

  // Which test mode are you running?
  switch( mode )
  {
  case Mode::HullHull:
    hullHullTest() ;
    break;
  case Mode::HullTri:
    hullTriTest() ;
    break ;
  case Mode::TriTri:
    triTriTest() ;
    break ;
  case Mode::SphereHull:
    sphereHullTest() ;
    break; 
  case Mode::SphereTri:
    sphereTriTest() ;
    break ;
  case Mode::SphereAABB:
    sphereAABBTest() ;
    break ;
  case Mode::HullAABB:
    hullAABBTest() ;
    break ;
  case PlanePlanePlane:
    planePlanePlaneTest() ;
    break ;
  default:
    error( "INVALID MODE %d", mode ) ;
    break ;
  }
  
  
  
  
  glEnable( GL_DEPTH_TEST ) ;
  glClearColor( 0.1, 0.1, 0.1, 0.1 ) ;
  glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT ) ;

  glEnable( GL_BLEND ) ;
  glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA ) ;
  
  //glCullFace( GL_BACK ) ;//backface back face culling
  //glEnable( GL_CULL_FACE ) ;
  
  glEnable( GL_COLOR_MATERIAL ) ;
  
  glViewport( 0, 0, w, h ) ;
  glMatrixMode( GL_PROJECTION ) ;
  glLoadIdentity();
  gluPerspective( 45.0, 1.0, 0.5, 1000.0 ) ;
  frustum.persp( RADIANS(45.f), 1.f, 0.5f, 1000.f ) ;
  
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glTranslatef( 0, 0, -sbd ) ;
  glRotatef( my, 1, 0, 0 ) ;
  glRotatef( mx, 0, 1, 0 ) ;
  
  // Build the same matriix backwards with inverted values to get the phsyical frustum.
  Matrix4f mvm ;
  mvm = mvm * Matrix3f::rotationY( RADIANS(-mx) ) ;
  mvm = mvm * Matrix3f::rotationX( RADIANS(-my) ) ;
  mvm = mvm * Matrix4f::Translation( Vector3f(0,0,sbd) ) ;
  
  frustum.orient( mvm ) ;
  //frustum.drawDebug() ;
    
  glLineWidth( 1.f ) ;
  if( axisLinesOn )
    drawAxisLines() ;
  glLineWidth( lineWidth ) ;
  
  // LIGHTS, 
  glEnable( GL_LIGHTING ) ;
  glEnable( GL_LIGHT0 ) ;
  glEnable( GL_LIGHT1 ) ;
  //glEnable( GL_LIGHT2 ) ;
  //glEnable( GL_LIGHT3 ) ;
  
  float ld = 50.f ;
  lightPos0 = Vector4f(  ld,  ld,  ld, 1 ) ;
  lightPos1 = Vector4f( -ld,  ld, -ld, 1 ) ;
  lightPos2 = Vector4f(   0,  ld,   0, 1 ) ;
  lightPos3 = Vector4f( -ld,   0,   0, 1 ) ;

  glLightfv( GL_LIGHT0, GL_POSITION, &lightPos0.x ) ;
  glLightfv( GL_LIGHT1, GL_POSITION, &lightPos1.x ) ;
  glLightfv( GL_LIGHT2, GL_POSITION, &lightPos2.x ) ;
  glLightfv( GL_LIGHT3, GL_POSITION, &lightPos3.x ) ;
  
  float white[4] = {1,1,1,1};
  glLightfv( GL_LIGHT1, GL_DIFFUSE, white ) ;
  glLightfv( GL_LIGHT2, GL_DIFFUSE, white ) ;
  glLightfv( GL_LIGHT3, GL_DIFFUSE, white ) ;
  Vector4f spec(1,1,1,75) ;
  //glMaterialfv( GL_FRONT_AND_BACK, GL_SPECULAR, &spec.x ) ;
  //glMaterialf( GL_FRONT_AND_BACK, GL_SHININESS, spec.w ) ;
  
  drawDebug() ;
  msgDraw() ;
  
  glutSwapBuffers();
}

// Called every time a window is resized to resize the projection matrix
void resize( int newWidth, int newHeight )
{
  w = newWidth ;
  h = newHeight ;
}

static int lastX=0, lastY=0 ;
static int mmode=0;
void mouseMotion( int x, int y )
{
  int dx = x-lastX, dy=y-lastY ;
  
  // LMB
  if( mmode == GLUT_LEFT_BUTTON )
  {
    mx += dx, my += dy ;
  }
  else if( mmode == GLUT_RIGHT_BUTTON )
  {
    // dolly
    sbd +=0.1*(-dx+dy) ;
    clamp( sbd, 10, 1000 ) ;
  }
  
  lastX=x,lastY=y;
}

bool testHitHull( const Hull& hull, const Ray& ray )
{
  float t1,t2;
  bool hitHull = hull.intersectsRay( ray, t1, t2 ) ;
  if( hitHull )
  {
    //printf( "hit t1=%f, t2=%f\n", t1, t2 ) ;
    addPermDebugPoint( ray.at( t1 ), Yellow ) ;
    addPermDebugPoint( ray.at( t2 ), Red ) ;
  }
  return hitHull ;
}

bool testHitTri( const Triangle& tri, const Ray& ray )
{
  Vector3f triPt ;
  bool hitTri = tri.intersectsRay( ray, triPt ) ;
  if( hitTri )
    addPermDebugPoint( triPt, Red ) ;
  return hitTri ;
}

bool testHitAABB( const AABB& aabb, const Ray& ray )
{
  Vector3f pt ;
  bool hit = aabb.intersectsRay( ray, pt ) ;
  if( hit )
    addPermDebugPoint( pt, Red ) ;
  return hit ;
}

bool testHitSphere( const Sphere& sphere, const Ray& ray )
{
  Vector3f pt1, pt2 ;
  bool hit = ray.intersectsSphere( sphere, pt1, pt2 ) ;
  if( hit ) {
    addPermDebugPoint( pt1, Yellow ) ;
    addPermDebugPoint( pt2, Red ) ;
  }
  return hit ;
}

bool testHitPlane( const Plane& plane, const Ray& ray )
{
  Vector3f pt ;
  bool hitPlane = plane.intersectsRay( ray, pt ) ;
  if( hitPlane )
    addPermDebugPoint( pt, Red ) ;
  return hitPlane ;
}

// 0 for LMB, 2 for RMB
// state==0 is down, 1 is up
// (glutGetModifiers() & GLUT_ACTIVE_CTRL) to test if ctrl is down
// (glutGetModifiers() & GLUT_ACTIVE_ALT), (glutGetModifiers() & GLUT_ACTIVE_SHIFT)
void mouse( int button, int state, int x, int y )
{
  //printf( "%d %d %d %d\n", button,state,x,h-y ) ;
  
  lastX = x ;
  lastY = y ;
  mmode=button;
  
  if( !button && !state && (glutGetModifiers() & GLUT_ACTIVE_CTRL) )
  {
    Ray ray = frustum.getRay( h-y, x ) ; //invert the y
    
    bool rayHits=0;
    // Which test mode are you running?
    switch( mode )
    {
    case Mode::HullHull:
      // both hull1 and hull2 are visible
      rayHits |= testHitHull( hull1, ray ) ;
      rayHits |= testHitHull( hull2, ray ) ;
      break;
      
    case Mode::HullTri:
      rayHits |= testHitHull( hull1, ray ) ;
      rayHits |= testHitTri( tri1, ray ) ;
      break ;
      
    case Mode::TriTri:
      rayHits |= testHitTri( tri1, ray ) ;
      rayHits |= testHitTri( tri2, ray ) ;
      break ;
      
    case Mode::SphereHull:
      rayHits |= testHitSphere( sphere1, ray ) ;
      rayHits |= testHitHull( hull1, ray ) ;
      break ;
      
    case Mode::SphereTri:
      rayHits |= testHitSphere( sphere1, ray ) ;
      rayHits |= testHitTri( tri1, ray ) ;
      break ;
    
    case Mode::SphereAABB:
      rayHits |= testHitSphere( sphere1, ray ) ;
      rayHits |= testHitAABB( hull2.aabb, ray ) ;
      break ;
      
    case Mode::HullAABB:
      rayHits |= testHitHull( hull1, ray ) ;
      rayHits |= testHitAABB( hull2.aabb, ray ) ;
      break ;
      
    case Mode::PlanePlanePlane:
      rayHits |= testHitPlane( planeTri1.plane, ray ) ;
      rayHits |= testHitPlane( planeTri2.plane, ray ) ;
      rayHits |= testHitPlane( planeTri3.plane, ray ) ;
      break ;
    
    }

    if( rayHits )
      addPermDebugRay( ray, Red, Red ) ;
    else
      addPermDebugRay( ray, Yellow, Yellow ) ;
  }
  
}

void keyboard( unsigned char key, int x, int y )
{
  switch( key )
  {
  case '1':
    cycleFlag( mode, 0, Mode::PlanePlanePlane ) ;
    help() ;
    break ;
  case '!':
    decycleFlag( mode, 0, Mode::PlanePlanePlane ) ;
    help() ;
    break ;
    
  case '2':
    {
    int pMode[2];
    glGetIntegerv( GL_POLYGON_MODE, pMode ) ;
    if( pMode[0] == GL_FILL ) {
      glPolygonMode( GL_FRONT_AND_BACK, GL_LINE ) ;
      msg( "lw", makeString( "(l/L)inewidth (%.0f)", lineWidth ) );
    }
    else  glPolygonMode( GL_FRONT_AND_BACK, GL_FILL ) ;
    }
    break ;
    
  case '7':
    axisLinesOn = !axisLinesOn ;
    break ;
    
  case '=':
  case '+':
    pointsPerCloud++;
    msg( "ptspercloud", makeString( "%d pts", pointsPerCloud ), w-100, 40, 4.f ) ;
    goto NEWPOINTCLOUDS ; // goto programming revival.
    break ;
    
  case '-':
  case '_':
    pointsPerCloud--;
    if( pointsPerCloud < 0 ) pointsPerCloud = 0 ;
    msg( "ptspercloud", makeString( "%d pts", pointsPerCloud ), w-100, 40, 4.f ) ;    
    goto NEWPOINTCLOUDS ; // goto programming revival.
    break; 
  
  case 'c':
  CLEAR:
    debugPointsPerm.clear() ;
    debugLinesPerm.clear() ;
    debugTrisPerm.clear();
    break ;
    
  case 'e':
    showOriginalPoints = !showOriginalPoints;
    if( showOriginalPoints )
      msg( "lw", makeString( "(p/P)ointsize (%.0f)", ptSize ) ) ;
    break ;
    
  case 'h':
    help() ;
    break ;
  
  case 'l':
    lineWidth++;
    msg( "lw", makeString( "(l/L)inewidth (%.0f)", lineWidth ) );
    glLineWidth( lineWidth ) ;
    break ;
  case 'L':
    lineWidth--;
    if( lineWidth < 1.f ) lineWidth = 1.f ;
    msg( "lw", makeString( "(l/L)inewidth (%.0f)", lineWidth ) );
    glLineWidth( lineWidth ) ;
    break ;
  
  case 'm':
  NEWPOINTCLOUDS:
    newPointClouds() ;
    goto CLEAR ; // goto programming revival.
    break ;
  
  case 'p':
    ptSize++;
    msg( "lw", makeString( "(p/P)ointsize (%.0f)", ptSize ) ) ;
    glPointSize( ptSize ) ;
    break ;
  case 'P':
    ptSize--;
    if( ptSize < 1.f )  ptSize=1.f ;
    msg( "lw", makeString( "(p/P)ointsize (%.0f)", ptSize ) ) ;
    glPointSize( ptSize ) ;
    break ;
  
  case 'r':
    shiftPointClouds() ;
    goto CLEAR ; // goto programming revival.
    break ;

  case 27:
    exit(0);
    break;
    
  default:
    break;
  }
}

int main( int argc, char **argv )
{
  glutInit( &argc, argv ) ; // Initializes glut

  glutInitDisplayMode( GLUT_DOUBLE | GLUT_DEPTH | GLUT_RGBA ) ;
  glutInitWindowSize( w, h ) ;
  glutInitWindowPosition( 0, 0 ) ;
  glutCreateWindow( "The Hullinator" ) ;
  glutReshapeFunc( resize ) ;
  glutDisplayFunc( draw ) ;
  glutIdleFunc( draw ) ;
  
  glutMotionFunc( mouseMotion ) ;
  glutMouseFunc( mouse ) ;
  
  glutKeyboardFunc( keyboard ) ;
  init();

  glutMainLoop();
  return 0;
}












