//
//  main.cpp
//  IK
//
//  Created by Hyun Joon Shin on 2021/06/09.
//

#include <iostream>
#include <JGL/JGL_Window.hpp>
#include "AnimView.hpp"

#include "body.hpp"




Body body;

glm::vec3 oldPt3;
glm::vec2 oldPt2;
glm::vec3 pickPt;
glm::vec3 targetPt;
float oldD;
int picked = -1;
AnimView* animView;




void init() {
    body.clear();
    body.add(-1,glm::vec3(0,10,0));
    body.add(0,glm::vec3(0,10,0));
    body.add(1,glm::vec3(0,10,0));
    body.add(2,glm::vec3(0,10,0));
    body.add(3,glm::vec3(0,10,0));
    body.add(4,glm::vec3(0,10,0));
//    body.links[0].rotate(glm::exp(glm::quat(0, 0, 0, 0.1)));
//    body.links[1].rotate(glm::exp(glm::quat(0, 0, 0, -0.1)));
    body.updatePos();
}

void frame(float dt) {
//    body.links[0].rotate(glm::exp(glm::quat(0, 0, 0, 0.1)));
//    body.links[1].rotate(glm::exp(glm::quat(0, 0, 0, -0.1)));
    body.updatePos();
}

void render() {
    body.render();
    if( picked>=0 ) {
        drawSphere( targetPt, 1.5, glm::vec4(1,1,0,.1) );
    }
    drawQuad(glm::vec3(0,0,0), glm::vec3(0,1,0), glm::vec2(1000,1000));
}

bool press( const glm::vec2& pt2, const glm::vec3& pt3, float d ) {
    oldPt3 = pt3;
    oldPt2 = pt2;
    oldD = d;
    for( int i=0; i<body.links.size(); i++)
    if( length( pt3 - body.links[i].getPos() ) < 1.5 ) {
        picked = i;
        targetPt = pickPt = body.links[i].getPos();
        return true;
    }
    return false;
}

bool drag( const glm::vec2& pt2, const glm::vec3& pt3, float d ) {
    if( picked>=0 ) {
        targetPt = pickPt + animView->unproject( pt2, oldD ) - oldPt3;
        body.solveIK( picked, targetPt );
        return true;
    }
    return false;
}

bool release( const glm::vec2& pt2, const glm::vec3& pt3, float d ) {
    picked = -1;
    return true;
}

int main(int argc, const char * argv[]) {
    JGL::Window* window = new JGL::Window(800, 600, "simulation");
    window->alignment(JGL::ALIGN_ALL);
    animView = new AnimView(0, 0, 800, 600);
    animView->renderFunction = render;
    animView->frameFunction = frame;
    animView->initFunction = init;
    animView->pressFunc = press;
    animView->dragFunc = drag;
    animView->releaseFunc = release;

    init();
    window->show();
    JGL::_JGL::run();
    return 0;
}



















//#include <Eigen/core>
//#include <Eigen/dense>


/*
 struct Link {
 int parent;
 glm::vec3 l;
 glm::quat q = glm::quat(1,0,0,0);
 glm::quat gq = glm::quat(1,0,0,0);
 glm::vec3 p = glm::vec3(0);;
 glm::vec3 parentP = glm::vec3(0);
 glm::quat parentQ = glm::quat(1,0,0,0);
 
 void updatePos(const glm::vec3& pp, const glm::quat& pq ) {
 parentQ = pq;
 parentP = pp;
 gq = parentQ*q;
 p = glm::rotate( gq, l ) + parentP;
 }
 void render() const {
 drawSphere( p, 1 );
 drawSphere( parentP, 1 );
 drawCylinder( p, parentP, 0.5 );
 }
 };
 
 struct Body {
 glm::vec3 gp = glm::vec3(0);
 glm::quat gq = glm::quat(1,0,0,0);
 std::vector<Link> links;
 
 void updatePos() {
 for( auto& link: links ) {
 int parent = link.parent;
 if( parent<0 ) link.updatePos(gp, gq);
 else link.updatePos(links[parent].p, links[parent].gq);
 }
 }
 void clear() {
 links.clear();
 gp = glm::vec3(0);
 gq = glm::quat(1,0,0,0);
 }
 void add( int parent, glm::vec3 l ) {
 links.push_back({parent,l});
 }
 void render() const {
 for( const auto& l: links ) l.render();
 }
 std::vector<int> getAncestors( int target ) {
 int p = target;
 std::vector<int> list;
 while( p>=0 ) {
 list.push_back(p);
 p = links[p].parent;
 }
 return list;
 }
 void solveIK(int target, const glm::vec3& targetPt ) {
 using namespace Eigen;
 using namespace glm;
 
 auto ancestors = getAncestors( target );
 const float dt = 0.01;
 vec3 omega[] = {{1,0,0},{0,1,0},{0,0,1}};
 MatrixXf m = MatrixXf(3,ancestors.size()*3);
 VectorXf b = VectorXf(3);
 for( int iter = 0; iter<100; iter++ ) {
 
 vec3 ep = links[target].p;
 for( int i=0; i<ancestors.size(); i++ ) {
 vec3 p = ep-links[ancestors[i]].parentP;
 for( int j=0; j<3; j++ ) {
 vec3 v = cross( omega[j], p );
 m.block(0,i*3+j,3,1) << v.x, v.y, v.z;
 }
 }
 vec3 d = targetPt - ep;
 b << d.x, d.y, d.z;
 
 auto s = m.bdcSvd(ComputeThinU|ComputeThinV);
 s.setThreshold(0.003);
 auto x = s.solve(b);
 for( int i=0; i<ancestors.size(); i++ ) {
 for( int j=0; j<3; j++ )
 links[ancestors[i]].q = exp(quat(0,omega[j]*x(i*3+j)*dt)) * links[ancestors[i]].q;
 }
 updatePos();
 }
 }
 };
 */

