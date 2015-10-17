/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

/*
 * Base code for CS 251 Software Systems Lab
 * Department of Computer Science and Engineering, IIT Bombay
 *
 */


#include "cs251_base.hpp"


 //#include "callbacks.hpp"
// #include "callbacks.cpp"
#include "render.hpp"

#ifdef __APPLE__
	#include <GLUT/glut.h>
#else
	#include "GL/freeglut.h"
#endif
#ifndef __APPLE__
#include "GL/glui.h"
#else
#include "GL/glui.h"
#endif

#include <cstring>
using namespace std;

#include "dominos.hpp"


  /**  The is the constructor
   * This is the documentation block for the constructor.
   */

  namespace cs251
{
  //dominos_t::dominos_t()
  bool x1=true,y1=true;b2Body* sbody;b2Vec2 vel;float force,totalRotation,bodyAngle;b2Body* b22;
   float x0,y0;int tww,thh;float32 view_zoom1 = 1.0f;cs251::settings_t settings1;b2Vec2 p;//b2Vec2 extents1;
    b2MouseJointDef xd;int st;b2Vec2 er;float gh;b2Body* body5;b2Body* body55;b2Body* spherebody1;b2Body* spherebody2;
    b2Body* bs1;b2Body* bs2;b2Vec2* vertices;
    //return vel;
    dominos_t::dominos_t()
    {
    //Ground
    /*! \var b1
     * \brief pointer to the body ground
     */
     //keyboard_up_cb(unsigned char key, int x, int y);

    {
      //b2Body* b1;
      b2EdgeShape shape;
      shape.Set(b2Vec2(-90.0f, 0.0f), b2Vec2(900.0f, 0.0f));
      b2BodyDef bd;
      b22 = m_world->CreateBody(&bd);
      b22->CreateFixture(&shape, 0.0f);
    }
    {
      b2Body* b1;
      b2EdgeShape shape;
      shape.Set(b2Vec2(-90.0f, 0.0f), b2Vec2(-90.0f, 180.0f));
      b2BodyDef bd;
      b1 = m_world->CreateBody(&bd);
      b1->CreateFixture(&shape, 0.0f);
    }
    {
      b2Body* b1;
      b2EdgeShape shape;
      shape.Set(b2Vec2(900.0f, 180.0f), b2Vec2(900.0f, 0.0f));
      b2BodyDef bd;
      b1 = m_world->CreateBody(&bd);
      b1->CreateFixture(&shape, 0.0f);
    }
    {
      b2Body* b1;
      b2EdgeShape shape;
      shape.Set(b2Vec2(-90.0f, 180.0f), b2Vec2(900.0f, 180.0f));
      b2BodyDef bd;
      b1 = m_world->CreateBody(&bd);
      b1->CreateFixture(&shape, 0.0f);
    }

    //The see-saw system at the bottom
    {
      //The triangle wedge
      b2Body* tbody;
      b2PolygonShape poly;
      b2Vec2 vertices[3];
      vertices[0].Set(-1,0);
      vertices[1].Set(1,0);
      vertices[2].Set(0,1.5);
      poly.Set(vertices, 3);
      b2FixtureDef wedgefd;
      wedgefd.shape = &poly;
      wedgefd.density = 10.0f;
      wedgefd.friction = 0.0f;
      wedgefd.restitution = 0.0f;
      b2BodyDef wedgebd;
      wedgebd.position.Set(-30.0f, 40.0f);
      tbody = m_world->CreateBody(&wedgebd);
      tbody->CreateFixture(&wedgefd);

      //The plank on top of the wedge
      b2PolygonShape shape;
      shape.SetAsBox(20.0f, 0.1f);
      b2BodyDef bd2;
      bd2.position.Set(-33.0f, 41.5f);
      bd2.type = b2_dynamicBody;
      b2Body* pbody = m_world->CreateBody(&bd2);
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 1.f;
      fd2->shape = new b2PolygonShape;
      fd2->shape = &shape;
      pbody->CreateFixture(fd2);

      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      anchor.Set(-30.0f, 41.5f);
      jd.Initialize(tbody, pbody, anchor);
      m_world->CreateJoint(&jd);

      //The light box on the right side of the see-saw
      /*b2PolygonShape shape2;
      shape2.SetAsBox(2.0f, 2.0f);
      b2BodyDef bd3;
      bd3.position.Set(40.0f, 2.0f);
      bd3.type = b2_dynamicBody;
      b2Body* body3 = m_world->CreateBody(&bd3);
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 0.01f;
      fd3->shape = new b2PolygonShape;
      fd3->shape = &shape2;
      body3->CreateFixture(fd3);*/
    }


    {
      b2Body* b1;
      b2EdgeShape shape;
      shape.Set(b2Vec2(-90.0f, 40.0f), b2Vec2(10.0f, 40.0f));
      b2BodyDef bd;
      b1 = m_world->CreateBody(&bd);
      b1->CreateFixture(&shape, 0.0f);
    }

    {
      b2Body* b1;
      b2EdgeShape shape;
      shape.Set(b2Vec2(37.0f, 40.0f), b2Vec2(50.0f, 40.0f));
      b2BodyDef bd;
      b1 = m_world->CreateBody(&bd);
      b1->CreateFixture(&shape, 0.0f);
    }
    {
      b2Body* b1;
      b2EdgeShape shape;
      shape.Set(b2Vec2(50.0f, 40.0f), b2Vec2(80.0f, 50.0f));
      b2BodyDef bd;
      b1 = m_world->CreateBody(&bd);
      b1->CreateFixture(&shape, 0.0f);
    }
    {
      b2Body* b1;
      b2EdgeShape shape;
      shape.Set(b2Vec2(80.0f, 40.0f), b2Vec2(110.0f, 20.0f));
      b2BodyDef bd;
      b1 = m_world->CreateBody(&bd);
      b1->CreateFixture(&shape, 0.0f);
    }
    {
      b2Body* b1;
      b2EdgeShape shape;
      shape.Set(b2Vec2(110.0f, 20.0f), b2Vec2(210.0f, 20.0f));
      b2BodyDef bd;
      b1 = m_world->CreateBody(&bd);
      b1->CreateFixture(&shape, 0.0f);
    }
    {
      b2Body* b1;
      b2EdgeShape shape;
      shape.Set(b2Vec2(150.0f, 40.0f), b2Vec2(210.0f, 40.0f));
      b2BodyDef bd;
      b1 = m_world->CreateBody(&bd);
      b1->CreateFixture(&shape, 0.0f);
    }
    {
      b2Body* b1;
      b2EdgeShape shape;
      shape.Set(b2Vec2(280.0f, 40.0f), b2Vec2(230.0f, 40.0f));
      b2BodyDef bd;
      b1 = m_world->CreateBody(&bd);
      b1->CreateFixture(&shape, 0.0f);
    }
    {
      b2Body* b1;
      b2EdgeShape shape;
      shape.Set(b2Vec2(280.0f, 40.0f), b2Vec2(280.0f, 90.0f));
      b2BodyDef bd;
      b1 = m_world->CreateBody(&bd);
      b1->CreateFixture(&shape, 0.0f);
    }
    {
    b2PolygonShape shape;
      shape.SetAsBox(0.8f, 0.8f);

      b2FixtureDef fd;
      fd.shape = &shape;
      fd.density = 2.0f;
      fd.friction = 0.1f;



	  b2BodyDef bd1;
	  bd1.type = b2_dynamicBody;
	  for(int i=0;i<4;i++){
	  bd1.position.Set(255, 40.8+1.6*i);
      b2Body* sqbody51 = m_world->CreateBody(&bd1);
	  sqbody51->CreateFixture(&fd);
	  }
    }





    {
      b2BodyDef *bdz = new b2BodyDef;
      //bdz->type = b2_dynamicBody;
      bdz->position.Set(245,15);
      bdz->fixedRotation = true;

      //The open box
      b2FixtureDef *fd1z = new b2FixtureDef;
      fd1z->density = 10.0;
      fd1z->friction = 0.5;
      fd1z->restitution = 0.f;
      fd1z->shape = new b2PolygonShape;
      b2PolygonShape bs1;
      bs1.SetAsBox(10,0.2, b2Vec2(0.0f,-10.0f), 0);
      fd1z->shape = &bs1;
      b2FixtureDef *fd2z = new b2FixtureDef;
      fd2z->density = 100.0;
      fd2z->friction = 0.5;
      fd2z->restitution = 0.f;
      fd2z->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(0.2,2.5, b2Vec2(10.0f,-7.50f), 0);
      fd2z->shape = &bs2;
      b2FixtureDef *fd3z = new b2FixtureDef;
      fd3z->density = 100.0;
      fd3z->friction = 0.5;
      fd3z->restitution = 0.f;
      fd3z->shape = new b2PolygonShape;
      b2PolygonShape bs3;
      bs3.SetAsBox(0.2,2.5, b2Vec2(-10.0f,-7.50f), 0);
      fd3z->shape = &bs3;


      b2Body* box1 = m_world->CreateBody(bdz);
      box1->CreateFixture(fd1z);
      box1->CreateFixture(fd2z);
      box1->CreateFixture(fd3z);

      //The bar
      bdz->position.Set(220,30);
      bs1.SetAsBox(10,0.2, b2Vec2(0.0f,-10.0f), 0);
      fd1z->shape = &bs1;
      bs2.SetAsBox(0.2,0.2, b2Vec2(-10.0f,-9.80f), 0);
      fd2z->shape = &bs2;
      bs3.SetAsBox(0.2,0.2, b2Vec2(10.0f,-9.80f), 0);
      fd3z->shape = &bs3;

      fd1z->density = 34.0;
      b2Body* box2 = m_world->CreateBody(bdz);
      box2->CreateFixture(fd1z);
      box2->CreateFixture(fd2z);
      box2->CreateFixture(fd3z);

      // The pulley joint
      b2PulleyJointDef* myjoint = new b2PulleyJointDef();
      b2Vec2 worldAnchorOnBody2(220, 20); // Anchor point on body 1 in world axis
      b2Vec2 worldAnchorOnBody1(245, 15); // Anchor point on body 2 in world axis
      b2Vec2 worldAnchorGround2(220, 45); // Anchor point for ground 1 in world axis
      b2Vec2 worldAnchorGround1(245, 45); // Anchor point for ground 2 in world axis
      float32 ratio = 1.0f; // Define ratio
      myjoint->Initialize(box1, box2, worldAnchorGround1, worldAnchorGround2, box1->GetWorldCenter(), box2->GetWorldCenter(), ratio);
      m_world->CreateJoint(myjoint);



      b2CircleShape circle;
      circle.m_radius = 3.0f;


      b2FixtureDef ballfd;

      ballfd.shape = &circle;
      ballfd.density = 0.01f;
      ballfd.friction = 1.00f;
      ballfd.restitution = 0.5f;
      b2BodyDef ballbd;
      for (int i = 0; i < 3; ++i)
	{

	  ballbd.type = b2_dynamicBody;
	  ballbd.position.Set(214.0f + i*6.0f, 23.1f);
	  b2Body* spherebody1a = m_world->CreateBody(&ballbd);
	  spherebody1a->CreateFixture(&ballfd);
    }
      ballbd.position.Set(217.f , 28.1f);
	  b2Body* spherebody1a = m_world->CreateBody(&ballbd);
	  spherebody1a->CreateFixture(&ballfd);
	  ballbd.position.Set(223.f , 28.1f);
	  spherebody1a = m_world->CreateBody(&ballbd);
	  spherebody1a->CreateFixture(&ballfd);
	  ballbd.position.Set(220.f , 33.1f);
	  spherebody1a = m_world->CreateBody(&ballbd);
	  spherebody1a->CreateFixture(&ballfd);
    }








    {
    b2PolygonShape shape;
      shape.SetAsBox(18.8f, 0.8f);

      b2FixtureDef fd;
      fd.shape = &shape;
      fd.density = 20.0f;
      fd.friction = 0.1f;
	  b2BodyDef bd1;
	  //bd1.type = b2_dynamicBody;
	  bd1.position.Set(110.0f+0.3f*(69.4), 41.0-0.3*(4.5));
      b2Body* sqbody5 = m_world->CreateBody(&bd1);
	  sqbody5->CreateFixture(&fd);

	  b2CircleShape cicle;
      cicle.m_radius = 0.95;
      b2FixtureDef ballfdc;
      ballfdc.shape = &cicle;
      ballfdc.density = 1.0f;
      ballfdc.friction = 0.70f;
      ballfdc.restitution = 0.5f;

      b2BodyDef ballbdc;
	  ballbdc.type = b2_dynamicBody;
	  ballbdc.position.Set(110.0f+0.3f*(69.4)-19.6, 41.0-0.3*(4.0));
	  b2Body* cspherebody1 = m_world->CreateBody(&ballbdc);
	  cspherebody1->CreateFixture(&ballfdc);
	  cspherebody1->SetAngularVelocity(5);
	  ballbdc.position.Set(110.0f+0.3f*(69.4)+19.6, 41.0-0.3*(4.0));
	  b2Body* cspherebody2 = m_world->CreateBody(&ballbdc);
	  cspherebody2->CreateFixture(&ballfdc);
      cspherebody2->SetAngularVelocity(5);
    }
    {
    b2PolygonShape shape;
      shape.SetAsBox(20.8f, 1.2f);

      b2FixtureDef fd;
      fd.shape = &shape;
      fd.density = 20.0f;
      fd.friction = 0.1f;



	  b2BodyDef bd1;
	  //bd1.type = b2_dynamicBody;
	  bd1.position.Set(110.0f+0.3f*(69.4), 41.0-0.3*(4.5)-3.1);
      b2Body* sqbody5 = m_world->CreateBody(&bd1);
	  sqbody5->CreateFixture(&fd);

    }
    /*{
      b2Body* b1;
      b2EdgeShape shape;
      shape.Set(b2Vec2(110.0f, 40.0f), b2Vec2(150.0f, 40.0f));
      b2BodyDef bd;
      b1 = m_world->CreateBody(&bd);
      b1->CreateFixture(&shape, 0.0f);



    b2PolygonShape shape;
      shape.SetAsBox(20.0f, 0.1f);

      b2FixtureDef fd;
      fd.shape = &shape;
      fd.density = 20.0f;
      fd.friction = 0.1f;
    }*/

    /*{
     b2PolygonShape shape12;
      shape12.SetAsBox(20.0f, 0.05f);

      b2FixtureDef fd12;
      fd12.shape = &shape12;
      fd12.density = 20.0f;
      fd12.friction = 0.1f;



	  b2BodyDef bd12;
	  //bd12.type = b2_dynamicBody;
	  bd12.position.Set(130.0f , 40.0f);
	  b2Body* body51 = m_world->CreateBody(&bd12);
	  body51->CreateFixture(&fd12);


	  b2CircleShape circles;
      circles.m_radius = 0.2;


      b2FixtureDef ballfd12;

      ballfd12.shape = &circles;
      ballfd12.density = 10.0f;
      ballfd12.friction = 1.0f;
      ballfd12.restitution = 0.3f;
	  for(int i=0;i<80;i++){
	  b2BodyDef ballbd12;
	  ballbd12.type = b2_dynamicBody;
	  ballbd12.position.Set(110.2f + i*0.5f, 40.0f);
	  b2Body* spherebody12 = m_world->CreateBody(&ballbd12);
	  spherebody12->CreateFixture(&ballfd12);
	  b2RevoluteJointDef jd12;
      b2Vec2 anchor12;
      anchor12.Set(110.2f + i*0.5f, 40.0f);
      jd12.Initialize(spherebody12, body51, anchor12);
        spherebody12->SetAngularVelocity(20);
      m_world->CreateJoint(&jd12);
	  }
	  	  }*/


    //body and fixture defs are common to all chain links
{b2BodyDef bodyDeff;
bodyDeff.type = b2_staticBody;
b2Vec2 k(10.0f,40.0f);
bodyDeff.position=k;
b2FixtureDef fixtureDeff;
fixtureDeff.density = 100.0;
fixtureDeff.friction = 1.0;
b2PolygonShape polygonShapee;
polygonShapee.SetAsBox(0.1f,0.3f);
fixtureDeff.shape = &polygonShapee;
b2Body* link = m_world->CreateBody( &bodyDeff );
bodyDeff.type = b2_dynamicBody;

//and some others for a revolute joint

//set up the common properties of the joint before entering the loop
b2RevoluteJointDef revoluteJointDeff;
revoluteJointDeff.localAnchorA.Set( 0, 0.1f);
revoluteJointDeff.localAnchorB.Set( 0, -0.1f);
for (int i = 1; i < 90; i++) { b2Vec2 kl(10.0f+0.3f*i,40.0f);
bodyDeff.position=kl;
if(i==89){bodyDeff.type=b2_staticBody;}
b2Body* newLink = m_world->CreateBody( &bodyDeff );
newLink->CreateFixture( &fixtureDeff );
//PhysicsSprite* segmentSprite = [PhysicsSprite spriteWithFile:@"rope_seg_new2.png"];
//[self addChild:segmentSprite];
//[segmentSprite setPhysicsBody:link];

revoluteJointDeff.bodyA = link;
revoluteJointDeff.bodyB = newLink;
m_world->CreateJoint( &revoluteJointDeff );

link = newLink;
}//prepare for next iteration
}

/*{
    b2BodyDef bodyDeff;
bodyDeff.type = b2_dynamicBody;
b2Vec2 k(110.0f,41.0f);
bodyDeff.position=k;
b2FixtureDef fixtureDeff;
fixtureDeff.density = 10.0;
fixtureDeff.friction = 1.0;
b2PolygonShape polygonShapee;
polygonShapee.SetAsBox(1.0f,0.3f);
fixtureDeff.shape = &polygonShapee;
b2Body* link = m_world->CreateBody( &bodyDeff );
b2Body* link1;
link1=link;
//bodyDeff.type = b2_dynamicBody;

//and some others for a revolute joint

//set up the common properties of the joint before entering the loop
b2RevoluteJointDef revoluteJointDeff;
revoluteJointDeff.collideConnected = false;

revoluteJointDeff.localAnchorA.Set( 0, 0.1f);
revoluteJointDeff.localAnchorB.Set( 0, -0.1f);
for (int i = 0; i < 14; i++) { b2Vec2 kl(110.0f+1.2f*i,41.0f);
bodyDeff.position=kl;
//if(i==89){bodyDeff.type=b2_staticBody;}
b2Body* newLink = m_world->CreateBody( &bodyDeff );
b2Vec2 v(500.0,0);
//newLink->SetLinearVelocity(v);
newLink->CreateFixture( &fixtureDeff );
//PhysicsSprite* segmentSprite = [PhysicsSprite spriteWithFile:@"rope_seg_new2.png"];
//[self addChild:segmentSprite];
//[segmentSprite setPhysicsBody:link];

revoluteJointDeff.bodyA = link;
revoluteJointDeff.bodyB = newLink;
m_world->CreateJoint( &revoluteJointDeff );

link = newLink;
}//prepare for next ite
for (int i=0;i<3;i++){b2Vec2 kl(110.0f+1.2f*(139),41.0f-1.2f*i);
        bodyDeff.position=kl;
//if(i==89){
//bodyDeff.type=b2_dynamicBody;
//}
b2Body* newLink = m_world->CreateBody( &bodyDeff );
newLink->CreateFixture( &fixtureDeff );
//PhysicsSprite* segmentSprite = [PhysicsSprite spriteWithFile:@"rope_seg_new2.png"];
//[self addChild:segmentSprite];
//[segmentSprite setPhysicsBody:link];

revoluteJointDeff.bodyA = link;
revoluteJointDeff.bodyB = newLink;
m_world->CreateJoint( &revoluteJointDeff );
link = newLink;
}
for (int i=0;i<14;i++){b2Vec2 kl(110.0f+1.2f*(139-i),41.0f-1.2f*(9));
        bodyDeff.position=kl;
//if(i==89){
//bodyDeff.type=b2_dynamicBody;
//}
b2Body* newLink = m_world->CreateBody( &bodyDeff );
newLink->CreateFixture( &fixtureDeff );
//PhysicsSprite* segmentSprite = [PhysicsSprite spriteWithFile:@"rope_seg_new2.png"];
//[self addChild:segmentSprite];
//[segmentSprite setPhysicsBody:link];

revoluteJointDeff.bodyA = link;
revoluteJointDeff.bodyB = newLink;
m_world->CreateJoint( &revoluteJointDeff );
link = newLink;
}
for (int i=0;i<3;i++){b2Vec2 kl(110.0f,41.0f-1.2f*(9)+1.2f*i);
        bodyDeff.position=kl;
//if(i==89){
//bodyDeff.type=b2_dynamicBody;
//}
b2Body* newLink = m_world->CreateBody( &bodyDeff );
newLink->CreateFixture( &fixtureDeff );
//PhysicsSprite* segmentSprite = [PhysicsSprite spriteWithFile:@"rope_seg_new2.png"];
//[self addChild:segmentSprite];
//[segmentSprite setPhysicsBody:link];

revoluteJointDeff.bodyA = link;
revoluteJointDeff.bodyB = newLink;
m_world->CreateJoint( &revoluteJointDeff );
link = newLink;
}
revoluteJointDeff.bodyA = link1;
revoluteJointDeff.bodyB = link;
m_world->CreateJoint( &revoluteJointDeff );

}*/
    /*b2ChainShape ch;
    for(int i=0;i<100;i=i+1)
    {
       vertices[i].x=10.0f+0.2f*i;
       vertices[i].y=40.0f;
      //ch.CreateChain(vertices , 100);
      b2PolygonShape shape2;
      shape2.SetAsBox(0.2f, 0.05f);
      b2BodyDef bd3;
      bd3.position.Set(10.0f+0.2*i, 40.0f);
      //if(i!=0&&i!=99)bd3.type = b2_dynamicBody;
       bs1 = m_world->CreateBody(&bd3);
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 0.01f;
      fd3->shape = new b2PolygonShape;
      fd3->shape = &shape2;
      bs1->CreateFixture(fd3);

      b2PolygonShape shape22;
      shape22.SetAsBox(0.2f, 0.05f);
      b2BodyDef bd33;
      bd33.position.Set(10.0f+0.2*i+0.2f, 40.0f);
      bd33.type = b2_dynamicBody;
       bs2 = m_world->CreateBody(&bd33);
      b2FixtureDef *fd33 = new b2FixtureDef;.
      fd33->density = 0.01f;
      fd33->shape = new b2PolygonShape;
      fd33->shape = &shape22;
      bs2->CreateFixture(fd33);

      b2RevoluteJointDef jds;
      b2Vec2 anchors;
      anchors.Set(10.0f+0.2*i+0.1f, 40.0f);
      jds.Initialize(bs1, bs2, anchors);
      m_world->CreateJoint(&jds);
      if(i!=0&&i!=99)bd3.type = b2_dynamicBody;*/



/*

   / }
    ch.CreateChain(vertices , 100);*/
    {


          //static b2Body* body;

      b2PolygonShape poly;
      b2Vec2 vertices[5];
      vertices[0].Set(0.0f,0.0f);
      vertices[1].Set(8.0f,0.0f);
      vertices[2].Set(8.0f,1.0f);
      vertices[3].Set(0.0f,1.0f);
      vertices[4].Set(6.0f,3.0f);
      vertices[5].Set(2.0f,3.0f);
      poly.Set(vertices, 6);
      b2FixtureDef wedgefd;
      wedgefd.shape = &poly;
      wedgefd.density = 5.0f;
      wedgefd.friction = 1.0f;
      wedgefd.restitution = 1.0f;
      b2BodyDef wedgebd;
      wedgebd.type = b2_dynamicBody;
      wedgebd.position.Set(-84.0f, 91.4f);
      sbody = m_world->CreateBody(&wedgebd);
      sbody->CreateFixture(&wedgefd);
      vel=sbody->GetLinearVelocity();
      vel.x=0.0f;vel.y=0.0f;

//m_world->CreateJoint(&xd);     // if(vel.y==0){if(x1==true&&y1==false){vel.x=vel.x-5;}
      //              if(x1==false&&y1==false){vel.x=vel.x+5;}}

     //sbody->SetLinearVelocity(vel);



      /*b2PolygonShape shape;
      shape.SetAsBox(1.6f, 0.1f);

      b2FixtureDef fd;
      fd.shape = &shape;
      fd.density = 20.0f;
      fd.friction = 0.1f;



	  b2BodyDef bd1;
	  bd1.type = b2_dynamicBody;
	  bd1.position.Set(-81.0f , 90.7f);
	  b2Body* body5 = m_world->CreateBody(&bd1);
	  body5->CreateFixture(&fd);*/


      b2CircleShape circle;
      circle.m_radius = 0.7;


      b2FixtureDef ballfd;

      ballfd.shape = &circle;
      ballfd.density = 1.0f;
      ballfd.friction = 0.70f;
      ballfd.restitution = 0.5f;

      for (int i = 0; i < 2; ++i)
	{
	  b2BodyDef ballbd;
	  ballbd.type = b2_dynamicBody;
	  ballbd.position.Set(-82.6f + i*5.2f, 89.7f);
	  if(i==0){spherebody1 = m_world->CreateBody(&ballbd);
	  spherebody1->CreateFixture(&ballfd);}
	  else{spherebody2 = m_world->CreateBody(&ballbd);
	  spherebody2->CreateFixture(&ballfd);}

	  b2PolygonShape shape;
      shape.SetAsBox(0.1f, 1.5f);

      b2FixtureDef fd;
      fd.shape = &shape;
      fd.density = 20.0f;
      fd.friction = 0.1f;



	  b2BodyDef bd1;
	  bd1.type = b2_dynamicBody;
	  bd1.position.Set(-82.6f + i*5.2f, 91.2f);
if(i==0){ body5 = m_world->CreateBody(&bd1);
	  body5->CreateFixture(&fd);}
else{ body55 = m_world->CreateBody(&bd1);
	  body55->CreateFixture(&fd);}


	 // b2Body* b11;
     // b2EdgeShape sshape;
     // sshape.Set(b2Vec2(-90.0f, 87.0f), b2Vec2(90.0f, 87.0f));
     // b2BodyDef bdd;
     // b11 = m_world->CreateBody(&bdd);
     // b11->CreateFixture(&sshape, 0.0f);



	  b2RevoluteJointDef jd;
      b2Vec2 anchor;
      anchor.Set(-82.6f + i*5.2f, 89.7f);
      if(i==0)jd.Initialize(spherebody1, body5, anchor);
      else jd.Initialize(spherebody2, body55, anchor);
      m_world->CreateJoint(&jd);

b2PrismaticJointDef jointDef;
b2Vec2 worldAxis(0.0f, 1.0f);

if(i==0)jointDef.Initialize(body5, sbody, body5->GetWorldCenter(), worldAxis);
else jointDef.Initialize(body55, sbody, body55->GetWorldCenter(), worldAxis);

jointDef.lowerTranslation = -0.5f;

jointDef.upperTranslation = 1.0f;

jointDef.enableLimit = true;

jointDef.maxMotorForce = 2.0f;

jointDef.motorSpeed = 5.0f;

jointDef.enableMotor = true;
m_world->CreateJoint(&jointDef);


}
/*b2Vec2 xxx(-80.0f, 91.4f);
b2PrismaticJointDef jointDefv;
b2Vec2 worldAxis1(0.0f, 1.0f);

jointDefv.Initialize(body5, body55, xxx, worldAxis1);

jointDefv.lowerTranslation = -0.5f;

jointDefv.upperTranslation = 1.0f;

jointDefv.enableLimit = true;

jointDefv.maxMotorForce = 2.0f;

jointDefv.motorSpeed = 5.0f;

jointDefv.enableMotor = true;
m_world->CreateJoint(&jointDefv);*/




/*b2PrismaticJointDef jointDef;

b2Vec2 worldAxis(0.0f, 1.0f);

jointDef.Initialize(body5, sbody, body5->GetWorldCenter(), worldAxis);

jointDef.lowerTranslation = -0.0f;

jointDef.upperTranslation = 1 .0f;

jointDef.enableLimit = true;

jointDef.maxMotorForce = 1.0f;

jointDef.motorSpeed = 0.0f;

jointDef.enableMotor = true;
m_world->CreateJoint(&jointDef);*/


      }
    /*{
      b2PolygonShape shape;
      shape.SetAsBox(6.0f, 0.25f);

      b2BodyDef bd;
      bd.position.Set(-31.0f, 30.0f);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }*/





    //Top horizontal shelf
   /* {
      b2PolygonShape shape;
      shape.SetAsBox(6.0f, 0.25f);

      b2BodyDef bd;
      bd.position.Set(-31.0f, 30.0f);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }

    //Dominos
    {
      b2PolygonShape shape;
      shape.SetAsBox(0.1f, 1.0f);

      b2FixtureDef fd;
      fd.shape = &shape;
      fd.density = 20.0f;
      fd.friction = 0.1f;

      for (int i = 0; i < 10; ++i)
	{
	  b2BodyDef bd;
	  bd.type = b2_dynamicBody;
	  bd.position.Set(-35.5f + 1.0f * i, 31.25f);
	  b2Body* body = m_world->CreateBody(&bd);
	  body->CreateFixture(&fd);
	}
    }

    //Another horizontal shelf
    {
      b2PolygonShape shape;
      shape.SetAsBox(7.0f, 0.25f, b2Vec2(-20.f,20.f), 0.0f);

      b2BodyDef bd;
      bd.position.Set(1.0f, 6.0f);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }


    //The pendulum that knocks the dominos off
    {
      b2Body* b2;
      {
	b2PolygonShape shape;
	shape.SetAsBox(0.25f, 1.5f);

	b2BodyDef bd;
	bd.position.Set(-36.5f, 28.0f);
	b2 = m_world->CreateBody(&bd);
	b2->CreateFixture(&shape, 10.0f);
      }

      b2Body* b4;
      {
	b2PolygonShape shape;
	shape.SetAsBox(0.25f, 0.25f);

	b2BodyDef bd;
	bd.type = b2_dynamicBody;
	bd.position.Set(-40.0f, 33.0f);
	b4 = m_world->CreateBody(&bd);
	b4->CreateFixture(&shape, 2.0f);
      }

      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      anchor.Set(-37.0f, 40.0f);
      jd.Initialize(b2, b4, anchor);
      m_world->CreateJoint(&jd);
    }

    //The train of small spheres
    {
      b2Body* spherebody;

      b2CircleShape circle;
      circle.m_radius = 0.5;

      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 1.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.0f;

      for (int i = 0; i < 1; ++i)
	{
	  b2BodyDef ballbd;
	  ballbd.type = b2_dynamicBody;
	  ballbd.position.Set(-22.2f + i*1.0, 26.6f);
	  spherebody = m_world->CreateBody(&ballbd);
	  spherebody->CreateFixture(&ballfd);
	}
    }

    //The pulley system
    {
      b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_dynamicBody;
      bd->position.Set(-10,15);
      bd->fixedRotation = true;

      //The open box
      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 10.0;
      fd1->friction = 0.5;
      fd1->restitution = 0.f;
      fd1->shape = new b2PolygonShape;
      b2PolygonShape bs1;
      bs1.SetAsBox(2,0.2, b2Vec2(0.f,-1.9f), 0);
      fd1->shape = &bs1;
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 10.0;
      fd2->friction = 0.5;
      fd2->restitution = 0.f;
      fd2->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(0.2,2, b2Vec2(2.0f,0.f), 0);
      fd2->shape = &bs2;
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 10.0;
      fd3->friction = 0.5;
      fd3->restitution = 0.f;
      fd3->shape = new b2PolygonShape;
      b2PolygonShape bs3;
      bs3.SetAsBox(0.2,2, b2Vec2(-2.0f,0.f), 0);
      fd3->shape = &bs3;

      b2Body* box1 = m_world->CreateBody(bd);
      box1->CreateFixture(fd1);
      box1->CreateFixture(fd2);
      box1->CreateFixture(fd3);

      //The bar
      bd->position.Set(10,15);
      fd1->density = 30.1565;
      b2Body* box2 = m_world->CreateBody(bd);
      box2->CreateFixture(fd1);

      // The pulley joint
      b2PulleyJointDef* myjoint = new b2PulleyJointDef();
      b2Vec2 worldAnchorOnBody1(-10, 15); // Anchor point on body 1 in world axis
      b2Vec2 worldAnchorOnBody2(10, 15); // Anchor point on body 2 in world axis
      b2Vec2 worldAnchorGround1(-10, 20); // Anchor point for ground 1 in world axis
      b2Vec2 worldAnchorGround2(10, 20); // Anchor point for ground 2 in world axis
      float32 ratio = 1.0f; // Define ratio
      myjoint->Initialize(box1, box2, worldAnchorGround1, worldAnchorGround2, box1->GetWorldCenter(), box2->GetWorldCenter(), ratio);
      m_world->CreateJoint(myjoint);
    }

    //The revolving horizontal platform
    {
      b2PolygonShape shape;
      shape.SetAsBox(2.2f, 0.2f);

      b2BodyDef bd;
      bd.position.Set(14.0f, 14.0f);
      bd.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->CreateFixture(fd);

      b2PolygonShape shape2;
      shape2.SetAsBox(0.2f, 2.0f);
      b2BodyDef bd2;
      bd2.position.Set(14.0f, 16.0f);
      b2Body* body2 = m_world->CreateBody(&bd2);

      b2RevoluteJointDef jointDef;
      jointDef.bodyA = body;
      jointDef.bodyB = body2;
      jointDef.localAnchorA.Set(0,0);
      jointDef.localAnchorB.Set(0,0);
      jointDef.collideConnected = false;
      m_world->CreateJoint(&jointDef);
    }

    //The heavy sphere on the platform
    {
      b2Body* sbody;
      b2CircleShape circle;
      circle.m_radius = 1.0;

      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 0.099f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.0f;
      b2BodyDef ballbd;
      ballbd.type = b2_dynamicBody;
      ballbd.position.Set(14.0f, 18.0f);
      sbody = m_world->CreateBody(&ballbd);
      sbody->CreateFixture(&ballfd);
    }*/





    //The see-saw system at the bottom
    /*{
      //The triangle wedge
      b2Body* sbody;
      b2PolygonShape poly;
      b2Vec2 vertices[3];
      vertices[0].Set(-1,0);
      vertices[1].Set(1,0);
      vertices[2].Set(0,1.5);
      poly.Set(vertices, 3);
      b2FixtureDef wedgefd;
      wedgefd.shape = &poly;
      wedgefd.density = 10.0f;
      wedgefd.friction = 0.0f;
      wedgefd.restitution = 0.0f;
      b2BodyDef wedgebd;
      wedgebd.position.Set(30.0f, 0.0f);
      sbody = m_world->CreateBody(&wedgebd);
      sbody->CreateFixture(&wedgefd);

      //The plank on top of the wedge
      b2PolygonShape shape;
      shape.SetAsBox(15.0f, 0.2f);
      b2BodyDef bd2;
      bd2.position.Set(30.0f, 1.5f);
      bd2.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd2);
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 1.f;
      fd2->shape = new b2PolygonShape;
      fd2->shape = &shape;
      body->CreateFixture(fd2);

      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      anchor.Set(27.0f, 1.5f);
      jd.Initialize(sbody, body, anchor);
      m_world->CreateJoint(&jd);

      //The light box on the right side of the see-saw
      b2PolygonShape shape2;
      shape2.SetAsBox(2.0f, 2.0f);
      b2BodyDef bd3;
      bd3.position.Set(40.0f, 2.0f);
      bd3.type = b2_dynamicBody;
      b2Body* body3 = m_world->CreateBody(&bd3);
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 0.01f;
      fd3->shape = new b2PolygonShape;
      fd3->shape = &shape2;
      body3->CreateFixture(fd3);
    }
  }*/
}

     void dominos_t::Key_board(unsigned char key,int32 x,int32 y)
        {
  B2_NOT_USED(x);
    B2_NOT_USED(y);
    x0=x;y0=y;
    //static b2Vec2 vel;


    x1=true;y1=true;

    switch (key)
    {
      case 'q': //move left
        vel.x=0.5f;vel.y=0.0f;
        x1=false;y1=false;
       // sbody->SetLinearVelocity( vel );
        break;
      case 'w': //stop
        vel.x=-.5f;vel.y=0.0f;
        x1=true;y1=false;
        //sbody->SetLinearVelocity( vel );
        break;
      case 'e': //move right
        sbody->SetAngularVelocity(0);
        x1=true;y1=true;
       // sbody->SetLinearVelocity( vel );
        break;
      case 'h':
        vel.x=0;vel.y=0;
        sbody->SetLinearVelocity( vel );
        sbody->SetAngularVelocity(0);
        break;
       case 'i':
sbody->SetAngularVelocity(3);//sbody->SetTransform( sbody->GetPosition(), 3-1*bodyAngle );
       break;
      /* case 'a':
       er=sbody->GetPosition();
       er.y=1.4f;
       gh=sbody->GetAngle();
       sbody->SetTransform(er,gh);
       //sbody->SetType(b2_staticBody);
       break;*/
      default:
      vel.x=0;vel.y=0;
      x1=false;y1=true;
     // sbody->SetLinearVelocity( vel );
        //run default behaviour
        //Test::Keyboard(key);

    }
    force = spherebody1->GetMass() * vel.x / (1/1000.0);
    b2Vec2 cir2=spherebody1->GetPosition();
    cir2.y=cir2.y+0.5f;
    spherebody1->ApplyForce( b2Vec2(force,0.0f), cir2 ,true);
    vel.x=0;vel.y=0;
    }

   void dominos_t::sed(int32 button,int32 state,int32 x,int32 y)
    {//B2_NOT_USED(x);
    //B2_NOT_USED(y);
    //if (button == GLUT_LEFT_BUTTON){
    //int mod = glutGetModifiers();
   //if (state == GLUT_DOWN){
    //st=state;
    //float32 u = x / static_cast<float32>(tww);
    //float32 v = (thh - y) / float32(thh);

    ////float32 ratio = static_cast<float32>(tww) / static_cast<float32>(thh);
    //b2Vec2 extents(ratio * 25.0f, 25.0f);
    //extents *= view_zoom1;

   //// b2Vec2 lower = settings1.view_center - extents;
    //b2Vec2 upper = settings1.view_center + extents;
    //x=u;y=v;
    b2Vec2 p;
    p.x=10;p.y=10;
    //p.x = (1.0f - u) * lower.x + u * upper.x;
    //p.y = (1.0f - v) * lower.y + v * upper.y;
    //p.y=y%10;
    //p.x=x%10;
    float re=sbody->GetAngle();

    //p=sbody->GetPosition();
    b2Vec2 p1=sbody->GetPosition();
    p1.x=p1.x+p.x;
    p1.y=p1.y+p.y;
    float s1=spherebody1->GetAngle();
    b2Vec2 p2=spherebody1->GetPosition();
    p2.x=p.x+p2.x;
    p2.y=p2.y+p.y;
    if(button==GLUT_LEFT_BUTTON&&state==GLUT_DOWN)spherebody1->SetTransform(p2,s1);
    float s2=spherebody2->GetAngle();
    b2Vec2 p3=spherebody2->GetPosition();
    p3.x=p.x+p3.x;
    p3.y=p3.y+p.y;
    if(button==GLUT_LEFT_BUTTON&&state==GLUT_DOWN)spherebody2->SetTransform(p3,s2);
    float s3=body5->GetAngle();
    b2Vec2 p4=body5->GetPosition();
    p4.x=p.x+p4.x;
    p4.y=p4.y+p.y;
    if(button==GLUT_LEFT_BUTTON&&state==GLUT_DOWN)body5->SetTransform(p4,s3);
    float s4=body55->GetAngle();
    b2Vec2 p5=body55->GetPosition();
    p5.x=p.x+p5.x;
    p5.y=p5.y+p.y;
    if(button==GLUT_LEFT_BUTTON&&state==GLUT_DOWN)body55->SetTransform(p5,s4);
    //p.x=p.x+10.0f;
    //b2MouseJointDef xd;
     // xd.b2JointDef::b2JointDef();
      //xd.b2MouseJointDef();
     // b2Vec2 xx=callbacks_t::convert_screen_to_world( x0,  y0);
    if(button==GLUT_LEFT_BUTTON&&state==GLUT_DOWN) sbody->SetTransform(p1,re);
     //sbody->SetType(b2_staticBody);
      if(button==GLUT_LEFT_BUTTON&&state==GLUT_DOWN)sbody->SetAwake(true);
//xd.target=x;
    //xd.maxForce=100.0f;
      //xd.bodyA=b22;
      //xd.bodyB=sbody;
      //xd.target=p;
     // xd.collideConnected = true;
//xd.maxForce = 1000 * sbody->GetMass();
//xd.dampingRatio = 0;
//m_world->CreateJoint(&xd);
}



void dominos_t::sed1()
{if (st == GLUT_UP){}}

  /*    b2Body* m_car;
	b2Body* m_wheel1;
	b2Body* m_wheel2;

	float32 m_hz;
	float32 m_zeta;
	float32 m_speed;
	b2WheelJoint* m_spring1;
	b2WheelJoint* m_spring2;*/


   // sbody->SetLinearVelocity(body->GetLinearVelocity());


      /*Car::Car()
       {
		m_hz = 4.0f;
		m_zeta = 0.7f;
		m_speed = 50.0f;

		b2Body* ground = NULL;
		{
			b2BodyDef bd;
			ground = m_world->CreateBody(&bd);

			b2EdgeShape shape;

			b2FixtureDef fd;
			fd.shape = &shape;
			fd.density = 0.0f;
			fd.friction = 0.6f;

			shape.Set(b2Vec2(-20.0f, 0.0f), b2Vec2(20.0f, 0.0f));
			ground->CreateFixture(&fd);

			float32 hs[10] = {0.25f, 1.0f, 4.0f, 0.0f, 0.0f, -1.0f, -2.0f, -2.0f, -1.25f, 0.0f};

			float32 x = 20.0f, y1 = 0.0f, dx = 5.0f;

			for (int32 i = 0; i < 10; ++i)
			{
				float32 y2 = hs[i];
				shape.Set(b2Vec2(x, y1), b2Vec2(x + dx, y2));
				ground->CreateFixture(&fd);
				y1 = y2;
				x += dx;
			}

			for (int32 i = 0; i < 10; ++i)
			{
				float32 y2 = hs[i];
				shape.Set(b2Vec2(x, y1), b2Vec2(x + dx, y2));
				ground->CreateFixture(&fd);
				y1 = y2;
				x += dx;
			}

			shape.Set(b2Vec2(x, 0.0f), b2Vec2(x + 40.0f, 0.0f));
			ground->CreateFixture(&fd);

			x += 80.0f;
			shape.Set(b2Vec2(x, 0.0f), b2Vec2(x + 40.0f, 0.0f));
			ground->CreateFixture(&fd);

			x += 40.0f;
			shape.Set(b2Vec2(x, 0.0f), b2Vec2(x + 10.0f, 5.0f));
			ground->CreateFixture(&fd);

			x += 20.0f;
			shape.Set(b2Vec2(x, 0.0f), b2Vec2(x + 40.0f, 0.0f));
			ground->CreateFixture(&fd);

			x += 40.0f;
			shape.Set(b2Vec2(x, 0.0f), b2Vec2(x, 20.0f));
			ground->CreateFixture(&fd);
		}

		// Teeter
		{
			b2BodyDef bd;
			bd.position.Set(140.0f, 1.0f);
			bd.type = b2_dynamicBody;
			b2Body* body = m_world->CreateBody(&bd);

			b2PolygonShape box;
			box.SetAsBox(10.0f, 0.25f);
			body->CreateFixture(&box, 1.0f);

			b2RevoluteJointDef jd;
			jd.Initialize(ground, body, body->GetPosition());
			jd.lowerAngle = -8.0f * b2_pi / 180.0f;
			jd.upperAngle = 8.0f * b2_pi / 180.0f;
			jd.enableLimit = true;
			m_world->CreateJoint(&jd);

			body->ApplyAngularImpulse(100.0f, true);
		}

		// Bridge
		{
			int32 N = 20;
			b2PolygonShape shape;
			shape.SetAsBox(1.0f, 0.125f);

			b2FixtureDef fd;
			fd.shape = &shape;
			fd.density = 1.0f;
			fd.friction = 0.6f;

			b2RevoluteJointDef jd;

			b2Body* prevBody = ground;
			for (int32 i = 0; i < N; ++i)
			{
				b2BodyDef bd;
				bd.type = b2_dynamicBody;
				bd.position.Set(161.0f + 2.0f * i, -0.125f);
				b2Body* body = m_world->CreateBody(&bd);
				body->CreateFixture(&fd);

				b2Vec2 anchor(160.0f + 2.0f * i, -0.125f);
				jd.Initialize(prevBody, body, anchor);
				m_world->CreateJoint(&jd);

				prevBody = body;
			}

			b2Vec2 anchor(160.0f + 2.0f * N, -0.125f);
			jd.Initialize(prevBody, ground, anchor);
			m_world->CreateJoint(&jd);
		}

		// Boxes
		{
			b2PolygonShape box;
			box.SetAsBox(0.5f, 0.5f);

			b2Body* body = NULL;
			b2BodyDef bd;
			bd.type = b2_dynamicBody;

			bd.position.Set(230.0f, 0.5f);
			body = m_world->CreateBody(&bd);
			body->CreateFixture(&box, 0.5f);

			bd.position.Set(230.0f, 1.5f);
			body = m_world->CreateBody(&bd);
			body->CreateFixture(&box, 0.5f);

			bd.position.Set(230.0f, 2.5f);
			body = m_world->CreateBody(&bd);
			body->CreateFixture(&box, 0.5f);

			bd.position.Set(230.0f, 3.5f);
			body = m_world->CreateBody(&bd);
			body->CreateFixture(&box, 0.5f);

			bd.position.Set(230.0f, 4.5f);
			body = m_world->CreateBody(&bd);
			body->CreateFixture(&box, 0.5f);
		}

		// Car
		{
			b2PolygonShape chassis;
			b2Vec2 vertices[8];
			vertices[0].Set(-1.5f, -0.5f);
			vertices[1].Set(1.5f, -0.5f);
			vertices[2].Set(1.5f, 0.0f);
			vertices[3].Set(0.0f, 0.9f);
			vertices[4].Set(-1.15f, 0.9f);
			vertices[5].Set(-1.5f, 0.2f);
			chassis.Set(vertices, 6);

			b2CircleShape circle;
			circle.m_radius = 0.4f;

			b2BodyDef bd;
			bd.type = b2_dynamicBody;
			bd.position.Set(0.0f, 1.0f);
			m_car = m_world->CreateBody(&bd);
			m_car->CreateFixture(&chassis, 1.0f);

			b2FixtureDef fd;
			fd.shape = &circle;
			fd.density = 1.0f;
			fd.friction = 0.9f;

			bd.position.Set(-1.0f, 0.35f);
			m_wheel1 = m_world->CreateBody(&bd);
			m_wheel1->CreateFixture(&fd);

			bd.position.Set(1.0f, 0.4f);
			m_wheel2 = m_world->CreateBody(&bd);
			m_wheel2->CreateFixture(&fd);

			b2WheelJointDef jd;
			b2Vec2 axis(0.0f, 1.0f);

			jd.Initialize(m_car, m_wheel1, m_wheel1->GetPosition(), axis);
			jd.motorSpeed = 0.0f;
			jd.maxMotorTorque = 20.0f;
			jd.enableMotor = true;
			jd.frequencyHz = m_hz;
			jd.dampingRatio = m_zeta;
			m_spring1 = (b2WheelJoint*)m_world->CreateJoint(&jd);

			jd.Initialize(m_car, m_wheel2, m_wheel2->GetPosition(), axis);
			jd.motorSpeed = 0.0f;
			jd.maxMotorTorque = 10.0f;
			jd.enableMotor = false;
			jd.frequencyHz = m_hz;
			jd.dampingRatio = m_zeta;
			m_spring2 = (b2WheelJoint*)m_world->CreateJoint(&jd);
		}
	}
	x::x()
	{
    //body definition
    b2BodyDef myBodyDef;
    myBodyDef.type = b2_dynamicBody;

    //shape definition
    b2PolygonShape polygonShape;
    polygonShape.SetAsBox(1, 1); //a 2x2 rectangle

    //fixture definition
    b2FixtureDef myFixtureDef;
    myFixtureDef.shape = &polygonShape;
    myFixtureDef.density = 1;

    //create dynamic body
    myBodyDef.position.Set(0, 10);
    body = m_world->CreateBody(&myBodyDef);
    body->CreateFixture(&myFixtureDef);

    //a static body
    myBodyDef.type = b2_staticBody;
    myBodyDef.position.Set(0, 0);
    b2Body* staticBody = m_world->CreateBody(&myBodyDef);

    //add four walls to the static body
    polygonShape.SetAsBox( 20, 1, b2Vec2(0, 0), 0);//ground
    staticBody->CreateFixture(&myFixtureDef);
    polygonShape.SetAsBox( 20, 1, b2Vec2(0, 40), 0);//ceiling
    staticBody->CreateFixture(&myFixtureDef);
    polygonShape.SetAsBox( 1, 20, b2Vec2(-20, 20), 0);//left wall
    staticBody->CreateFixture(&myFixtureDef);
    polygonShape.SetAsBox( 1, 20, b2Vec2(20, 20), 0);//right wall
    staticBody->CreateFixture(&myFixtureDef);

    moveState = MS_STOP;
  }*/
 /* void Car::Key__board1(int key,int x,int y)
     {
         B2_NOT_USED(x);
    B2_NOT_USED(y);
	 switch (key)
		{case GLUT_ACTIVE_SHIFT:
		case GLUT_KEY_LEFT:
			m_spring1->SetMotorSpeed(m_speed);
			break;
		case GLUT_KEY_RIGHT:
			m_spring1->SetMotorSpeed(-m_speed);
			break;
			break;
        case 's':
			m_spring1->SetMotorSpeed(0.0f);
			break;

		case 'q':
			m_hz = b2Max(0.0f, m_hz - 1.0f);
			m_spring1->SetSpringFrequencyHz(m_hz);
			m_spring2->SetSpringFrequencyHz(m_hz);
			break;

		case 'e':
			m_hz += 1.0f;
			m_spring1->SetSpringFrequencyHz(m_hz);
			m_spring2->SetSpringFrequencyHz(m_hz);
			break;
		}
	 }*/
   /* void Car::key__board(unsigned char key,int x,int y)
        {
         B2_NOT_USED(x);
    B2_NOT_USED(y);
	 switch (key)
		{
		case 'a':
			m_spring1->SetMotorSpeed(m_speed);
			break;
		case 'd':
			m_spring1->SetMotorSpeed(-m_speed);
			break;
			break;
        case 's':
			m_spring1->SetMotorSpeed(0.0f);
			break;

		case 'q':
			m_hz = b2Max(0.0f, m_hz - 1.0f);
			m_spring1->SetSpringFrequencyHz(m_hz);
			m_spring2->SetSpringFrequencyHz(m_hz);
			break;

		case 'e':
			m_hz += 1.0f;
			m_spring1->SetSpringFrequencyHz(m_hz);
			m_spring2->SetSpringFrequencyHz(m_hz);
			break;
		}
	 }

	 void Car::Step(settings_t* settings)
	    {
		m_debug_draw.DrawString(5, m_text_line, "Keys: left = a, brake = s, right = d, hz down = q, hz up = e");
		m_text_line +=1;
		m_debug_draw.DrawString(5, m_text_line, "frequency = %g hz, damping ratio = %g", m_hz, m_zeta);
		m_text_line += 1;

		settings->view_center.x = m_car->GetPosition().x;
		base_sim_t::step(settings);
	}*/


  //callbacks_t::mouse_cb(int32 button, int32 state, int32 x, int32 y);
  sim_t *sim = new sim_t("Dominos project \nKeys:Acelerate=q;deccelerate=w;stop=h;\nrotate=i", dominos_t::create);
}
