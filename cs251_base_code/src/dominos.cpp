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
      shape.Set(b2Vec2(-90.0f, -50.0f), b2Vec2(900.0f, -50.0f));
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
      shape.Set(b2Vec2(110.0f, 20.0f), b2Vec2(209.40f, 20.0f));
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
      shape.Set(b2Vec2(210.0f, 19.0f), b2Vec2(230.0f, 19.0f));
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
      bdz->type = b2_dynamicBody;
      bdz->position.Set(245,15);
      bdz->fixedRotation = true;

      //The open box
      b2FixtureDef *fd1z = new b2FixtureDef;
      fd1z->density = 100.0;
      fd1z->friction = 0.5;
      fd1z->restitution = 0.f;
      fd1z->shape = new b2PolygonShape;
      b2PolygonShape bs1;
      bs1.SetAsBox(10,0.2, b2Vec2(0.0f,-10.0f), 0);
      fd1z->shape = &bs1;
      b2FixtureDef *fd2z = new b2FixtureDef;
      fd2z->density = 10.0;
      fd2z->friction = 0.5;
      fd2z->restitution = 0.f;
      fd2z->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(0.2,2.5, b2Vec2(10.0f,-7.50f), 0);
      fd2z->shape = &bs2;
      b2FixtureDef *fd3z = new b2FixtureDef;
      fd3z->density = 10.0;
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

      fd1z->density = 104.5;
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
      ballfd.density = 4.0f;
      ballfd.friction = 100.00f;
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








    /*{
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
    }*/
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


b2RevoluteJointDef revoluteJointDeff;
revoluteJointDeff.localAnchorA.Set( 0, 0.1f);
revoluteJointDeff.localAnchorB.Set( 0, -0.1f);
for (int i = 1; i < 90; i++) { b2Vec2 kl(10.0f+0.3f*i,40.0f);
bodyDeff.position=kl;
if(i==89){bodyDeff.type=b2_staticBody;}
b2Body* newLink = m_world->CreateBody( &bodyDeff );
newLink->CreateFixture( &fixtureDeff );


revoluteJointDeff.bodyA = link;
revoluteJointDeff.bodyB = newLink;
m_world->CreateJoint( &revoluteJointDeff );

link = newLink;
}
}

    {


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
      wedgefd.density = 7.0f;
      wedgefd.friction = 1.0f;
      wedgefd.restitution = 1.0f;
      b2BodyDef wedgebd;
      wedgebd.type = b2_dynamicBody;
      wedgebd.position.Set(-84.0f, 91.4f);
      sbody = m_world->CreateBody(&wedgebd);
      sbody->CreateFixture(&wedgefd);
      vel=sbody->GetLinearVelocity();
      vel.x=0.0f;vel.y=0.0f;


      b2CircleShape circle;
      circle.m_radius = 0.7;


      b2FixtureDef ballfd;

      ballfd.shape = &circle;
      ballfd.density = 2.0f;
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
      }

    }

     void dominos_t::Key_board(unsigned char key,int32 x,int32 y)
        {
  B2_NOT_USED(x);
    B2_NOT_USED(y);
    x0=x;y0=y;



    x1=true;y1=true;

    switch (key)
    {
      case 'q': //move left
        vel.x=0.5f;vel.y=0.0f;
        x1=false;y1=false;

        break;
      case 'w': //stop
        vel.x=-.5f;vel.y=0.0f;
        x1=true;y1=false;

        break;
      case 'e': //move right
        sbody->SetAngularVelocity(0);
        x1=true;y1=true;

        break;
      case 'h':
        vel.x=0;vel.y=0;
        sbody->SetLinearVelocity( vel );
        sbody->SetAngularVelocity(0);
        break;
       case 'i':
sbody->SetAngularVelocity(3);
       break;

      default:
      vel.x=0;vel.y=0;
      x1=false;y1=true;

    }
    force = spherebody1->GetMass() * vel.x / (1/1000.0);
    b2Vec2 cir2=spherebody1->GetPosition();
    cir2.y=cir2.y+0.5f;
    spherebody1->ApplyForce( b2Vec2(force,0.0f), cir2 ,true);
    vel.x=0;vel.y=0;
    }

   void dominos_t::sed(int32 button,int32 state,int32 x,int32 y)
    {
    b2Vec2 p;
    p.x=10;p.y=10;

    float re=sbody->GetAngle();


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

    if(button==GLUT_LEFT_BUTTON&&state==GLUT_DOWN) sbody->SetTransform(p1,re);

      if(button==GLUT_LEFT_BUTTON&&state==GLUT_DOWN)sbody->SetAwake(true);

}



void dominos_t::sed1()
{if (st == GLUT_UP){}}

  sim_t *sim = new sim_t("Dominos project \nKeys:Acelerate=q;deccelerate=w;stop=h;\nrotate=i", dominos_t::create);
}
