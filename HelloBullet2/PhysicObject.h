#pragma once
#include "btBulletDynamicsCommon.h"
#include "irrlicht.h"
#include <iostream>
#include "audiere.h"

using namespace std;
using namespace irr;
using namespace audiere;

class PhysicObject
{
	// Graphics
	core::vector3df m_scale = core::vector3df(1.f, 1.f, 1.f);
	core::vector3df m_pos = core::vector3df(0.f, 0.f, 0.f);
	core::vector3df m_displacement = core::vector3df(0.f, 0.f, 0.f);
	scene::ISceneNode* m_node = 0;
	scene::ISceneNode* m_nodeFX = 0;
	// Physics
	//static btAlignedObjectArray<btCollisionShape*> collisionShapes;
	static btDynamicsWorld* m_world;
	btScalar m_mass = 0.f;
	btScalar m_restitution = 0.f;
	btVector3 m_localInertia = btVector3(0, 0, 0);
	btCollisionShape* m_collShape = 0;
	btTransform m_transform;
	bool m_isDynamic = false;
	btMotionState* m_motionState = 0;
	btRigidBody* m_body;
	SoundEffectPtr m_successSFX = NULL;
	std::string orchestraFn = "../media/orchestra-hit.mp3";
	core::matrix4 m_translationMatrix;

	
	//float m_fSscale = 0.5;
	//btVector3 HalfExtents(m_scale.X * m_fSscale, m_scale.Y * m_fSscale, m_scale.Z * m_fSscale);
	PhysicObject();
	btRigidBody* makeBulletMeshFromIrrlichtNode(const scene::ISceneNode* node);
	void teleport(btVector3 position, btQuaternion& orientation) const;

public:
	PhysicObject(scene::ISceneNode* node, btCollisionShape* shape, core::vector3df scale, float scaleFactor, 
		core::vector3df pos, btScalar mass, btScalar restitution, irr::core::vector3df displacement = irr::core::vector3df(0.f, 0.f, 0.f));
	//btRigidBody* makeBulletMeshFromIrrlichtNode(const scene::ISceneNode* node);
	void setPos(core::vector3df pos);
	core::vector3df getPos();
	scene::ISceneNode* getNode();
	btRigidBody* getBody();
	static btDynamicsWorld* getWorld();
	static void setWorld(btDynamicsWorld* world);
	void setLinearVelocity(btVector3 vel);
	void synchPos(core::vector3df pos, core::vector3df rot);
	void destroy();
	void hit();
	void addFX(irr::scene::ISceneNode* node);
};

