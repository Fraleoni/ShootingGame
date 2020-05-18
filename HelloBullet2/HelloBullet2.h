#pragma once
#include "btBulletDynamicsCommon.h"
#include "irrlicht.h"
#include "BulletDebugRender.h"
#include "PhysicObject.h"
#include "Graphics.h"
#include "IrrEventReceiver.h""
#include <stdio.h>
#include <chrono>
#include <thread>
#include <string>
#include "MyShaderCallback.h"
#include "audiere.h"
#include <cstddef>
extern "C"
{
#include <lua.h>
#include <lauxlib.h>
#include <lualib.h>
}

#include "LuaBridge/LuaBridge.h"


bool isDebugging = false;
float maxDistance = 400.f;
float friction = .6f;
position2d<s32> healthPos = position2d<s32>(20, 20);
//PhysicObject* CreateSphere(Graphics* g, IMeshSceneNode* sferaNode, const float& sphereRestitution);
PhysicObject* CreateSphere(Graphics* graphics, vector3df pos, vector3df scale, btScalar mass, btScalar restitution);
PhysicObject* CreateBox(Graphics* g, vector3df pos, vector3df scale, btScalar mass, const float& restitution);
PhysicObject* CreateBox(Graphics* g, IMeshSceneNode* node, vector3df pos, vector3df scale, btScalar mass, const float& restitution);
PhysicObject* CreateCylinder(Graphics* g, vector3df pos, vector3df scale, btScalar mass, const float& restitution);
void CreateWorld(Graphics* graphics, PhysicObject*& t_movingBox, scene::IMeshSceneNode*& t_movingBoxNode, s32 materialType);
PhysicObject* CreateBox(Graphics* g, IMeshSceneNode* node, vector3df pos, vector3df scale, btScalar mass, const float& restitution);
void CreateWall(Graphics* g, s32 materialType, core::vector3df startpos = core::vector3df(0.f, 1.5f, 0.f));
IParticleSystemSceneNode* CreateExplosion(Graphics* g);
IParticleSystemSceneNode* CreateSmoke(Graphics* g);
//scene::IParticleSystemSceneNode* ps;
Graphics* graphics;

