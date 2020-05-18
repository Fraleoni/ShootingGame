/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2007 Erwin Coumans  http://continuousphysics.com/Bullet/
This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:
1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

///-----includes_start-----
//#include "HelloBullet2.h"
#include "btBulletDynamicsCommon.h"
#include "irrlicht.h"
#include "BulletDebugRender.h"
#include "PhysicObject.h"
#include "Graphics.h"
//#include "IrrEventReceiver.h""
#include "MyEventReceiver.h"
#include <stdio.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <string>
#include <cmath>
#include "HelloBullet2.h"
#include "BulletAnimatedMeshSceneNode.h"

using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;
using namespace audiere; 
using namespace std;

SoundEffectPtr g_collision = NULL;
SoundEffectPtr g_shoot = NULL;

auto lastSound = std::chrono::steady_clock::now();
auto clayStart = std::chrono::steady_clock::now();

int main(int argc, char** argv)
{
	bool contactCB(btManifoldPoint & cp, const btCollisionObjectWrapper * obj1, int id1, int index1, const btCollisionObjectWrapper * obj2, int id2, int index2);
	///-----initialization_start-----
	int i;
	constexpr float groundRestitution = 0.65f;
	constexpr float sphereRestitution = 0.95f;
	irr::video::SMaterial debugMat;
	debugMat.Lighting = false;
	bool UseHighLevelShaders = true;
	bool UseCgShaders = true;
	gContactAddedCallback = contactCB;
	
	///------ LUA Initialization -------
	// create a Lua state
	lua_State* L = luaL_newstate();

	// load standard libs 
	luaL_openlibs(L);
	luabridge::getGlobalNamespace(L)
		//.beginNamespace("test")
		.beginClass<PhysicObject>("PhysicObject")
		.addConstructor<void(*) (scene::ISceneNode*, btCollisionShape*, core::vector3df, float, core::vector3df, btScalar, btScalar, vector3df)>()
		.addFunction("setPos", &PhysicObject::setPos)
		.addFunction("setLinearVelocity", &PhysicObject::setLinearVelocity)
		.endClass();
	//.endNamespace();

	///------Audiere Initialization-----
	std::string hitAudioFn = "../media/boulder.mp3";
	std::string cannonAudioFn = "../media/cannon.mp3";
	AudioDevicePtr audioDevice(audiere::OpenDevice()); 
	//OutputStreamPtr shoot(OpenSound(audioDevice, shootAudioFn.c_str(), false));
	g_collision = OpenSoundEffect(audioDevice, hitAudioFn.c_str(), MULTIPLE);
	g_shoot = OpenSoundEffect(audioDevice, cannonAudioFn.c_str(), MULTIPLE);
	
	///-----Irrlicht initialization----
	video::E_DRIVER_TYPE driverType = video::EDT_OPENGL;
	vector3df currRot = vector3df(0, 1, 0);
	//IrrEventReceiver receiver;
	MyEventReceiver receiver;
	IrrlichtDevice* device =
		createDevice(video::EDT_OPENGL, dimension2d<u32>(1280, 960), 32, false, false, true, &receiver);
	device->getCursorControl()->setVisible(false);
	graphics = &Graphics(device);
	IVideoDriver* driver = graphics->getVideoDriver();
	//********* BILLBOARD *************
	scene::IBillboardSceneNode* bill = graphics->getSceneManager()->addBillboardSceneNode();
	bill->setMaterialType(video::EMT_TRANSPARENT_ADD_COLOR);
	bill->setMaterialTexture(0, driver->getTexture("../media/mirino.bmp"));
	bill->setMaterialFlag(video::EMF_LIGHTING, false);
	bill->setMaterialFlag(video::EMF_ZBUFFER, false);
	bill->setSize(core::dimension2d<f32>(10.0f, 10.0f));
	//bill->setID(ID_IsNotPickable); // This ensures that we don't accidentally ray-pick it
	//********* PARTICLES ***********
	//IParticleSystemSceneNode ps = CreateSmoke(graphics);
	//********* SHADERS *************
	if (UseCgShaders && !driver->queryFeature(EVDF_CG))
	{
		printf("Warning: No Cg support, disabling.\n");
		UseCgShaders = false;
	}
	io::path vsFileName; // filename for the vertex shader
	io::path psFileName; // filename for the pixel shader

	switch (driverType)
	{
	case video::EDT_DIRECT3D8:
		psFileName = "../media/d3d8.psh";
		vsFileName = "../media/d3d8.vsh";
		break;
	case video::EDT_DIRECT3D9:
		if (UseHighLevelShaders)
		{
			// Cg can also handle this syntax
			psFileName = "../media/d3d9.hlsl";
			vsFileName = psFileName; // both shaders are in the same file
		}
		else
		{
			psFileName = "../media/d3d9.psh";
			vsFileName = "../media/d3d9.vsh";
		}
		break;

	case video::EDT_OPENGL:
		if (UseHighLevelShaders)
		{
			if (!UseCgShaders)
			{
				psFileName = "../media/opengl.frag";
				vsFileName = "../media/opengl.vert";
			}
			else
			{
				// Use HLSL syntax for Cg
				psFileName = "../media/d3d9.hlsl";
				vsFileName = psFileName; // both shaders are in the same file
			}
		}
		else
		{
			psFileName = "../media/opengl.psh";
			vsFileName = "../media/opengl.vsh";
		}
		break;
	}
	if (!driver->queryFeature(video::EVDF_PIXEL_SHADER_1_1) &&
		!driver->queryFeature(video::EVDF_ARB_FRAGMENT_PROGRAM_1))
	{
		device->getLogger()->log("WARNING: Pixel shaders disabled "\
			"because of missing driver/hardware support.");
		psFileName = "";
	}

	if (!driver->queryFeature(video::EVDF_VERTEX_SHADER_1_1) &&
		!driver->queryFeature(video::EVDF_ARB_VERTEX_PROGRAM_1))
	{
		device->getLogger()->log("WARNING: Vertex shaders disabled "\
			"because of missing driver/hardware support.");
		vsFileName = "";
	}
	// create materials
	cout << "vsFileName" << vsFileName.c_str() << endl;
	cout << "psFileName" << psFileName.c_str() << endl;
	video::IGPUProgrammingServices* gpu = driver->getGPUProgrammingServices();
	s32 newMaterialType1 = 0;
	s32 newMaterialType2 = 0;

	if (gpu)
	{
		MyShaderCallBack* mc = new MyShaderCallBack(device, UseHighLevelShaders, UseCgShaders);

		// create the shaders depending on if the user wanted high level
		// or low level shaders:

		if (UseHighLevelShaders)
		{
			// Choose the desired shader type. Default is the native
			// shader type for the driver, for Cg pass the special
			// enum value EGSL_CG
			const video::E_GPU_SHADING_LANGUAGE shadingLanguage =
				UseCgShaders ? video::EGSL_CG : video::EGSL_DEFAULT;

			// create material from high level shaders (hlsl, glsl or cg)

			newMaterialType1 = gpu->addHighLevelShaderMaterialFromFiles(
				vsFileName, "vertexMain", video::EVST_VS_1_1,
				psFileName, "pixelMain", video::EPST_PS_1_1,
				mc, video::EMT_SOLID, 0, shadingLanguage);

			newMaterialType2 = gpu->addHighLevelShaderMaterialFromFiles(
				vsFileName, "vertexMain", video::EVST_VS_1_1,
				psFileName, "pixelMain", video::EPST_PS_1_1,
				mc, video::EMT_TRANSPARENT_ADD_COLOR, 0, shadingLanguage);
		}
		else
		{
			// create material from low level shaders (asm or arb_asm)

			newMaterialType1 = gpu->addShaderMaterialFromFiles(vsFileName,
				psFileName, mc, video::EMT_SOLID);

			newMaterialType2 = gpu->addShaderMaterialFromFiles(vsFileName,
				psFileName, mc, video::EMT_TRANSPARENT_ADD_COLOR);
		}

		mc->drop();
	}
	//********* SHADERS *************
	/*IAnimatedMesh* t_horseMesh = graphics->getSceneManager()->getMesh("../media/horse/LD_HorseRtime02.obj");
	scene::IAnimatedMeshSceneNode* t_horse =  graphics->getSceneManager()->addAnimatedMeshSceneNode(
		t_horseMesh,
		0, 0);
	t_horse->setPosition(core::vector3df(10, 4, -10));
	t_horse->setRotation(core::vector3df(0, 90, 0));
	t_horse->setScale(core::vector3df(.5f));
	t_horse->setMaterialFlag(EMF_LIGHTING, false);*/
	//t_horse->setMaterialFlag(video::EMF_BACK_FACE_CULLING, false);
	//t_horse->setMaterialType(EMT_NORMAL_MAP_SOLID);
	//t_horse->setMaterialTexture(0, driver->getTexture("../media/horse/HorseMain2k00.jpg"));
	//t_horse->setMaterialTexture(0, driver->getTexture("../media/horse/Hair12Main2k.jpg"));

	graphics->getSceneManager()->addSkyBoxSceneNode(
		driver->getTexture("../media/elyvisions/arch3_up.png"),  //top
		driver->getTexture("../media/elyvisions/arch3_dn.png"),  //bottom
		driver->getTexture("../media/elyvisions/arch3_lf.png"),  //left
		driver->getTexture("../media/elyvisions/arch3_rt.png"),  //right
		driver->getTexture("../media/elyvisions/arch3_bk.png"),  //front
		driver->getTexture("../media/elyvisions/arch3_ft.png")); //back

	driver->setTextureCreationFlag(video::ETCF_CREATE_MIP_MAPS, true);
	//driver->setFog(video::SColor(0, 138, 125, 81), video::EFT_FOG_EXP, 50, 200, .03f, true, false);
	ISceneManager* smgr = graphics->getSceneManager();
	IGUIEnvironment* guienv = graphics->getGUIEnvironment();
	ICameraSceneNode* cam = smgr->addCameraSceneNodeFPS(0, 50.0, .03f, -1, 0, 0, false, 0.0, false, true);
	cam->setPosition(vector3df(0, 10, -30));
	cam->setRotation(vector3df(0, -0, 0));
	/*ALfloat listenerOri[] = { 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f };
	alListener3f(AL_POSITION, 0, 10, -30.f);
	// check for errors
	alListener3f(AL_VELOCITY, 0, 0, 0);
	// check for errors
	alListenerfv(AL_ORIENTATION, listenerOri);
	// check for errors*/


	device->setWindowCaption(L"Irrlicht Bullet 0.1");
	IGUIStaticText* txt = guienv->addStaticText(L"Ciao Bullet!", rect<int>(10, 10, 170, 22), false, true, 0, -1, true);
	guienv->addImage(driver->getTexture("../media/health.png"), healthPos);
	///-----Bullet initialization----
	BulletDebugRender bulletDebugRender = BulletDebugRender();
	bulletDebugRender.setGraphics(graphics);
	//bulletDebugRender.setDebugMode(DBG_DrawWireframe);
	int frames = 0;
	auto start = std::chrono::steady_clock::now();
	///collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
	btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
	///btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
	btBroadphaseInterface* overlappingPairCache = new btDbvtBroadphase();
	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;
	btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);
	PhysicObject::setWorld(dynamicsWorld);
	BulletAnimatedMeshSceneNode::setWorld(dynamicsWorld);
	dynamicsWorld = 0;
	if (isDebugging)
	{
		PhysicObject::getWorld()->debugDrawWorld();
		PhysicObject::getWorld()->setDebugDrawer(&bulletDebugRender);
	}
	PhysicObject::getWorld()->setGravity(btVector3(0, -9.8, 0));
	//keep track of the shapes, we release memory at exit.
	//make sure to re-use collision shapes among rigid bodies whenever possible!
	btAlignedObjectArray<btCollisionShape*> collisionShapes;

	///create a few basic rigid bodies
	IMeshSceneNode* t_movingBoxNode = 0;
	PhysicObject* t_movingBox = 0;

	//CreateWorld(graphics, t_movingBox, t_movingBoxNode, newMaterialType2);
	CreateWorld(graphics, t_movingBox, t_movingBoxNode, 0);

	// In order to do framerate independent movement, we have to know
	// how long it was since the last frame
	u32 t_then = device->getTimer()->getTime();
	// This is the movemen speed in units per second.
	const f32 MOVEMENT_SPEED = 5.f;
	float shootMaxPow = 29.f;
	float shootCurrPow = 29.f;
	float shootDeltaPow = 20.f;
	float shootIncrement = .5f;
	float shootVel = 60.0f;
	float shootMass = 1.8f;
	float billDistance = 40;
	bool createWallLock = false;
	while (device->run())
	{
		if(shootCurrPow < shootMaxPow) shootCurrPow += shootIncrement;
		// Calcolo la direzione di sparo e la posizione del mirino
		core::line3d<f32> ray;
		ray.start = cam->getPosition();// + vector3df(-5.f,5.f,0);
		ray.end = ray.start + (cam->getTarget() - ray.start).normalize() * shootVel;
		bill->setPosition(ray.start + (cam->getTarget() - ray.start).normalize() * billDistance);
		vector3df t_v = ray.getVector();
		btScalar t_mass(shootMass);

		// Work out a frame delta time.
		const u32 t_now = device->getTimer()->getTime();
		const f32 frameDeltaTime = (f32)(t_now - t_then) / 1000.f; // Time in seconds
		t_then = t_now;
		++frames;
		auto now = std::chrono::steady_clock::now();
		auto diff = now - start;
		auto end = now + std::chrono::milliseconds(long(frameDeltaTime));
		if (diff >= std::chrono::seconds(1)) {
			/*IMeshSceneNode* t_sferaNode = 0;
			vector3df t_scale = vector3df(1.f, 1.f, 1.f);
			vector3df t_pos = vector3df(0.f, 30.f, 0.f);
			btScalar t_mass(1);
			CreateSphere(graphics, t_pos, t_scale, t_mass, sphereRestitution);//*/
			start = now;
			//std::cout << "FPS: " << frames << std::endl;
			const std::wstring s = L"FPS: " + std::to_wstring(frames) + L" - Num objs: " + std::to_wstring(PhysicObject::getWorld()->getNumCollisionObjects());
			txt->setText(s.c_str());
			//printf("FPS: %d \n", frames);
			frames = 0;
		}
		
		PhysicObject::getWorld()->stepSimulation(1.f / 60.f, 10);
		btTransform trans;
		//print positions of all objects
		for (int j = PhysicObject::getWorld()->getNumCollisionObjects() - 1; j >= 0; j--)
		{
			btCollisionObject* obj = PhysicObject::getWorld()->getCollisionObjectArray()[j];
			btRigidBody* body = btRigidBody::upcast(obj);
			//if (body && body->getMotionState())
			if (body->getMotionState())
			{
				PhysicObject* t_physicObject = static_cast<PhysicObject*>(obj->getUserPointer());
				if (typeid(*t_physicObject) != typeid(PhysicObject)) {
					std::cout << "*";
					break;
				}//*/
				irr::scene::ISceneNode* node = t_physicObject->getNode();
				if (body->getUserIndex() == 1) { //Dynamic Box
					core::vector3df nodePosition = node->getPosition();
					if (receiver.IsKeyDown(irr::KEY_KEY_W))
						nodePosition.Y += MOVEMENT_SPEED * frameDeltaTime;
					else if (receiver.IsKeyDown(irr::KEY_KEY_S))
						nodePosition.Y -= MOVEMENT_SPEED * frameDeltaTime;
					if (receiver.IsKeyDown(irr::KEY_KEY_A))
						nodePosition.X -= MOVEMENT_SPEED * frameDeltaTime;
					else if (receiver.IsKeyDown(irr::KEY_KEY_D))
						nodePosition.X += MOVEMENT_SPEED * frameDeltaTime;
					// Pause  some seconds
					else if (receiver.IsKeyDown(irr::KEY_KEY_F))
						_sleep(2000);
					//Build Wall
					else if (shootCurrPow >= shootDeltaPow && receiver.IsKeyDown(irr::KEY_KEY_G)) {
						if (!createWallLock) {
							//createWallLock = true;
							shootCurrPow -= shootDeltaPow;
							if (shootCurrPow < 0) shootCurrPow = 0;
							CreateWall(graphics, 0, vector3df(0.f, 1.5f, 0.f));
						}
					}
					else if (receiver.IsKeyDown(irr::KEY_KEY_H))
						createWallLock = false;
					t_movingBox->setPos(nodePosition);
					if (receiver.IsKeyDown(irr::KEY_KEY_Z)) {
						auto now = std::chrono::steady_clock::now();
						auto diff = now - clayStart;
						if (diff > std::chrono::seconds(1)) {
							clayStart = std::chrono::steady_clock::now();
							vector3df t_scale = vector3df(4.f, 1.f, 4.f);
							vector3df t_pos = vector3df(-30.f, 5.f, 0.f);							
							//vector3df t_pos = vector3df(0.f, 1.f, -5.f);
							btVector3 t_vel = btVector3(10.f, 20.f, 0.f);
							//btVector3 t_vel = btVector3(0.f, 0.f, 0.f);
							float t_clayRestitution = 0.01f;
							t_mass = .1f;
							//shootCurrPow -= shootDeltaPow;
							//if (shootCurrPow < 0) shootCurrPow = 0;
							///*
							//PhysicObject* t_clay = CreateBox(graphics, t_pos, t_scale, t_mass, t_clayRestitution);
							//PhysicObject* t_clay = CreateSphere(graphics, t_pos, t_scale, t_mass, t_clayRestitution);
							PhysicObject* t_clay = CreateCylinder(graphics, t_pos, t_scale, t_mass, t_clayRestitution);
							//t_clay->getBody()->setCollisionFlags(t_movingBox->getBody()->getCollisionFlags() & ~btCollisionObject::CF_STATIC_OBJECT);
							t_clay->setLinearVelocity(t_vel);//*/
							t_clay->getBody()->setCollisionFlags(t_clay->getBody()->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
							t_clay->getBody()->setUserIndex(4);
						}
					}
					if (shootCurrPow >= shootDeltaPow && (receiver.IsKeyDown(irr::KEY_SPACE) || receiver.GetMouseState().LeftButtonDown)) {
						vector3df t_scale = vector3df(.5f, .5f, .5f);
						vector3df t_pos = cam->getPosition();
						shootCurrPow -= shootDeltaPow;
						if (shootCurrPow < 0) shootCurrPow = 0;
						///*
						PhysicObject* t_bullet = CreateSphere(graphics, t_pos, t_scale, t_mass, sphereRestitution);
						t_bullet->getBody()->setCollisionFlags(t_movingBox->getBody()->getCollisionFlags() & ~btCollisionObject::CF_STATIC_OBJECT);
						t_bullet->setLinearVelocity(btVector3(t_v.X, t_v.Y, t_v.Z));//*/
						t_bullet->getBody()->setCollisionFlags(t_bullet->getBody()->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
						t_bullet->getBody()->setUserIndex(2);
						//t_bullet->addFX(CreateExplosion(graphics));
						t_bullet->addFX(CreateSmoke(graphics));

						g_shoot->play();
						//**************************************************
						/*float t_scaleFactor = 0.5;
						core::vector3df scale(1.f, 1.f, 1.f);
						core::vector3df t_rot(0.f, 0.f, 0.f);
						IMeshSceneNode* sferaNode = graphics->getSceneManager()->addSphereSceneNode(scale.X);
						if (sferaNode) {
							sferaNode->setMaterialFlag(EMF_LIGHTING, false);
							sferaNode->setScale(scale);
							sferaNode->setPosition(t_pos);
							sferaNode->setMaterialTexture(0, graphics->getVideoDriver()->getTexture("../media/earth.jpg"));
						}
						btMotionState* m_motionState = 0;
						btTransform t_groundTransform;
						t_groundTransform.setIdentity();
						t_groundTransform.setOrigin(btVector3(t_pos.X, t_pos.Y, t_pos.Z));

						btVector3 m_localInertia = btVector3();
						bool m_isDynamic = (t_mass != 0.f);
						btCollisionShape* m_collShape = new btSphereShape(scale.X * t_scaleFactor);
						if (m_isDynamic)
							m_collShape->calculateLocalInertia(t_mass, m_localInertia);
						m_motionState = new btDefaultMotionState(t_groundTransform);
						btRigidBody::btRigidBodyConstructionInfo t_rbInfo(t_mass, m_motionState, m_collShape, m_localInertia);
						btRigidBody* m_body = new btRigidBody(t_rbInfo);

						BulletAnimatedMeshSceneNode* t_newBullet = new BulletAnimatedMeshSceneNode(m_body,
							sferaNode, graphics->getSceneManager(),-1, t_pos, t_rot, t_scale);
						t_newBullet->getBody()->setCollisionFlags(t_movingBox->getBody()->getCollisionFlags() & ~btCollisionObject::CF_STATIC_OBJECT);
						t_newBullet->setLinearVelocity(btVector3(t_v.X, t_v.Y, t_v.Z));//*/
						//**************************************************
					}
				}

				if (body->isActive()) {
					//Calculate distance from Player
					vector3df t_camPos = cam->getAbsolutePosition();
					btVector3 t_objPos = body->getCenterOfMassPosition();
					vector3df t_objPosIrr(t_objPos.getX(), t_objPos.getY(), t_objPos.getZ());
					float dist = t_camPos.getDistanceFrom(t_objPosIrr);
					//float dist = std::sqrt(std::pow(t_camPos.X - t_objPos.x(), 2) + std::pow(t_camPos.Y - t_objPos.y(), 2) + std::pow(t_camPos.Z - t_objPos.z(), 2));
					if (dist > maxDistance) {
						t_physicObject->destroy();
						break;
					}
					body->getMotionState()->getWorldTransform(trans);
					vector3df currPos = vector3df(
						float(trans.getOrigin().getX()),
						float(trans.getOrigin().getY()),
						float(trans.getOrigin().getZ()));
					//node->setPosition(currPos);
					irr::core::vector3df currRot;
					const btQuaternion& tQuat = body->getOrientation();
					irr::core::quaternion q(tQuat.getX(), tQuat.getY(), tQuat.getZ(), tQuat.getW());
					q.toEuler(currRot);
					currRot *= irr::core::RADTODEG;
					//node->setRotation(currRot);
					t_physicObject->synchPos(currPos, currRot);
				}
			}
			else
			{
				trans = obj->getWorldTransform();
				//printf("world pos object %d = %f,%f,%f\n", j, float(trans.getOrigin().getX()), float(trans.getOrigin().getY()), float(trans.getOrigin().getZ()));
			}
		}
		driver->beginScene(true, true, SColor(255, 100, 101, 140));
		smgr->drawAll();
		if (isDebugging)
		{
			driver->setMaterial(debugMat);
			driver->setTransform(irr::video::ETS_WORLD, irr::core::IdentityMatrix);
			PhysicObject::getWorld()->debugDrawWorld();
		}
		guienv->drawAll();
		SColor t_color = video::SColor(255, 255 - 255 * shootCurrPow/shootMaxPow, 255 * shootCurrPow/shootMaxPow * shootCurrPow / shootMaxPow, 158);
		driver->draw2DRectangle(t_color, recti(45, 27, 48 + shootCurrPow * 3, 40));
		driver->endScene();

		std::this_thread::sleep_until(end);
	}
	device->drop();

	//cleanup in the reverse order of creation/initialization
	///-----cleanup_start-----
	//remove the rigidbodies from the dynamics world and delete them
	for (i = PhysicObject::getWorld()->getNumCollisionObjects() - 1; i >= 0; i--)
	{
		btCollisionObject* obj = PhysicObject::getWorld()->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		PhysicObject::getWorld()->removeCollisionObject(obj);
		delete obj;
	}

	//delete collision shapes
	for (int j = 0; j < collisionShapes.size(); j++)
	{
		btCollisionShape* shape = collisionShapes[j];
		collisionShapes[j] = 0;
		delete shape;
	}

	//delete dynamics world
	delete PhysicObject::getWorld();

	//delete solver
	delete solver;

	//delete broadphase
	delete overlappingPairCache;

	//delete dispatcher
	delete dispatcher;

	delete collisionConfiguration;

	//next line is optional: it will be cleared by the destructor when the array goes out of scope
	collisionShapes.clear();
		
	return 0;
}

bool contactCB(btManifoldPoint& cp, const btCollisionObjectWrapper* obj1, int id1, int index1, const btCollisionObjectWrapper* obj2, int id2, int index2)
{
	auto now = std::chrono::steady_clock::now();
	auto diff = now - lastSound;
		
	if (diff > std::chrono::milliseconds(100)) {
		//std::cout << "Collision" << endl;
		float vel = 
			obj1->getCollisionObject()->getInterpolationLinearVelocity().length2() + 
			obj2->getCollisionObject()->getInterpolationLinearVelocity().length2();
		//if (obj1->getCollisionObject()->getUserIndex() + obj2->getCollisionObject()->getUserIndex() > 4)
		if (vel > 100.f) {
			lastSound = std::chrono::steady_clock::now();
			g_collision->play();
		}
	}
	int collisionIndex = obj1->getCollisionObject()->getUserIndex() + obj2->getCollisionObject()->getUserIndex();
	PhysicObject* t_po;
	switch (collisionIndex) {
	case 3: // Bullet and DynamicBox
	case 6: // Bullet and Clay
		if (obj1->getCollisionObject()->getUserIndex() == 4) {
			t_po = (PhysicObject*)obj1->getCollisionObject()->getUserPointer();
		}
		else {
			t_po = (PhysicObject*)obj2->getCollisionObject()->getUserPointer();
		}
		t_po->hit();
		t_po->addFX(CreateExplosion(graphics));
		break;
	case 10: // Bullet and Brick 
		if (obj1->getCollisionObject()->getUserIndex() == 8) {
			t_po = (PhysicObject*)obj1->getCollisionObject()->getUserPointer();
		}
		else {
			t_po = (PhysicObject*)obj2->getCollisionObject()->getUserPointer();
		}
		t_po->hit();
		t_po->addFX(CreateSmoke(graphics));
		break;
	}
/*	if (collisionIndex == 8) {
		PhysicObject* t_po = (PhysicObject*)obj1->getCollisionObject()->getUserPointer();
		t_po->hit();
	}//*/
	return false;
}

/*scene::IParticleSystemSceneNode createExplosion(Graphics* graphics) {
	scene::IParticleSystemSceneNode* ps =
		graphics->getSceneManager()->addParticleSystemSceneNode(false);

	scene::IParticleEmitter* em = ps->createBoxEmitter(
		core::aabbox3d<f32>(-2, 0, -2, 2, 1, 2), // emitter size
		core::vector3df(0.0f, 0.006f, 0.0f),   // initial direction
		80, 100,                             // emit rate
		video::SColor(0, 255, 255, 255),       // darkest color
		video::SColor(0, 255, 255, 255),       // brightest color
		400, 1000, 0,                         // min and max age, angle
		core::dimension2df(2.f, 2.f),         // min size
		core::dimension2df(4.f, 4.f));        // max size

	ps->setEmitter(em); // this grabs the emitter
	em->drop(); // so we can drop it here without deleting it

	scene::IParticleAffector* paf = ps->createFadeOutParticleAffector(video::SColor(1.f, 1.f, 0, 0), 1000);
	ps->addAffector(paf); // same goes for the affector
	paf->drop();

	return ps;
}//*/

void CreateWorld(Graphics* g, PhysicObject*& t_movingBox, scene::IMeshSceneNode*& movingBoxNode, s32 materialType)
{
	float groundRestitution = .35f;
	float sphereRestitution = .35f;
	video::IVideoDriver* driver = g->getVideoDriver();
	// GROUND
	{
		vector3df t_scale = vector3df(60.f, 2.f, 60.f);
		vector3df t_pos = vector3df(0.f, -0.5f, 0.f);
		btScalar t_mass(0.f);
		PhysicObject* t_ground = CreateBox(g, t_pos, t_scale, t_mass, groundRestitution);
		t_ground->getNode()->setMaterialTexture(0, driver->getTexture("../media/detailmap3.jpg"));
		t_ground->getBody()->setUserIndex(16);
		t_ground->getBody()->setFriction(.6f);

		t_ground->getBody()->setCollisionFlags(t_ground->getBody()->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
	}

	//DYNAMIC BOX
	{
		vector3df t_scale = vector3df(2.f, 2.f, 2.f);
		vector3df t_pos = vector3df(-5.f, -2.f, 0.f);
		btScalar t_mass(0.f);
		t_movingBox = CreateBox(g, t_pos, t_scale, t_mass, groundRestitution);
		scene::IMeshSceneNode* t_movingBoxNode = static_cast<IMeshSceneNode*>(t_movingBox->getNode());
		t_movingBoxNode->setMaterialTexture(0, driver->getTexture("../media/detailmap3.jpg"));
		t_movingBox->getBody()->setCollisionFlags(t_movingBox->getBody()->getCollisionFlags() & ~btCollisionObject::CF_STATIC_OBJECT);
		t_movingBox->getBody()->setUserIndex(1);
	}
	CreateWall(g, materialType, vector3df(0.f, 1.5f, 0.f));
	//CreateWall(g, materialType, vector3df(0.f, 1.5f, 4.f));
}

PhysicObject* CreateBox(Graphics* g, vector3df pos, vector3df scale, btScalar mass, const float& restitution)
{
	float t_scaleFactor = 0.5;
	IMeshSceneNode* t_pianoNode = g->getSceneManager()->addCubeSceneNode(1.0f, 0, -1, pos, vector3df(0), scale);
	if (t_pianoNode) {
		t_pianoNode->setMaterialFlag(EMF_LIGHTING, false);
		t_pianoNode->setMaterialTexture(0, g->getVideoDriver()->getTexture("../media/water.jpg"));
	}
	btVector3 t_halfExtents(scale.X * t_scaleFactor, scale.Y * t_scaleFactor, scale.Z * t_scaleFactor);
	btCollisionShape* t_pianoShape = new btBoxShape(t_halfExtents);
	PhysicObject* t_box = new PhysicObject(t_pianoNode, t_pianoShape, scale, t_scaleFactor, pos, mass, btScalar(restitution));

	return t_box;
}

PhysicObject* CreateCylinder(Graphics* g, vector3df pos, vector3df scale, btScalar mass, const float& restitution)
{
	float t_scaleFactor = 0.5f;
	btVector3 t_halfExtents(scale.X * t_scaleFactor, scale.Y * t_scaleFactor, scale.Z * t_scaleFactor);
	vector3df t_displacement = vector3df(0.f, -t_halfExtents.getY(), 0.f);
	IMesh* cylinderMesh = g->getSceneManager()->getGeometryCreator()->createCylinderMesh(
		scale.Y * t_scaleFactor, //radius
		scale.Y, // length
		20,      // tesselation
		SColor(255, 100, 100, 100), // color
		true,    // closeTop
		0.0f     // oblique
	);
	ISceneNode* t_cylinderNode = g->getSceneManager()->addMeshSceneNode(
		cylinderMesh,
		0,            //parent 
		-1,           //id 
		pos + t_displacement,//pos 
		vector3df(0), //rot 
		scale);       //scale
	if (t_cylinderNode) {
		t_cylinderNode->setMaterialFlag(EMF_LIGHTING, false);
		t_cylinderNode->setMaterialTexture(0, g->getVideoDriver()->getTexture("../media/water.jpg"));
	}
	btCollisionShape* t_cylinderShape = new btCylinderShape(t_halfExtents);
	PhysicObject* t_cylinder = new PhysicObject(t_cylinderNode, t_cylinderShape, scale, t_scaleFactor,
		pos, mass, btScalar(restitution), t_displacement);

	return t_cylinder;
}

PhysicObject* CreateBox(Graphics* g, IMeshSceneNode* node, vector3df pos, vector3df scale, btScalar mass, const float& restitution)
{
	float t_scaleFactor = 0.5;
	node = g->getSceneManager()->addCubeSceneNode(1.f);
	if (node) {
		node->setMaterialFlag(EMF_LIGHTING, false);
		node->setMaterialTexture(0, g->getVideoDriver()->getTexture("../media/water.jpg"));
	}
	btVector3 t_halfExtents(scale.X * t_scaleFactor, scale.Y * t_scaleFactor, scale.Z * t_scaleFactor);
	btCollisionShape* t_pianoShape = new btBoxShape(t_halfExtents);
	PhysicObject* t_box = new PhysicObject(node, t_pianoShape, scale, t_scaleFactor, pos, mass, btScalar(restitution));

	return t_box;
}

PhysicObject* CreateSphere(Graphics* g, vector3df pos, vector3df scale, btScalar mass, btScalar restitution)
{
	float t_scaleFactor = 0.5;
	IMeshSceneNode* sferaNode = g->getSceneManager()->addSphereSceneNode(scale.X);
	if (sferaNode) {
		sferaNode->setMaterialFlag(EMF_LIGHTING, false);
		sferaNode->setScale(scale);
		sferaNode->setPosition(pos);
		sferaNode->setMaterialTexture(0, g->getVideoDriver()->getTexture("../media/earth.jpg"));
	}
	//btVector3 t_halfExtents(t_Scale.X * 0.5f, t_Scale.Y * 0.5f, t_Scale.Z * 0.5f);
	btCollisionShape* sferaShape = new btSphereShape(scale.X * t_scaleFactor);
	float scaleFactor = 1;
	PhysicObject* t_sfera = new PhysicObject(sferaNode, sferaShape, scale, t_scaleFactor, pos, mass, btScalar(restitution));

	return t_sfera;
}//*/

void CreateWall(Graphics* g, s32 materialType, core::vector3df startpos) {
	int t_n = 3;
	//	vector3df t_scale = vector3df(.6f, .4f, .2f);
	vector3df t_scale = vector3df(6.f, 4.f, 2.f);
	float t_offsetX = t_n * t_scale.X / 2.f;
	// Centra rispetto a X 
	// vector3df t_startPos = vector3df(startpos.X - t_offsetX, startpos.Y, startpos.Z);
	vector3df t_startPos = startpos - vector3df(t_offsetX, 0.f, 0.f);
	float rest = 0.5f;
	for (int i = 0; i < t_n; i++) {
		for (int j = 0; j < t_n; j++) {
			vector3df t_pos = t_startPos + vector3df(t_scale.X / 2.f + i * t_scale.X, j * t_scale.Y, 0.f);
			btScalar t_mass(0.5f);
			PhysicObject* t_brick = CreateBox(g, t_pos, t_scale, t_mass, rest);
			scene::IMeshSceneNode* t_brickNode = static_cast<IMeshSceneNode*>(t_brick->getNode());
			t_brickNode->setMaterialTexture(0, g->getVideoDriver()->getTexture("../media/rockwall.jpg"));
			// Apply Shader
			if (materialType)
				t_brickNode->setMaterialType((video::E_MATERIAL_TYPE)materialType);
			//t_brick->getBody()->setCollisionFlags(t_brick->getBody()->getCollisionFlags() & ~btCollisionObject::CF_STATIC_OBJECT);
			//t_brick->getBody()->setUserIndex(1);
			t_brick->getBody()->setFriction(friction);
			t_brick->getBody()->setUserIndex(8);
		}
	}
}

IParticleSystemSceneNode* CreateExplosion(Graphics* g)
{
	irr::video::IVideoDriver* driver = g->getVideoDriver();
	IParticleSystemSceneNode* ps =
		g->getSceneManager()->addParticleSystemSceneNode(false);

	scene::IParticleEmitter* em = ps->createBoxEmitter(
		core::aabbox3d<f32>(-.5, -.5, -.5, .5, .5, .5), // emitter size
		core::vector3df(0.0f, 0.006f, 0.0f),  // initial direction
		80, 100,                              // emit rate
		video::SColor(0, 255, 0, 0),          // darkest color
		video::SColor(0, 255, 255, 255),      // brightest color
		400, 1000, 0,                         // min and max age, angle
		core::dimension2df(1.f, 1.f),         // min size
		core::dimension2df(2.f, 2.f));        // max size

	ps->setEmitter(em); // this grabs the emitter
	em->drop(); // so we can drop it here without deleting it

	scene::IParticleAffector* paf = ps->createFadeOutParticleAffector(video::SColor(1.f, 1.f, 0, 0), 1000);
	ps->addAffector(paf); // same goes for the affector
	paf->drop();

	ps->setPosition(core::vector3df(0, 0, 0));
	ps->setScale(core::vector3df(2, 2, 2));
	ps->setMaterialFlag(video::EMF_LIGHTING, false);
	//ps->setMaterialFlag(video::EMF_ZWRITE_ENABLE, false);
	ps->setMaterialTexture(0, driver->getTexture("../media/fire.bmp"));
	ps->setMaterialType(video::EMT_TRANSPARENT_ADD_COLOR);//*/

	return ps;
}

IParticleSystemSceneNode* CreateSmoke(Graphics* g)
{
	irr::video::IVideoDriver* driver = g->getVideoDriver();
	IParticleSystemSceneNode* ps =
		g->getSceneManager()->addParticleSystemSceneNode(false);

	scene::IParticleEmitter* em = ps->createBoxEmitter(
		core::aabbox3d<f32>(-.5, -.5, -.5, .5, .5, .5), // emitter size
		core::vector3df(0.0f, 0.006f, 0.0f),  // initial direction
		80, 100,                              // emit rate
		video::SColor(0, 0, 0, 0),            // darkest color
		video::SColor(0, 255, 255, 255),      // brightest color
		400, 1000, 0,                         // min and max age, angle
		core::dimension2df(1.f, 1.f),         // min size
		core::dimension2df(2.f, 2.f));        // max size

	ps->setEmitter(em); // this grabs the emitter
	em->drop(); // so we can drop it here without deleting it

	scene::IParticleAffector* paf = ps->createFadeOutParticleAffector(video::SColor(1.f, 1.f, 0, 0), 1000);
	ps->addAffector(paf); // same goes for the affector
	paf->drop();

	ps->setPosition(core::vector3df(0, 0, 0));
	ps->setScale(core::vector3df(2, 2, 2));
	ps->setMaterialFlag(video::EMF_LIGHTING, false);
	//ps->setMaterialFlag(video::EMF_ZWRITE_ENABLE, false);
	ps->setMaterialTexture(0, driver->getTexture("../media/smoke2.jpg"));
	ps->setMaterialTexture(1, driver->getTexture("../media/smoke3.jpg"));
	ps->setMaterialType(video::EMT_TRANSPARENT_ADD_COLOR);//*/

	return ps;
}
