#pragma once
#include <iostream>
#include "btBulletDynamicsCommon.h"
#include <irrlicht.h>

using namespace std;
using namespace irr;
using namespace scene;

class BulletAnimatedMeshSceneNode :
	public scene::IAnimatedMeshSceneNode
{
private:
	static btDynamicsWorld* m_world;
	btScalar m_mass = 0.f;
	btScalar m_restitution = 0.f;
	btVector3 m_localInertia = btVector3(0, 0, 0);
	btCollisionShape* m_collShape = 0;
	btTransform m_transform;
	bool m_isDynamic = false;
	btMotionState* m_motionState = 0;
	btRigidBody* m_body;
	ISceneNode* m_node;

public:
	BulletAnimatedMeshSceneNode(btRigidBody* actor, ISceneNode* parent,
		scene::ISceneManager* mgr, s32 id,
		const core::vector3df& position = core::vector3df(0, 0, 0),
		const core::vector3df& rotation = core::vector3df(0, 0, 0),
		const core::vector3df& scale = core::vector3df(1.0f, 1.0f, 1.0f));
	void OnPreRender();
	void setPos(core::vector3df pos);
	//core::vector3df getPos();
	scene::ISceneNode* getNode();
	btRigidBody* getBody();
	static btDynamicsWorld* getWorld();
	static void setWorld(btDynamicsWorld* world);
	void setLinearVelocity(btVector3 vel);
	void destroy();

	// Ereditato tramite IAnimatedMeshSceneNode
	virtual void render() override;
	virtual const core::aabbox3d<f32>& getBoundingBox() const override;
	virtual void setCurrentFrame(f32 frame) override;
	virtual bool setFrameLoop(s32 begin, s32 end) override;
	virtual void setAnimationSpeed(f32 framesPerSecond) override;
	virtual f32 getAnimationSpeed() const override;
	virtual IShadowVolumeSceneNode* addShadowVolumeSceneNode(const IMesh* shadowMesh = 0, s32 id = -1, bool zfailmethod = true, f32 infinity = 1000.0f) override;
	virtual IBoneSceneNode* getJointNode(const c8* jointName) override;
	virtual IBoneSceneNode* getJointNode(u32 jointID) override;
	virtual u32 getJointCount() const override;
	virtual bool setMD2Animation(EMD2_ANIMATION_TYPE anim) override;
	virtual bool setMD2Animation(const c8* animationName) override;
	virtual f32 getFrameNr() const override;
	virtual s32 getStartFrame() const override;
	virtual s32 getEndFrame() const override;
	virtual void setLoopMode(bool playAnimationLooped) override;
	virtual bool getLoopMode() const override;
	virtual void setAnimationEndCallback(IAnimationEndCallBack* callback = 0) override;
	virtual void setReadOnlyMaterials(bool readonly) override;
	virtual bool isReadOnlyMaterials() const override;
	virtual void setMesh(IAnimatedMesh* mesh) override;
	virtual IAnimatedMesh* getMesh(void) override;
	virtual const SMD3QuaternionTag* getMD3TagTransformation(const core::stringc& tagname) override;
	virtual void setJointMode(E_JOINT_UPDATE_ON_RENDER mode) override;
	virtual void setTransitionTime(f32 Time) override;
	virtual void animateJoints(bool CalculateAbsolutePositions = true) override;
	virtual void setRenderFromIdentity(bool On) override;
	virtual ISceneNode* clone(ISceneNode* newParent = 0, ISceneManager* newManager = 0) override;
};

