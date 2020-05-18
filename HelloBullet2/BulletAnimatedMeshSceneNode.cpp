#include "BulletAnimatedMeshSceneNode.h"

btDynamicsWorld* BulletAnimatedMeshSceneNode::m_world;

BulletAnimatedMeshSceneNode::BulletAnimatedMeshSceneNode(
	btRigidBody* body,
	ISceneNode* node,
	scene::ISceneManager* mgr, s32 id,
	const core::vector3df& position,
	const core::vector3df& rotation,
	const core::vector3df& scale) :IAnimatedMeshSceneNode(node->getParent(), mgr, id, position, rotation, scale),
	m_body(body), m_node(node)
{
    btTransform t_groundTransform;
    t_groundTransform.setIdentity();
    t_groundTransform.setOrigin(btVector3(position.X, position.Y, position.Z));
    m_isDynamic = (m_mass != 0.f);
    if (m_isDynamic)
        m_collShape->calculateLocalInertia(m_mass, m_localInertia);
    m_motionState = new btDefaultMotionState(t_groundTransform);
    m_body->setUserPointer((void*)(this));
    m_body->setRestitution(m_restitution);
    m_world->addRigidBody(m_body);
}

void BulletAnimatedMeshSceneNode::OnPreRender()
{
    if (IsVisible)
    {
        // reorient/reposition the scene node every frame
        if (m_body && m_body->isActive())
        {
            btTransform trans;
            m_body->getMotionState()->getWorldTransform(trans);
            core::vector3df currPos = core::vector3df(
                float(trans.getOrigin().getX()),
                float(trans.getOrigin().getY()),
                float(trans.getOrigin().getZ()));
            irr::core::vector3df currRot;
            const btQuaternion& tQuat = m_body->getOrientation();
            irr::core::quaternion q(tQuat.getX(), tQuat.getY(), tQuat.getZ(), tQuat.getW());
            q.toEuler(currRot);
            currRot *= irr::core::RADTODEG;
            this->m_node->setPosition(currPos);
            this->m_node->setRotation(currRot);
        }
        // because this node supports rendering of mixed mode meshes consisting of
        // transparent and solid material at the same time, we need to go through all
        // materials, check of what type they are and register this node for the right
        // render pass according to that.

        video::IVideoDriver* driver = SceneManager->getVideoDriver();

        int transparentCount = 0;
        int solidCount = 0;
        int t_mCount = m_node->getMaterialCount();
        // count transparent and solid materials in this scene node
        for (u32 i = 0; i < t_mCount; ++i)
        {
            video::IMaterialRenderer* rnd =
                driver->getMaterialRenderer(m_node->getMaterial(i).MaterialType);

            if (rnd && rnd->isTransparent())
                ++transparentCount;
            else
                ++solidCount;

            if (solidCount && transparentCount)
                break;
        }

        // register according to material types counted
        //! but first, check if it's in our camera's frustum before we decide we want to register for rendering
        if (SceneManager->getActiveCamera()->getViewFrustum()->getBoundingBox().isPointInside(getPosition()))
        {
            if (solidCount)
                SceneManager->registerNodeForRendering(this, scene::ESNRP_SOLID);

            if (transparentCount)
                SceneManager->registerNodeForRendering(this, scene::ESNRP_TRANSPARENT);
        }
    }

    ISceneNode::OnRegisterSceneNode();
    s32 t_joinsNr = this->getJointCount();
    if (IsVisible)
        for (s32 i = 0; i < (s32)t_joinsNr; ++i)
            if (this->getJointNode(i))
                this->getJointNode(i)->OnRegisterSceneNode();
}

void BulletAnimatedMeshSceneNode::setWorld(btDynamicsWorld* world) {
    m_world = world;
}

btDynamicsWorld* BulletAnimatedMeshSceneNode::getWorld() {
    return m_world;
}

scene::ISceneNode* BulletAnimatedMeshSceneNode::getNode() {
    return m_node;
}

btRigidBody* BulletAnimatedMeshSceneNode::getBody() {
    return m_body;
}

void BulletAnimatedMeshSceneNode::setPos(irr::core::vector3df pos) {
    m_node->setPosition(pos);
    //reposition(btVector3(m_pos.X, m_pos.Y, m_pos.Z), btVector3(m_pos.X, m_pos.Y, m_pos.Z));
    btTransform t_groundTransform;
    t_groundTransform.setIdentity();
    t_groundTransform.setOrigin(btVector3(pos.X, pos.Y, pos.Z));
    m_body->setWorldTransform(t_groundTransform);
    m_motionState->setWorldTransform(t_groundTransform);
    m_body->activate();
    /*m_world->removeRigidBody(m_body);
    m_world->addRigidBody(m_body);*/
}

void BulletAnimatedMeshSceneNode::setLinearVelocity(btVector3 vel) {
    m_body->setLinearVelocity(vel);
    btTransform t_groundTransform;
    t_groundTransform.setIdentity();
    t_groundTransform.setOrigin(btVector3(getPosition().X, getPosition().Y, getPosition().Z));
    m_body->setWorldTransform(t_groundTransform);
    m_motionState->setWorldTransform(t_groundTransform);
    m_body->activate();
}

void BulletAnimatedMeshSceneNode::destroy()
{
    m_world->removeCollisionObject(m_body);
    delete m_body;
    m_node->remove();
    delete this;
}

void BulletAnimatedMeshSceneNode::render()
{
}

const core::aabbox3d<f32>& BulletAnimatedMeshSceneNode::getBoundingBox() const
{
    // TODO: inserire l'istruzione return qui
    return core::aabbox3d<f32>();
}

void BulletAnimatedMeshSceneNode::setCurrentFrame(f32 frame)
{
}

bool BulletAnimatedMeshSceneNode::setFrameLoop(s32 begin, s32 end)
{
    return false;
}

void BulletAnimatedMeshSceneNode::setAnimationSpeed(f32 framesPerSecond)
{
}

f32 BulletAnimatedMeshSceneNode::getAnimationSpeed() const
{
    return f32();
}

IShadowVolumeSceneNode* BulletAnimatedMeshSceneNode::addShadowVolumeSceneNode(const IMesh* shadowMesh, s32 id, bool zfailmethod, f32 infinity)
{
    return nullptr;
}

IBoneSceneNode* BulletAnimatedMeshSceneNode::getJointNode(const c8* jointName)
{
    return nullptr;
}

IBoneSceneNode* BulletAnimatedMeshSceneNode::getJointNode(u32 jointID)
{
    return nullptr;
}

u32 BulletAnimatedMeshSceneNode::getJointCount() const
{
    return u32();
}

bool BulletAnimatedMeshSceneNode::setMD2Animation(EMD2_ANIMATION_TYPE anim)
{
    return false;
}

bool BulletAnimatedMeshSceneNode::setMD2Animation(const c8* animationName)
{
    return false;
}

f32 BulletAnimatedMeshSceneNode::getFrameNr() const
{
    return f32();
}

s32 BulletAnimatedMeshSceneNode::getStartFrame() const
{
    return s32();
}

s32 BulletAnimatedMeshSceneNode::getEndFrame() const
{
    return s32();
}

void BulletAnimatedMeshSceneNode::setLoopMode(bool playAnimationLooped)
{
}

bool BulletAnimatedMeshSceneNode::getLoopMode() const
{
    return false;
}

void BulletAnimatedMeshSceneNode::setAnimationEndCallback(IAnimationEndCallBack* callback)
{
}

void BulletAnimatedMeshSceneNode::setReadOnlyMaterials(bool readonly)
{
}

bool BulletAnimatedMeshSceneNode::isReadOnlyMaterials() const
{
    return false;
}

void BulletAnimatedMeshSceneNode::setMesh(IAnimatedMesh* mesh)
{
}

IAnimatedMesh* BulletAnimatedMeshSceneNode::getMesh(void)
{
    return nullptr;
}

const SMD3QuaternionTag* BulletAnimatedMeshSceneNode::getMD3TagTransformation(const core::stringc& tagname)
{
    return nullptr;
}

void BulletAnimatedMeshSceneNode::setJointMode(E_JOINT_UPDATE_ON_RENDER mode)
{
}

void BulletAnimatedMeshSceneNode::setTransitionTime(f32 Time)
{
}

void BulletAnimatedMeshSceneNode::animateJoints(bool CalculateAbsolutePositions)
{
}

void BulletAnimatedMeshSceneNode::setRenderFromIdentity(bool On)
{
}

ISceneNode* BulletAnimatedMeshSceneNode::clone(ISceneNode* newParent, ISceneManager* newManager)
{
    return nullptr;
}
