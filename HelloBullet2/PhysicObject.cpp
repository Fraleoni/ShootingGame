#include <vector>
#include "PhysicObject.h"

using namespace irr;

btDynamicsWorld* PhysicObject::m_world;

PhysicObject::PhysicObject(irr::scene::ISceneNode* node, btCollisionShape* shape, irr::core::vector3df scale, 
    float scaleFactor, irr::core::vector3df pos, btScalar mass, btScalar restitution, irr::core::vector3df displacement) :
	m_node(node), m_collShape(shape),m_scale(scale),m_pos(pos),m_mass(mass),m_restitution(restitution), m_displacement(displacement)
{
    //m_rot = m_rot.makeIdentity();
    m_translationMatrix.setTranslation(m_displacement);
	//PhysicObject::collisionShapes.push_back(m_collShape);
    AudioDevicePtr audioDevice(audiere::OpenDevice());
    m_successSFX = OpenSoundEffect(audioDevice, orchestraFn.c_str(), MULTIPLE);
	btTransform t_groundTransform;
	t_groundTransform.setIdentity();
	t_groundTransform.setOrigin(btVector3(m_pos.X, m_pos.Y, m_pos.Z));
    //t_groundTransform.setRotation();
	m_isDynamic = (m_mass != 0.f);
	if (m_isDynamic)
		m_collShape->calculateLocalInertia(m_mass, m_localInertia);
	m_motionState = new btDefaultMotionState(t_groundTransform);
	btRigidBody::btRigidBodyConstructionInfo t_rbInfo(m_mass, m_motionState, m_collShape, m_localInertia);
	m_body = new btRigidBody(t_rbInfo);
    //m_body->setCollisionFlags(m_body->getCollisionFlags() & ~btCollisionObject::CF_STATIC_OBJECT);
	m_node->setScale(m_scale);
	m_node->setPosition(m_pos+m_displacement);
    m_node->setMaterialFlag(irr::video::EMF_FOG_ENABLE, true);
    m_body->setUserPointer((void*)(this));
    m_body->setRestitution(m_restitution);
    //btVector3 t_halfExtents(m_scale.X * scaleFactor, m_scale.Y * scaleFactor, m_scale.Z * scaleFactor);
    //m_collShape->setLocalScaling(t_halfExtents);

    /*if (m_collShape->getShapeType() == 0) {
        btVector3 t_halfExtents(m_scale.X * 0.5f, m_scale.Y * 0.5f, m_scale.Z * 0.5f);
        m_collShape->setLocalScaling(t_halfExtents);
    }
    else {
        btVector3 t_fullExtents(m_scale.X, m_scale.Y, m_scale.Z);
        m_collShape->setLocalScaling(t_fullExtents);
    }//*/
    m_world->addRigidBody(m_body);

}

void PhysicObject::setWorld(btDynamicsWorld* world) {
    m_world = world;
}

scene::ISceneNode* PhysicObject::getNode() {
    return m_node;
}

btRigidBody* PhysicObject::getBody() {
    return m_body;
}

void PhysicObject::setPos(irr::core::vector3df pos) {
    m_pos = pos;
    m_node->setPosition(m_pos+m_displacement);
    
    //reposition(btVector3(m_pos.X, m_pos.Y, m_pos.Z), btVector3(m_pos.X, m_pos.Y, m_pos.Z));
    btTransform t_groundTransform;
    t_groundTransform.setIdentity();
    t_groundTransform.setOrigin(btVector3(m_pos.X, m_pos.Y, m_pos.Z));
    m_body->setWorldTransform(t_groundTransform);
    m_motionState->setWorldTransform(t_groundTransform);
    m_body->activate();
    /*m_world->removeRigidBody(m_body);
    m_world->addRigidBody(m_body);*/
}

void PhysicObject::setLinearVelocity(btVector3 vel) {
    m_body->setLinearVelocity(vel);
    btTransform t_groundTransform;
    t_groundTransform.setIdentity();
    t_groundTransform.setOrigin(btVector3(m_pos.X, m_pos.Y, m_pos.Z));
    m_body->setWorldTransform(t_groundTransform);
    m_motionState->setWorldTransform(t_groundTransform);
    m_body->activate();
}

void PhysicObject::hit() {
    m_successSFX->play();
}

void PhysicObject::destroy()
{
    m_world->removeCollisionObject(m_body);
    delete m_body;
    m_node->remove();
    /*if (m_nodeFX)
        m_nodeFX->remove();//*/
    delete this;
}

void PhysicObject::synchPos(irr::core::vector3df pos, irr::core::vector3df rot) {
    core::matrix4 t_rotationMatrix = core::matrix4();
    t_rotationMatrix.setRotationDegrees(rot);
    
    
    m_node->setRotation(rot);
    m_translationMatrix.transformVect(pos);
    m_node->setPosition(pos);
}

void PhysicObject::teleport(btVector3 position, btQuaternion& orientation) const {

    btTransform transform;
    transform.setIdentity();

    transform.setOrigin(position);
    transform.setRotation(orientation);

    m_body->setWorldTransform(transform);
    m_body->getMotionState()->setWorldTransform(transform);

    m_body->setLinearVelocity(btVector3(0.0f, 0.0f, 0.0f));
    m_body->setAngularVelocity(btVector3(0.0f, 0.0f, 0.0f));
    m_body->clearForces();
}

core::vector3df PhysicObject::getPos() {
    return this->m_pos;
}

//Convert Irrlicht node to Bullet mesh
// from:  https://studiofreya.com/game-maker/bullet-physics/from-irrlicht-mesh-to-bullet-physics-mesh/
btRigidBody* PhysicObject::makeBulletMeshFromIrrlichtNode(const irr::scene::ISceneNode* node)
{
    using irr::core::vector2df;
    using irr::core::vector3df;
    using irr::scene::IMesh;
    using irr::scene::IMeshBuffer;
    using irr::scene::IMeshSceneNode;
    using irr::scene::IAnimatedMeshSceneNode;

    // Handy lambda for converting from irr::vector to btVector
    auto toBtVector = [&](const vector3df& vec) -> btVector3
    {
        btVector3 bt(vec.X, vec.Y, vec.Z);

        return bt;
    };

    irr::scene::ESCENE_NODE_TYPE type = node->getType();

    IAnimatedMeshSceneNode* meshnode = nullptr;
    btRigidBody* body = nullptr;

    switch (type)
    {
    case irr::scene::ESNT_ANIMATED_MESH:
    {
        meshnode = (IAnimatedMeshSceneNode*)node;
    }
    break;
    }

    if (meshnode)
    {
        const vector3df nodescale = meshnode->getScale();

        IMesh* mesh = meshnode->getMesh();
        const size_t buffercount = mesh->getMeshBufferCount();

        // Save position
        btVector3 position = toBtVector(meshnode->getPosition());

        // Save data here
        std::vector<irr::video::S3DVertex>    verticesList;
        std::vector<int>                  indicesList;

        for (size_t i = 0; i < buffercount; ++i)
        {
            // Current meshbuffer
            IMeshBuffer* buffer = mesh->getMeshBuffer(i);

            // EVT_STANDARD -> video::S3DVertex
            // EVT_2TCOORDS -> video::S3DVertex2TCoords
            // EVT_TANGENTS -> video::S3DVertexTangents
            const irr::video::E_VERTEX_TYPE vertexType = buffer->getVertexType();

            // EIT_16BIT
            // EIT_32BIT
            const irr::video::E_INDEX_TYPE  indexType = buffer->getIndexType();

            // Get working data
            const size_t numVerts = buffer->getVertexCount();
            const size_t numInd = buffer->getIndexCount();

            // Resize save buffers
            verticesList.resize(verticesList.size() + numVerts);
            indicesList.resize(indicesList.size() + numInd);

            void* vertices = buffer->getVertices();
            void* indices = buffer->getIndices();

            irr::video::S3DVertex* standard = reinterpret_cast<irr::video::S3DVertex*>(vertices);
            irr::video::S3DVertex2TCoords* two2coords = reinterpret_cast<irr::video::S3DVertex2TCoords*>(vertices);
            irr::video::S3DVertexTangents* tangents = reinterpret_cast<irr::video::S3DVertexTangents*>(vertices);

            int16_t* ind16 = reinterpret_cast<int16_t*>(indices);
            int32_t* ind32 = reinterpret_cast<int32_t*>(indices);

            for (size_t v = 0; v < numVerts; ++v)
            {
                auto& vert = verticesList[v];

                switch (vertexType)
                {
                case irr::video::EVT_STANDARD:
                {
                    const auto& irrv = standard[v];

                    vert = irrv;
                }
                break;
                case irr::video::EVT_2TCOORDS:
                {
                    const auto& irrv = two2coords[v];
                    (void)irrv;

                    // Not implemented
                }
                //break;
                case irr::video::EVT_TANGENTS:
                {
                    const auto& irrv = tangents[v];
                    (void)irrv;

                    // Not implemented
                }
                //break;
                //default:
                    //BOOST_ASSERT(0 && "unknown vertex type");
                }

            }

            for (size_t n = 0; n < numInd; ++n)
            {
                auto& index = indicesList[n];

                switch (indexType)
                {
                case irr::video::EIT_16BIT:
                {
                    index = ind16[n];
                }
                break;
                case irr::video::EIT_32BIT:
                {
                    index = ind32[n];
                }
                break;
                //default:
                    //BOOST_ASSERT(0 && "unkown index type");
                }

            }

        }

        // Make bullet rigid body
        if (!verticesList.empty() && !indicesList.empty())
        {
            // Working numbers
            const size_t numIndices = indicesList.size();
            const size_t numTriangles = numIndices / 3;

            // Error checking
            //BOOST_ASSERT(numTriangles * 3 == numIndices && "Number of indices does not make complete triangles");

            // Create triangles
            btTriangleMesh* btmesh = new btTriangleMesh();

            // Build btTriangleMesh
            for (size_t i = 0; i < numIndices; i += 3)
            {
                const btVector3& A = toBtVector(verticesList[indicesList[i + 0]].Pos);
                const btVector3& B = toBtVector(verticesList[indicesList[i + 1]].Pos);
                const btVector3& C = toBtVector(verticesList[indicesList[i + 2]].Pos);

                bool removeDuplicateVertices = true;
                btmesh->addTriangle(A, B, C, removeDuplicateVertices);
            }

            // Give it a default MotionState
            btTransform transform;
            transform.setIdentity();
            transform.setOrigin(position);
            btDefaultMotionState* motionState = new btDefaultMotionState(transform);

            // Create the shape
            btCollisionShape* btShape = new btBvhTriangleMeshShape(btmesh, true);
            btShape->setMargin(0.05f);

            // Create the rigid body object
            btScalar mass = 0.0f;
            body = new btRigidBody(mass, motionState, btShape);

        }

    }

    // Return Bullet rigid body
    return body;
}

btDynamicsWorld* PhysicObject::getWorld() {
    return m_world;
}


//**************  From: http://irrlicht.sourceforge.net/forum/viewtopic.php?f=4&t=52136#
// Given 2 euler rotations find a quaternion describing the relative rotation from the old to the new rotation
irr::core::quaternion getRelativeRotation(const irr::core::vector3df& oldRotationEulerDeg, const irr::core::vector3df& newRotationEulerDeg)
{
    irr::core::quaternion q1(oldRotationEulerDeg * irr::core::DEGTORAD);
    irr::core::quaternion q2(newRotationEulerDeg * irr::core::DEGTORAD);
    irr::core::quaternion qRel(q2 * q1.makeInverse());
    return qRel;
}

// Given an euler angle + a quaternion with a relative rotation do return an euler angle describing the combined absolute rotation
irr::core::vector3df applyRelativeRotation(const irr::core::vector3df& oldRotationEulerDeg, const irr::core::quaternion& relativeRotation)
{
    // Add relative rotation
    irr::core::quaternion qt(oldRotationEulerDeg * irr::core::DEGTORAD);
    irr::core::quaternion qt2(relativeRotation * qt);

    irr::core::vector3df rotateTarget;
    qt2.toEuler(rotateTarget);
    rotateTarget *= irr::core::RADTODEG;
    return rotateTarget;
}

irr::core::vector3df rotateAxesXYZToEuler(const irr::core::vector3df& oldRotation, const irr::core::vector3df& rotationAngles, bool useLocalAxes)
{
    irr::core::matrix4 transformation;
    transformation.setRotationDegrees(oldRotation);
    irr::core::vector3df axisX(1, 0, 0), axisY(0, 1, 0), axisZ(0, 0, 1);
    irr::core::matrix4 matRotX, matRotY, matRotZ;

    if (useLocalAxes)
    {
        transformation.rotateVect(axisX);
        transformation.rotateVect(axisY);
        transformation.rotateVect(axisZ);
    }

    matRotX.setRotationAxisRadians(rotationAngles.X * irr::core::DEGTORAD, axisX);
    matRotY.setRotationAxisRadians(rotationAngles.Y * irr::core::DEGTORAD, axisY);
    matRotZ.setRotationAxisRadians(rotationAngles.Z * irr::core::DEGTORAD, axisZ);

    irr::core::matrix4 newTransform = matRotX * matRotY * matRotZ * transformation;
    return newTransform.getRotationDegrees();
}

void PhysicObject::addFX(irr::scene::ISceneNode* node) {
    m_nodeFX = node;
    m_node->addChild(m_nodeFX);
    m_node->setPosition(core::vector3df(0.f, 0.f, 0.f));
}