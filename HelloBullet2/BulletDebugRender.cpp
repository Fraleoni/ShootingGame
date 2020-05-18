#include "BulletDebugRender.h"
#include "Graphics.h"
#include <iostream>

// Handy lambda for converting from irr::vector to btVector
auto toBtVector = [&](const vector3df& vec) -> btVector3
{
    btVector3 bt(vec.X, vec.Y, vec.Z);

    return bt;
};
// Handy lambda for converting from irr::vector to btVector
auto toIrrVector = [&](const btVector3& vec) -> vector3df
{
    vector3df irr(vec.getX(), vec.getY(), vec.getZ());

    return irr;
};

BulletDebugRender::BulletDebugRender() {
}

BulletDebugRender::BulletDebugRender(Graphics* g){ 
    m_graphics = g;
}

BulletDebugRender::~BulletDebugRender()
 {
}

void BulletDebugRender::drawLine(const btVector3& from, const btVector3& to, const btVector3& color)
{
    drawLine(from, to, color, color);
}

void BulletDebugRender::drawLine(const btVector3& from, const btVector3& to, const btVector3& fromColor, const btVector3& toColor)
{

    SColorf fromC;  fromC.set(fromColor[0], fromColor[1], fromColor[2], fromColor[3]);
    SColorf toC;    toC.set(toColor[0], toColor[1], toColor[2], toColor[3]);
    m_graphics->drawLine(toIrrVector(from), toIrrVector(to), fromC, toC);
}

void BulletDebugRender::drawSphere(const btVector3& p, btScalar radius, const btVector3& color)
{
//    m_graphics->drawSphere(const btVector3 & p, btScalar radius, const btVector3 & colorconst btVector3 & p, btScalar radius, const btVector3 & color);
   // std::cout << "*";
}

void BulletDebugRender::drawTriangle(const btVector3& a, const btVector3& b, const btVector3& c, const btVector3& color, btScalar alpha)
{
    //std::cout << "*";
}

void BulletDebugRender::drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB, btScalar distance, int lifeTime, const btVector3& color)
{
    //std::cout << "*";
}

void BulletDebugRender::reportErrorWarning(const char* warningString)
{
    //std::cout << "*";
}

void BulletDebugRender::draw3dText(const btVector3& location, const char* textString)
{
    //std::cout << "*";
}

void BulletDebugRender::setDebugMode(int debugMode)
{
    if (debugMode != m_debugMode) {
        m_debugMode = debugMode;
        cout << "Debugmode changed to "<< m_debugMode;
    }
}

void BulletDebugRender::setGraphics(Graphics* g) {
    m_graphics = g;
}
