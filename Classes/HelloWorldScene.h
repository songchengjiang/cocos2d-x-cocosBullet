#ifndef __HELLOWORLD_SCENE_H__
#define __HELLOWORLD_SCENE_H__

#include "cocos2d.h"
#include "external/bullet/include/btBulletDynamicsCommon.h"

class HelloWorld : public cocos2d::Layer
{
public:
    // there's no 'id' in cpp, so we recommend returning the class instance pointer
    static cocos2d::Scene* createScene();
	~HelloWorld();

    // Here's a difference. Method 'init' in cocos2d-x returns bool, instead of returning 'id' in cocos2d-iphone
    virtual bool init();
    
    // a selector callback
    void menuCloseCallback(cocos2d::Ref* pSender);
    
    // implement the "static create()" method manually
    CREATE_FUNC(HelloWorld);

	virtual void update(float delta) override;

	void onTouchesBegan(const std::vector<cocos2d::Touch*>& touches, cocos2d::Event  *event);
	void onTouchesMoved(const std::vector<cocos2d::Touch*>& touches, cocos2d::Event  *event);
	void onTouchesEnded(const std::vector<cocos2d::Touch*>& touches, cocos2d::Event  *event);

private:

	void initPhysics();
	void initScenes();
	void initBoxs();
	void updatePhysics(float delta);
	void exitPhysics();
	void shootBox(const cocos2d::Vec3 &des);

private:

	btDefaultCollisionConfiguration *_collisionConfiguration;
	btCollisionDispatcher *_dispatcher;
	btDbvtBroadphase *_broadphase;
	btSequentialImpulseConstraintSolver *_solver;
	btDiscreteDynamicsWorld *_dynamicsWorld;

	cocos2d::Camera *_camera;
	float _angle;
	std::vector<std::pair<btRigidBody*, cocos2d::Sprite3D*> > _physicSprite3DList;
	std::vector<std::pair<btRigidBody*, cocos2d::Sprite3D*> > _bulletList;

	bool _needShootBox;
};

#endif // __HELLOWORLD_SCENE_H__
