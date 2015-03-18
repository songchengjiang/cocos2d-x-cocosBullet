#include "HelloWorldScene.h"

USING_NS_CC;

#define START_POS_X -0.5
#define START_POS_Y -2.5
#define START_POS_Z -0.5

#define ARRAY_SIZE_X 5
#define ARRAY_SIZE_Y 5
#define ARRAY_SIZE_Z 5

Scene* HelloWorld::createScene()
{
    // 'scene' is an autorelease object
    auto scene = Scene::create();
    
    // 'layer' is an autorelease object
    auto layer = HelloWorld::create();

    // add layer as a child to scene
    scene->addChild(layer);

    // return the scene
    return scene;
}

// on "init" you need to initialize your instance
bool HelloWorld::init()
{
    //////////////////////////////
    // 1. super init first
    if ( !Layer::init() )
    {
        return false;
    }
    
    Size visibleSize = Director::getInstance()->getVisibleSize();
    Vec2 origin = Director::getInstance()->getVisibleOrigin();

 //   /////////////////////////////
 //   // 2. add a menu item with "X" image, which is clicked to quit the program
 //   //    you may modify it.

 //   // add a "close" icon to exit the progress. it's an autorelease object
 //   auto closeItem = MenuItemImage::create(
 //                                          "CloseNormal.png",
 //                                          "CloseSelected.png",
 //                                          CC_CALLBACK_1(HelloWorld::menuCloseCallback, this));
 //   
	//closeItem->setPosition(Vec2(origin.x + visibleSize.width - closeItem->getContentSize().width/2 ,
 //                               origin.y + closeItem->getContentSize().height/2));

 //   // create menu, it's an autorelease object
 //   auto menu = Menu::create(closeItem, NULL);
 //   menu->setPosition(Vec2::ZERO);
 //   this->addChild(menu, 1);

 //   /////////////////////////////
 //   // 3. add your codes below...

    // add a label shows "Hello World"
    // create and initialize a label
    
	auto title = Label::createWithTTF("cocosBullet Demo", "fonts/Marker Felt.ttf", 24);
	title->setPosition(visibleSize.width / 2.0f, visibleSize.height - visibleSize.height * 0.05f);
	this->addChild(title);

    auto label = Label::createWithTTF("Reset", "fonts/Marker Felt.ttf", 24);
	auto menuItem1 = MenuItemLabel::create(label, [=](Ref *ref){
		for (auto iter : _physicSprite3DList){
			_dynamicsWorld->removeRigidBody(iter.first);
			this->removeChild(iter.second);
		}
		_physicSprite3DList.clear();

		for (auto iter : _bulletList){
			_dynamicsWorld->removeRigidBody(iter.first);
			this->removeChild(iter.second);
		}
		_bulletList.clear();

		initBoxs();
	});
	menuItem1->setAnchorPoint(Vec2::ZERO);
	menuItem1->setPosition(10.0f, visibleSize.height-100);
	auto menu = Menu::create(menuItem1, nullptr);
	menu->setPosition(Vec2::ZERO);
    // add the label as a child to this layer
    this->addChild(menu);

	auto listener = EventListenerTouchAllAtOnce::create();
	listener->onTouchesBegan = CC_CALLBACK_2(HelloWorld::onTouchesBegan, this);
	listener->onTouchesMoved = CC_CALLBACK_2(HelloWorld::onTouchesMoved, this);
	listener->onTouchesEnded = CC_CALLBACK_2(HelloWorld::onTouchesEnded, this);
	_eventDispatcher->addEventListenerWithSceneGraphPriority(listener, this);


	initPhysics();
    
	scheduleUpdate();

	_angle = 0.0f;
    return true;
}


void HelloWorld::menuCloseCallback(Ref* pSender)
{
#if (CC_TARGET_PLATFORM == CC_PLATFORM_WP8) || (CC_TARGET_PLATFORM == CC_PLATFORM_WINRT)
	MessageBox("You pressed the close button. Windows Store Apps do not implement a close button.","Alert");
    return;
#endif

    Director::getInstance()->end();

#if (CC_TARGET_PLATFORM == CC_PLATFORM_IOS)
    exit(0);
#endif
}

void HelloWorld::onTouchesBegan(const std::vector<Touch*>& touches, cocos2d::Event  *event)
{
	_needShootBox = true;
}

void HelloWorld::onTouchesMoved(const std::vector<Touch*>& touches, cocos2d::Event  *event)
{
	if (touches.size())
	{
		auto touch = touches[0];
		auto delta = touch->getDelta();

		_angle -= CC_DEGREES_TO_RADIANS(delta.x);
		_camera->setPosition3D(Vec3(50.0f * sinf(_angle), 25.0f, 50.0f * cosf(_angle)));
		_camera->lookAt(Vec3(0.0f, 0.0f, 0.0f), Vec3(0.0f, 1.0f, 0.0f));
		_needShootBox = false;
	}
}

void HelloWorld::onTouchesEnded(const std::vector<Touch*>& touches, cocos2d::Event  *event)
{
	if (!_needShootBox) return;
	if (!touches.empty())
	{
		auto location = touches[0]->getLocationInView();

		Vec3 nearP(location.x, location.y, -1.0f), farP(location.x, location.y, 1.0f);
		nearP = _camera->unproject(nearP);
		farP = _camera->unproject(farP);
		Vec3 dir(farP - nearP);
		shootBox(_camera->getPosition3D() + dir * 10.0f);
	}
}

void HelloWorld::initPhysics()
{
	_collisionConfiguration = new btDefaultCollisionConfiguration();
	_dispatcher = new	btCollisionDispatcher(_collisionConfiguration);
	_broadphase = new btDbvtBroadphase();
	_solver = new btSequentialImpulseConstraintSolver;
	_dynamicsWorld = new btDiscreteDynamicsWorld(_dispatcher,_broadphase,_solver,_collisionConfiguration);
	_dynamicsWorld->setGravity(btVector3(0.0f, -10.0f, 0.0f));
	initScenes();
}

void HelloWorld::updatePhysics(float delta)
{
	static float physicTime = 0.0f;
	if (0.03f < physicTime){
		_dynamicsWorld->stepSimulation(physicTime);
		physicTime = 0.0f;
	}
	physicTime += delta;
}

void HelloWorld::exitPhysics()
{
	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	for (int i = _dynamicsWorld->getNumCollisionObjects() - 1; i >= 0 ;i--)
	{
		btCollisionObject* obj = _dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		//auto cs = body->getCollisionShape();
		//if (cs)
		//	delete cs;

		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		_dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}

	delete _dynamicsWorld;
	delete _solver;
	delete _broadphase;
	delete _dispatcher;
	delete _collisionConfiguration;
}

void HelloWorld::update( float delta )
{
	updatePhysics(delta);

	for (auto iter : _physicSprite3DList){
		if (iter.first && iter.first->getMotionState())
		{
			if (!iter.first->wantsSleeping())
			{
				btTransform trans;
				float m[16];
				iter.first->getMotionState()->getWorldTransform(trans);
				trans.getOpenGLMatrix(m);
				Mat4 mat(m);
				Mat4 sclMat;
				Mat4::createScale(iter.second->getScaleX(), iter.second->getScaleY(), iter.second->getScaleZ(), &sclMat);
				iter.second->setNodeToParentTransform(mat * sclMat);
				iter.second->setColor(Color3B(255, 255, 255));
				iter.second->setOpacity(255);
			}else
			{
				iter.second->setColor(Color3B(0, 255, 0));
				iter.second->setOpacity(128);
			}

		}
	}

	for (auto iter : _bulletList){
		if (iter.first && iter.first->getMotionState())
		{
			if (!iter.first->wantsSleeping())
			{
				btTransform trans;
				float m[16];
				iter.first->getMotionState()->getWorldTransform(trans);
				trans.getOpenGLMatrix(m);
				Mat4 mat(m);
				Mat4 sclMat;
				Mat4::createScale(iter.second->getScaleX(), iter.second->getScaleY(), iter.second->getScaleZ(), &sclMat);
				iter.second->setNodeToParentTransform(mat * sclMat);
			}
		}
	}
}

void HelloWorld::initScenes()
{
	Size visibleSize = Director::getInstance()->getVisibleSize();
	_camera = Camera::createPerspective(30.0f, visibleSize.width / visibleSize.height, 1.0f, 1000.0f);
	_camera->setCameraFlag(CameraFlag::USER1);
	_camera->setPosition3D(Vec3(0.0f, 25.0f, 50.0f));
	_camera->lookAt(Vec3::ZERO);
	this->addChild(_camera);

	float groundSize = 50.0f;
	btBoxShape* groundShape = new btBoxShape(btVector3(btScalar(groundSize * 0.5f),btScalar(groundSize * 0.5f),btScalar(groundSize * 0.5f)));
	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,-groundSize * 0.5f,0));
	//We can also use DemoApplication::localCreateRigidBody, but for clarity it is provided here:
	{
		btScalar mass(0.);
		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);
		btVector3 localInertia(0,0,0);
		if (isDynamic)
			groundShape->calculateLocalInertia(mass,localInertia);
		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,groundShape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);
		//add the body to the dynamics world
		_dynamicsWorld->addRigidBody(body);

		auto sprite3d = Sprite3D::create("box.c3t");
		sprite3d->setTexture("grid.jpg");
		sprite3d->setCameraMask((unsigned short)CameraFlag::USER1);
		sprite3d->setScale(groundSize);
		sprite3d->setPosition3D(Vec3(0.0f, -groundSize * 0.5f, 0.0f));
		this->addChild(sprite3d);
	}

	initBoxs();
}

void HelloWorld::shootBox( const cocos2d::Vec3 &des )
{
	float boxSize = 0.5f;
	if (_dynamicsWorld)
	{
		float mass = 1.f;
		btTransform startTransform;
		startTransform.setIdentity();
		Vec3 camPos = _camera->getPosition3D();
		startTransform.setOrigin(btVector3(camPos.x, camPos.y, camPos.z));

		btBoxShape* box = new btBoxShape(btVector3(boxSize * 0.5f,boxSize * 0.5f,boxSize * 0.5f));
		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		btVector3 localInertia(0,0,0);
		box->calculateLocalInertia(mass,localInertia);
		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,box,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);
		//add the body to the dynamics world
		_dynamicsWorld->addRigidBody(body);

		body->setLinearFactor(btVector3(1,1,1));
		//body->setRestitution(1);

		btVector3 linVel(des.x-camPos.x,des.y-camPos.y,des.z-camPos.z);
		linVel.normalize();
		linVel*=100.0f;

		body->getWorldTransform().setOrigin(btVector3(camPos.x, camPos.y, camPos.z));
		body->getWorldTransform().setRotation(btQuaternion(0,0,0,1));
		body->setLinearVelocity(linVel);
		body->setAngularVelocity(btVector3(0,0,0));
		body->setCcdMotionThreshold(0.5);
		body->setCcdSweptSphereRadius(0.4f);


		auto sprite3d = Sprite3D::create("box.c3t");
		sprite3d->setTexture("cocos.jpg");
		sprite3d->setScale(boxSize);
		sprite3d->setPosition3D(camPos);
		sprite3d->setCameraMask((unsigned short)CameraFlag::USER1);
		this->addChild(sprite3d);
		_bulletList.push_back(std::make_pair(body, sprite3d));
	}
}

void HelloWorld::initBoxs()
{
	float boxSize = 1.0f;
	{
		//create a few dynamic rigidbodies
		// Re-using the same collision is better for memory usage and performance

		btBoxShape* colShape = new btBoxShape(btVector3(boxSize * 0.5f,boxSize * 0.5f,boxSize * 0.5f));
		//btCollisionShape* colShape = new btSphereShape(btScalar(1.));

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar	mass(1.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			colShape->calculateLocalInertia(mass,localInertia);

		float start_x = START_POS_X - ARRAY_SIZE_X/2;
		float start_y = START_POS_Y;
		float start_z = START_POS_Z - ARRAY_SIZE_Z/2;

		for (int k=0;k<ARRAY_SIZE_Y;k++)
		{
			for (int i=0;i<ARRAY_SIZE_X;i++)
			{
				for(int j = 0;j<ARRAY_SIZE_Z;j++)
				{
					float x = 1.0*i + start_x;
					float y = 5.0+1.0*k + start_y;
					float z = 1.0*j + start_z;
					startTransform.setOrigin(btVector3(
						btScalar(x),
						btScalar(y),
						btScalar(z)));


					//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
					btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
					btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,colShape,localInertia);
					btRigidBody* body = new btRigidBody(rbInfo);
					_dynamicsWorld->addRigidBody(body);


					auto sprite3d = Sprite3D::create("box.c3t");
					sprite3d->setCameraMask((unsigned short)CameraFlag::USER1);
					sprite3d->setScale(boxSize);
					sprite3d->setPosition3D(Vec3(x, y, z));
					this->addChild(sprite3d);

					_physicSprite3DList.push_back(std::make_pair(body, sprite3d));
				}
			}
		}
	}
}

HelloWorld::~HelloWorld()
{
	exitPhysics();
}
