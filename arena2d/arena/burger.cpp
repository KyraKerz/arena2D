#include "burger.h" 
Burger::Burger(b2World * world): _lidarBuffer(0){
	b2BodyDef b;
	b.allowSleep = false;
	b.fixedRotation = false;
	b.type = b2_dynamicBody;
	b2FixtureDef fix;
	fix.filter.categoryBits = COLLIDE_CATEGORY_PLAYER;

	// main body
	b.position = b2Vec2(0, 0);
	_base = world->CreateBody(&b);
	b2PolygonShape shape;
	b2Vec2 verts[8];
	float dist = BURGER_BASE_NON_BEVEL/2.f;
	float dist2 = BURGER_BASE_SIZE/2.f;
	verts[0].Set(dist2, dist); 
	verts[1].Set(dist, dist2);
	verts[2].Set(-dist, dist2);
	verts[3].Set(-dist2, dist);
	verts[4].Set(-dist2, -dist);
	verts[5].Set(-dist, -dist2);
	verts[6].Set(dist, -dist2);
	verts[7].Set(dist2, -dist);
	shape.Set(verts, 8);
	fix.shape = &shape;
	fix.density = BURGER_BASE_DENSITY;
	_base->CreateFixture(&fix);
	// right wheel
	b2PolygonShape box_shape;
	fix.shape = &box_shape;
	fix.density = BURGER_WHEEL_DENSITY;
	float wheel_hw = BURGER_WHEEL_WIDTH/2.f;
	_wheelPosition[RIGHT].Set(BURGER_WHEEL_OFFSET, -BURGER_WHEEL_DISTANCE-wheel_hw);
	_wheelPosition[LEFT].Set(BURGER_WHEEL_OFFSET, BURGER_WHEEL_DISTANCE+wheel_hw);
	box_shape.SetAsBox(BURGER_WHEEL_RADIUS, wheel_hw, _wheelPosition[RIGHT], 0);
	_base->CreateFixture(&fix);

	// left wheel
	box_shape.SetAsBox(BURGER_WHEEL_RADIUS, wheel_hw, _wheelPosition[LEFT], 0);
	_base->CreateFixture(&fix);

	// safety radius sensor
	b2CircleShape circle_shape;
	circle_shape.m_radius = BURGER_SAFE_RADIUS*0.9;
	fix.shape = &circle_shape;
	fix.filter.categoryBits |= COLLIDE_CATEGORY_DONT_RENDER | COLLIDE_CATEGORY_PLAYER;
	fix.filter.maskBits = COLLIDE_CATEGORY_STAGE | COLLIDE_CATEGORY_GOAL;
	fix.isSensor = true;
	_safetyRadiusSensor = _base->CreateFixture(&fix);

	_contactCount = 0;

	_lidar = NULL;
	updateLidar();

	// generating buffer for laser data and trail
	if(_SETTINGS->video.enabled){
		// laser data
		glGenBuffers(1, &_lidarBuffer);
		_lidarBufferCount = 0;
		
		// trail
		glGenBuffers(1, &_trailBuffer);
		glBindBuffer(GL_ARRAY_BUFFER, _trailBuffer);
		glBufferData(GL_ARRAY_BUFFER, sizeof(float)*2*BURGER_TRAIL_BUFFER_SIZE, NULL, GL_DYNAMIC_DRAW);
		resetTrail();
	}
}

void Burger::updateLidar()
{
	// creating lidar scanner
	delete _lidar;
	const f_robotSettings & s = _SETTINGS->robot;
	_lidar = new LidarCast(	s.laser_num_samples, s.laser_max_distance,
							f_rad(s.laser_start_angle), f_rad(s.laser_end_angle),
							s.laser_noise, _base);
}

Burger::~Burger()
{
	if(_SETTINGS->video.enabled){
		glDeleteBuffers(1, &_lidarBuffer);
		glDeleteBuffers(1, &_trailBuffer);
	}

	// destroy lidar
	delete _lidar;

	// destroy body
	_base->GetWorld()->DestroyBody(_base);
}

void Burger::reset(const b2Vec2 & position)
{
	_base->SetLinearVelocity(b2Vec2_zero);
	_base->SetAngularVelocity(0);
	_base->SetTransform(position, f_frandomRange(0, 2*M_PI));
	_contactCount = 0;

	if(_SETTINGS->video.enabled)
		resetTrail();
}

void Burger::resetTrail()
{
	_trailVertexCount = 1;
	_lastTrailPosition = _base->GetTransform().p;
	glBindBuffer(GL_ARRAY_BUFFER, _trailBuffer);
	glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(float)*2, &_lastTrailPosition);
}

void Burger::updateTrail()
{
	if(_trailVertexCount >= BURGER_TRAIL_BUFFER_SIZE)// no more space for vertices
		return;

	b2Vec2 pos = _base->GetTransform().p;
	if((pos-_lastTrailPosition).Length() > BURGER_BASE_SIZE/2)// enough distance from last vertex
	{
		glBindBuffer(GL_ARRAY_BUFFER, _trailBuffer);
		glBufferSubData(GL_ARRAY_BUFFER, _trailVertexCount*sizeof(float)*2, sizeof(float)*2, &pos);
		_lastTrailPosition = pos;
		_trailVertexCount++;
	}
}

void Burger::getClosestObstacle(float & distance, float & angle)
{
	int i = _lidar->getClosestIndex();
	distance = _lidar->getDistances()[i];
	b2Vec2 p = _base->GetLocalPoint(_lidar->getPoints()[i]);
	b2Vec2 lidar_pos(BURGER_LIDAR_OFFSET, 0);
	p = p-lidar_pos;
	angle = f_deg(zVector2D::signedAngle(zVector2D(p.x, p.y), zVector2D(1, 0)));
}

void Burger::performAction(Action a)
{
	Twist t;
	const f_robotSettings & s = _SETTINGS->robot;
	switch(a)	
	{
	case FORWARD:
	{
		t.linear = s.forward_speed.linear;
		t.angular = s.forward_speed.angular;
	}break;
	case FORWARD_LEFT:
	{
		t.linear = s.left_speed.linear;
		t.angular = s.left_speed.angular;
	}break;
	case FORWARD_RIGHT:
	{
		t.linear = s.right_speed.linear;
		t.angular = s.right_speed.angular;
	}break;
	case FORWARD_STRONG_LEFT:
	{
		t.linear = s.strong_left_speed.linear;
		t.angular = s.strong_left_speed.angular;
	}break;
	case FORWARD_STRONG_RIGHT:
	{
		
		t.linear = s.strong_right_speed.linear;
		t.angular = s.strong_right_speed.angular;
	}break;
	case BACKWARD:
	{
		t.linear = s.backward_speed.linear;
		t.angular = s.backward_speed.angular;
	}break;
	case STOP:
	default:
	{
		t.linear = 0;
		t.angular = 0;
	}break;
	}

	performAction(t);
}


void Burger::scan()
{
	b2Transform t = _base->GetTransform();
	b2Vec2 center = _base->GetWorldPoint(b2Vec2(BURGER_LIDAR_OFFSET, 0)); 
	_lidar->scan(_base->GetWorld(), center, t.q.GetAngle());
}

void Burger::renderScan(bool area)
{
	_lidarBufferCount = _lidar->getNumSamples()+2;
	const b2Vec2 * samples = _lidar->getPointsWithCenter();
	glBindBuffer(GL_ARRAY_BUFFER, _lidarBuffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(b2Vec2)*_lidarBufferCount, samples, GL_STATIC_DRAW);

	_RENDERER->resetModelviewMatrix();
	glPointSize(BURGER_LIDAR_POINT_SIZE);
	glBindBuffer(GL_ARRAY_BUFFER, _lidarBuffer);
	GLint vertex_loc = Z_SHADER->getVertexLoc();
	glEnableVertexAttribArray(vertex_loc);
	glVertexAttribPointer(vertex_loc, 2, GL_FLOAT, GL_FALSE, 0, 0);
	Z_SHADER->setColor(zColor(BURGER_LIDAR_POINT_COLOR));
	glDrawArrays(GL_POINTS, 1, _lidarBufferCount-2);
	if(area){
		Z_SHADER->setColor(zColor(BURGER_LIDAR_AREA_COLOR));
		glDrawArrays(GL_TRIANGLE_FAN, 0, _lidarBufferCount);
	}

	// restore
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glDisableVertexAttribArray(vertex_loc);
	glPointSize(1);
}


void Burger::renderTrail()
{
	_RENDERER->resetModelviewMatrix();
	glLineWidth(BURGER_TRAIL_WIDTH);
	glBindBuffer(GL_ARRAY_BUFFER, _trailBuffer);
	GLint vertex_loc = Z_SHADER->getVertexLoc();
	glEnableVertexAttribArray(vertex_loc);
	glVertexAttribPointer(vertex_loc, 2, GL_FLOAT, GL_FALSE, 0, 0);
	Z_SHADER->setColor(zColor(BURGER_TRAIL_COLOR));
	glDrawArrays(GL_LINE_STRIP, 0, _trailVertexCount);

	// restore
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glDisableVertexAttribArray(vertex_loc);
	glLineWidth(1);
}

void Burger::performAction(const Twist & t)
{
	float wheel_distance = BURGER_WHEEL_DISTANCE+BURGER_WHEEL_WIDTH;
	float left_speed = t.linear - t.angular*wheel_distance/2.0f;
	float right_speed = t.linear + t.angular*wheel_distance/2.0f;
	float rad = _base->GetAngle();
	b2Vec2 dir(cos(rad), sin(rad));
	float mass = _base->GetMass();
	b2Vec2 wheel_pos(BURGER_WHEEL_OFFSET, wheel_distance/2.0f);
	b2Vec2 vel = _base->GetLinearVelocityFromLocalPoint(wheel_pos);
	_base->ApplyLinearImpulse(mass*(left_speed*dir-vel), _base->GetWorldPoint(wheel_pos), false);
	wheel_pos.y = -wheel_pos.y;
	vel = _base->GetLinearVelocityFromLocalPoint(wheel_pos);
	_base->ApplyLinearImpulse(mass*(right_speed*dir-vel), _base->GetWorldPoint(wheel_pos), false);
}
