require "chipmunk"

gravity = Chipmunk.v(0, -100)

space = CP.space_new()
CP.space_set_gravity(space, gravity)

ground = CP.segment_shape_new(CP.space_get_static_body(space), Chipmunk.v(-20, 5), Chipmunk.v(20, -5), 0.0)
CP.shape_set_friction(ground, 1.0)
CP.space_add_shape(space, ground)

radius = 5.0
mass = 1.0

moment = CP.moment_for_circle(mass, 0.0, radius, Chipmunk.v(0, 0))

ball_body = CP.space_add_body(space, CP.body_new(mass, moment))
CP.body_set_position(ball_body, Chipmunk.v(0, 15))

ball_shape = CP.space_add_shape(space, CP.circle_shape_new(ball_body, radius, Chipmunk.v(0, 0)))
CP.shape_set_friction(ball_shape, 0.7)

time_step = 1.0/60.0
time = 0.0
while time < 2
  pos = CP.body_get_position(ball_body)
  vel = CP.body_get_velocity(ball_body)
  printf(
    "Time is %5.2f. ball_body is at (%5.2f, %5.2f). It's velocity is (%5.2f, %5.2f)\n",
    time, pos.x, pos.y, vel.x, vel.y
  )
  
  CP.space_step(space, time_step)
  
  time += time_step
end

CP.shape_free(ball_shape)
CP.body_free(ball_body)
CP.shape_free(ground)
CP.space_free(space)
