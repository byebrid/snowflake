import vpython as vp
import datetime
import math
import random
print('RAN at {}'.format(datetime.datetime.now()))

rad = 1  # Radius of particles
num_inbound = 6  # Number of particles that are spawned around snowflake
start_dist = 100  # Distance particles start from origin
centre_speed = 0.01  # Speed at which particles approach centre


class Particle(vp.simple_sphere):
    def __init__(self, ** kwargs):
        """Instantiates a vpython sphere with the given kwargs."""
        self.steps = 0
        super().__init__(**kwargs)

    def touches(self, other):
        """If incoming particle touches stationary particle, returns True and
        places incoming particle on edge of stationary one.  Else, returns
        False.
        """
        dist = other.pos - self.pos
        sum_radii = self.radius + other.radius
        if vp.mag(dist) <= sum_radii:
            # Puts incoming particle on very edge of stationary particle.
            other.pos = self.pos + sum_radii * vp.norm(dist)
            return True
        else:
            return False

    def move(self):
        """Moves particle towards (0, 0, 0). Also moves randomly perpendicular
        to this trajectory.
        """
        self.steps += 1
        centre_vector = vp.norm(vp.vector(0, 0, 0) - self.pos)
        perp_vector = vp.rotate(
            centre_vector, angle=math.pi / 2)
        self.pos += centre_speed * centre_vector
        self.pos += 0.2 * random.uniform(-1, 1) * perp_vector


# Creating array to store all particles that have stopped moving.
stationary = []
stationary.append(Particle(pos=vp.vector(0, 0, 0), radius=rad))

# Creating array to store all currently moving particles
particles = []
for i in range(num_inbound):
    # Angle from positive x from which particle goes towards origin
    # angle = 2 * math.pi / num_inbound * i
    particles.append(
        Particle(radius=rad,
                 # angle=angle,
                 # pos=start_dist * vp.vector(math.cos(angle), math.sin(angle), 0))
                 pos=start_dist * vp.norm(vp.vector.random())
                 )
             )


max_steps=start_dist / centre_speed + 100
while True:
    # Making copy of particles array to avoid issues while iterating
    copy=particles
    # To ensure program remains fast as more particles are added
    if len(stationary) > 30:
        stationary=stationary[-20:]


    for particle in particles:
        particle.move()
        # If particle hasn't collided, then recreate it
        if particle.steps > max_steps:
            print("Particle took too long to collide.")
            copy.remove(particle)
            # _angle=particle.angle
            new=Particle(
                radius=rad,
                pos=start_dist * vp.norm(vp.vector.random())
                # angle=_angle,
                # pos=start_dist * \
                #     vp.vector(math.cos(_angle), math.sin(_angle), 0)
            )
            copy.append(new)
            particle.visible=False
            del particle
            continue
        # Check 'all' stationary particles to see if particle has collided
        for stat_particle in stationary:
            if particle.touches(stat_particle):
                stationary.append(particle)
                copy.remove(particle)
                # _angle=particle.angle
                new=Particle(
                    radius=rad,
                    pos=start_dist * vp.norm(vp.vector.random())
                    # angle=_angle,
                    # pos=start_dist * \
                    #     vp.vector(math.cos(_angle), math.sin(_angle), 0)
                )
                copy.append(new)
                break
    particles=copy

print("END")
