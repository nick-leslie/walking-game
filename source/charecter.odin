package game

import jolt "../libs/jolt"
import rl "vendor:raylib"
import "core:log"
import "base:runtime"
import "core:math"
import "core:math/linalg"


Character :: struct {
	move_input:        Vec3,
	jump_requested:    bool,
	up:                Vec3,
	prev_position:     Vec3,
	position:          Vec3,
	look_yaw:          f32,
	look_pitch:        f32,
	physics_character: ^jolt.CharacterVirtual,
	look_input:        [2]f32,
	look_quat:         Quat,
}

charecter_physics_update :: proc() {
	g.character.prev_position = g.character.position
	jump_pressed := g.character.jump_requested

	// get up vector (and update it in the character struct just in case)
	up: Vec3; jolt.CharacterBase_GetUp(auto_cast g.character.physics_character, &up)
	g.character.up = up

	// A cheaper way to update the character's ground velocity, the platforms that the character is standing on may have changed velocity
	jolt.CharacterVirtual_UpdateGroundVelocity(g.character.physics_character)
	ground_velocity: Vec3; jolt.CharacterBase_GetGroundVelocity(auto_cast g.character.physics_character, &ground_velocity)

	current_velocity: Vec3; jolt.CharacterVirtual_GetLinearVelocity(g.character.physics_character, &current_velocity)
	current_vertical_velocity := linalg.dot(current_velocity, up) * up

	new_velocity: Vec3
	if jolt.CharacterBase_GetGroundState(auto_cast g.character.physics_character) == .OnGround {
		// Assume velocity of ground when on ground
		new_velocity = ground_velocity

		// Jump
		// moving_towards_ground := (current_vertical_velocity.y - ground_velocity.y) < 0.1
		if jump_pressed {
			new_velocity += JUMP_SPEED * up
		}
	} else {
		new_velocity = current_vertical_velocity
	}

	// Add gravity
	gravity: Vec3; jolt.PhysicsSystem_GetGravity(g.physicsManager.physicsSystem, &gravity)
	new_velocity += gravity * fixed_step

	input := linalg.mul(g.character.look_quat, g.character.move_input)
	input.y = 0
	input = linalg.normalize0(input)
	if jolt.CharacterBase_IsSupported(auto_cast g.character.physics_character) == true {
		if rl.IsKeyDown(.LEFT_SHIFT) {
			new_velocity += input * (MOVE_SPEED + SPRINT_MOD)
			log.debug("sprinting")
		} else {
			new_velocity += input * MOVE_SPEED
		}
	} else {
		// preserve horizontal velocity
		current_horizontal_velocity := current_velocity - current_vertical_velocity
		new_velocity += current_horizontal_velocity * AIR_DRAG
		new_velocity += input * AIR_MOVE_SPEED
	}
	// set the velocity to the character
	jolt.CharacterVirtual_SetLinearVelocity(g.character.physics_character, &new_velocity)

	extended_settings := jolt.ExtendedUpdateSettings {
		stickToFloorStepDown             = {0, -0.5, 0},
		walkStairsStepUp                 = {0, 0.4, 0},
		walkStairsMinStepForward         = 0.02,
		walkStairsStepForwardTest        = 0.30,
		walkStairsCosAngleForwardContact = math.cos(math.to_radians_f32(75.0)),
		walkStairsStepDownExtra          = {},
	}

	// update the character physics (btw there's also CharacterVirtual_ExtendedUpdate with stairs support)
	jolt.CharacterVirtual_ExtendedUpdate(
		g.character.physics_character,
		fixed_step,
		&extended_settings,
		PHYS_LAYER_MOVING,
		g.physicsManager.physicsSystem,
		nil,
		nil,
	)

	// read the new position into our structure
	jolt.CharacterVirtual_GetPosition(g.character.physics_character, &g.character.position)

	// if we're on the ground, try pushing currect contacts away
	if jolt.CharacterBase_GetGroundState(auto_cast g.character.physics_character) == .OnGround {
		for i in 0 ..< jolt.CharacterVirtual_GetNumActiveContacts(g.character.physics_character) {
			contact: jolt.CharacterVirtualContact; jolt.CharacterVirtual_GetActiveContact(g.character.physics_character, i, &contact)
			for &floor in g.chunks.floor {
				if contact.bodyB == floor do continue
			}
			if contact.motionTypeB == .Dynamic {
				PUSH_FORCE :: 100
				push_vector := -contact.contactNormal * PUSH_FORCE
				jolt.BodyInterface_AddImpulse2(
					g.physicsManager.bodyInterface,
					contact.bodyB,
					&push_vector,
					&contact.position,
				)
			}
		}
	}
}


add_character :: proc(pm: ^Physics_Manager, spawn_pos: Vec3) {
	spawn_pos := spawn_pos
	// capsule shape with origin at the bottom
	capsule_shape := jolt.RotatedTranslatedShape_Create(
		position = &{0, CHARACTER_CAPSULE_HALF_HEIGHT, 0},
		rotation = &QUAT_IDENTITY,
		// todo we can change the shape to be what ever we want
		shape = auto_cast jolt.CapsuleShape_Create(
			CHARACTER_CAPSULE_HALF_HEIGHT,
			CHARACTER_CAPSULE_RADIUS,
		),
	)

	settings: jolt.CharacterVirtualSettings
	jolt.CharacterVirtualSettings_Init(&settings)
	settings.base.shape = auto_cast capsule_shape
	settings.innerBodyShape = auto_cast capsule_shape // "inner shape" that actually participates in physics (e.g. reacts to raycast and stuff)
	settings.base.maxSlopeAngle = math.to_radians_f32(69.0)
	character := jolt.CharacterVirtual_Create(
		&settings,
		&spawn_pos,
		&QUAT_IDENTITY,
		0,
		pm.physicsSystem,
	)

	// use static var so the pointers survive
	@(static) listener_procs: jolt.CharacterContactListener_Procs
	listener_procs = {
		OnContactAdded = proc "c" (
			context_ptr: rawptr,
			character: ^jolt.CharacterVirtual,
			other_body_id: jolt.BodyID,
			_: jolt.SubShapeID,
			contact_point: ^Vec3,
			contact_normal: ^Vec3,
			contact_settings: ^jolt.CharacterContactSettings,
		) {
			for &floor in g.chunks.floor {
				if other_body_id == floor do return

			}

			context = (cast(^runtime.Context)context_ptr)^

			log.debugf("Contact added: %v", other_body_id)
		},
		OnContactPersisted = proc "c" (
			context_ptr: rawptr,
			character: ^jolt.CharacterVirtual,
			other_body_id: jolt.BodyID,
			_: jolt.SubShapeID,
			contact_point: ^Vec3,
			contact_normal: ^Vec3,
			contact_settings: ^jolt.CharacterContactSettings,
		) {
			for &floor in g.chunks.floor {
				if other_body_id == floor do return
			}

			context = (cast(^runtime.Context)context_ptr)^

			log.debugf("Contact persisted: %v", other_body_id)
		},
		OnContactRemoved = proc "c" (
			context_ptr: rawptr,
			character: ^jolt.CharacterVirtual,
			other_body_id: jolt.BodyID,
			_: jolt.SubShapeID,
		) {
			for &floor in g.chunks.floor {
				if other_body_id == floor do return

			}

			context = (cast(^runtime.Context)context_ptr)^

			log.debugf("Contact removed: %v", other_body_id)
		},
	}
	log.debug("before contact listener")
	listener := jolt.CharacterContactListener_Create(&g_context)
	log.debug("after contact listener")
	jolt.CharacterContactListener_SetProcs(&listener_procs)
	jolt.CharacterVirtual_SetListener(character, listener)
	g.character = {}
	g.character.physics_character = character
}
