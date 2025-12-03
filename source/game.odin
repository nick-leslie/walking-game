/*
This file is the starting point of your game.

Some important procedures are:
- game_init_window: Opens the window
- game_init: Sets up the game state
- game_update: Run once per frame
- game_should_close: For stopping your game when close button is pressed
- game_shutdown: Shuts down game and frees memory
- game_shutdown_window: Closes window

The procs above are used regardless if you compile using the `build_release`
script or the `build_hot_reload` script. However, in the hot reload case, the
contents of this file is compiled as part of `build/hot_reload/game.dll` (or
.dylib/.so on mac/linux). In the hot reload cases some other procedures are
also used in order to facilitate the hot reload functionality:

- game_memory: Run just before a hot reload. That way game_hot_reload.exe has a
	pointer to the game's memory that it can hand to the new game DLL.
- game_hot_reloaded: Run after a hot reload so that the `g` global
	variable can be set to whatever pointer it was in the old DLL.

NOTE: When compiled as part of `build_release`, `build_debug` or `build_web`
then this whole package is just treated as a normal Odin package. No DLL is
created.
*/

package game

import "core:math"
import "core:fmt"
// import "core:math/linalg"
import rl "vendor:raylib"
import rlgl "vendor:raylib/rlgl"
import jolt "../libs/jolt"
import "base:runtime"
import "core:log"
import "core:math/linalg"
// import "core:math/noise"

PIXEL_WINDOW_HEIGHT :: 180


PHYS_LAYER_MOVING :: jolt.ObjectLayer(0)
PHYS_LAYER_NON_MOVING :: jolt.ObjectLayer(1)
PHYS_BROAD_LAYER_MOVING :: jolt.BroadPhaseLayer(0)
PHYS_BROAD_LAYER_NON_MOVING :: jolt.BroadPhaseLayer(1)

CHARACTER_CAPSULE_HALF_HEIGHT : f32 : 1
CHARACTER_CAPSULE_RADIUS : f32 : 0.3
MAX_LOADED_CHUNKS :: 9
SCREEN_WIDTH ::1280
SCREEN_HEIGHT :: 720


Game_Memory :: struct {
    run:bool,
    physicsManager:Physics_Manager,
    character:Character,
    boxes: [dynamic]Box,
    chunks:Chunk,
    render_texture:rl.RenderTexture,
    chunk_load_index:int,
}
//todo merge the floor meshes
// todo do we want array of structs
Chunk :: struct {
    floor: [MAX_LOADED_CHUNKS]jolt.BodyID,
    points:[MAX_LOADED_CHUNKS][dynamic]f32,
    texture:[MAX_LOADED_CHUNKS]rl.Texture,
    position:[MAX_LOADED_CHUNKS]Vec3,
    scale:[MAX_LOADED_CHUNKS]Vec3,
    sample_size:[MAX_LOADED_CHUNKS]u32,
    cell_size:[MAX_LOADED_CHUNKS]f32,
    model:[MAX_LOADED_CHUNKS]rl.Model,
}

Physics_Manager :: struct {
    jobSystem:^jolt.JobSystem,
    physicsSystem:^jolt.PhysicsSystem,
    objectLayerPairFilter:^jolt.ObjectLayerPairFilter,
    broadPhaseLayerFilter: ^jolt.BroadPhaseLayerInterface,
    objectVsBroadPhaseLayerFilter:^jolt.ObjectVsBroadPhaseLayerFilter,
    bodyInterface:^jolt.BodyInterface,
   	fixed_update_accumulator: f32,
}

Box :: struct {
    // for interpolating physics
    prev_position: Vec3,
    prev_rotation: Quat,

    position: Vec3,
    rotation: Quat,
    extent: Vec3,
    color: rl.Color,
    body_id: jolt.BodyID,
}


Vec3 :: [3]f32
Quat :: quaternion128

QUAT_IDENTITY: Quat = 1
VEC3_ZERO: Vec3 = 0

//todo insert this into charecter
MOVE_SPEED       :: 10
SPRINT_MOD       :: 10
AIR_MOVE_SPEED   :: 0.5
AIR_DRAG         :: 0.9
JUMP_SPEED       :: 20
LOOK_SENSITIVITY :: 0.2
CAMERA_DISTANCE  :: 10


fixed_step: f32 = 1.0 / 60.0 // do we need this here or should we put this in the update
Character :: struct {
    move_input: Vec3,
    jump_requested:bool,
    up: Vec3,
    prev_position: Vec3,
    position: Vec3,
    look_yaw: f32,
    look_pitch: f32,
    physics_character:^jolt.CharacterVirtual,
    look_input:[2]f32,
    look_quat:Quat,
}

g: ^Game_Memory
g_context: runtime.Context // should we store this

update :: proc() {
    //reset movement controls could put this in a function
    g.character.move_input = {} // reset move every frame
    g.character.jump_requested = false
    g.character.look_input = {}
    g.character.look_quat = {}
   	if rl.IsKeyPressed(.ESCAPE) {
		g.run = false
	}
    if rl.IsKeyDown(.W) do g.character.move_input.z = -1
    else if rl.IsKeyDown(.S) do g.character.move_input.z = 1
    if rl.IsKeyDown(.A) do g.character.move_input.x = -1
    else if rl.IsKeyDown(.D) do g.character.move_input.x = 1
    if !g.character.jump_requested do g.character.jump_requested = rl.IsKeyPressed(.SPACE)

    g.character.look_input = rl.GetMouseDelta() * LOOK_SENSITIVITY
    g.character.look_yaw = math.wrap(g.character.look_yaw - g.character.look_input.x, 360)
    g.character.look_pitch = math.clamp(g.character.look_pitch - g.character.look_input.y, -89, 89)
    g.character.look_quat = linalg.quaternion_from_pitch_yaw_roll(linalg.to_radians(g.character.look_pitch), linalg.to_radians(g.character.look_yaw), 0)
}

physics_update :: proc() {
    charecter_physics_update()

    for &box in g.boxes {
        box.prev_position = box.position
        box.prev_rotation = box.rotation
        jolt.BodyInterface_GetPositionAndRotation(g.physicsManager.bodyInterface, box.body_id, &box.position, &box.rotation)
    }

    // update normal physics
    jolt.PhysicsSystem_Update(g.physicsManager.physicsSystem, fixed_step, 1, g.physicsManager.jobSystem)
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
        moving_towards_ground := (current_vertical_velocity.y - ground_velocity.y) < 0.1
        if jump_pressed && moving_towards_ground {
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
    log.debug(jolt.CharacterBase_IsSupported(auto_cast g.character.physics_character))
    if jolt.CharacterBase_IsSupported(auto_cast g.character.physics_character) == true {
        if rl.IsKeyDown(.LEFT_SHIFT) {
            new_velocity += input * (MOVE_SPEED + SPRINT_MOD)
            log.debug("sprinting")
        }  else {
            new_velocity += input * MOVE_SPEED
        }
    } else {
        log.debug("in air?")
        // preserve horizontal velocity
        current_horizontal_velocity := current_velocity - current_vertical_velocity
        new_velocity += current_horizontal_velocity * AIR_DRAG
        new_velocity += input * AIR_MOVE_SPEED
    }
    // set the velocity to the character
    jolt.CharacterVirtual_SetLinearVelocity(g.character.physics_character, &new_velocity)

    // update the character physics (btw there's also CharacterVirtual_ExtendedUpdate with stairs support)
    jolt.CharacterVirtual_Update(g.character.physics_character, fixed_step, PHYS_LAYER_MOVING, g.physicsManager.physicsSystem, nil, nil)

    // read the new position into our structure
    jolt.CharacterVirtual_GetPosition(g.character.physics_character, &g.character.position)

    // if we're on the ground, try pushing currect contacts away
    if jolt.CharacterBase_GetGroundState(auto_cast g.character.physics_character) == .OnGround {
        for i in 0..<jolt.CharacterVirtual_GetNumActiveContacts(g.character.physics_character) {
            contact:jolt.CharacterVirtualContact; jolt.CharacterVirtual_GetActiveContact(g.character.physics_character, i, &contact)
            for &floor in g.chunks.floor {
                if contact.bodyB == floor do continue
            }
            if contact.motionTypeB == .Dynamic {
                PUSH_FORCE :: 100
                push_vector := -contact.contactNormal * PUSH_FORCE
                jolt.BodyInterface_AddImpulse2(g.physicsManager.bodyInterface, contact.bodyB, &push_vector, &contact.position)
            }
        }
    }
}

draw :: proc() {
	{
    	rl.BeginTextureMode(g.render_texture)
        defer rl.EndTextureMode()
    	rl.ClearBackground(rl.BLUE)
     {
    	rl.BeginMode3D(spawn_3d_cam())
        defer rl.EndMode3D()
        rl.DrawGrid(100, 1)
        rl.DrawCapsule(g.character.position, g.character.position + g.character.up * CHARACTER_CAPSULE_HALF_HEIGHT * 2, CHARACTER_CAPSULE_RADIUS, 16, 8, rl.ORANGE)
        fixed_interpolation_delta := g.physicsManager.fixed_update_accumulator / fixed_step
        for &box in g.boxes {
            pos := linalg.lerp(box.prev_position, box.position, fixed_interpolation_delta)
            rot := linalg.quaternion_slerp(box.prev_rotation, box.rotation, fixed_interpolation_delta)
            angle, axis := linalg.angle_axis_from_quaternion(rot)

            rlgl.PushMatrix()
            rlgl.Translatef(pos.x, pos.y, pos.z)
            rlgl.Rotatef(linalg.to_degrees(angle), axis.x, axis.y, axis.z)
            rl.DrawCubeV(0, box.extent * 2, box.color)
            rl.DrawCubeWiresV(0, box.extent * 2, rl.BLACK)
            rlgl.PopMatrix()
            // log.debug(g.floor.position)
        }
        for i := 0; i<MAX_LOADED_CHUNKS;i+=1 {
            rl.DrawModel(g.chunks.model[i],{g.chunks.position[i].x,g.chunks.position[i].y,g.chunks.position[i].z},1,rl.RED)

        }
     }
	}
    {
    	rl.BeginDrawing()
    	defer rl.EndDrawing()
        // DrawTextureRec(target.texture, (Rectangle){ 0, 0, (float)target.texture.width, (float)-target.texture.height }, (Vector2){ 0, 0 }, WHITE);
        rl.DrawTextureRec(
            g.render_texture.texture,
            {0,0,f32(g.render_texture.texture.width),f32(-g.render_texture.texture.height)},
            {0,0},
            rl.WHITE,
        )
    	rl.DrawText(fmt.ctprintf("X:%f Y:%f Z:%f",g.character.position.x,g.character.position.y,g.character.position.z),0,0,20,rl.BLACK)
    	rl.DrawFPS(0,20)
    	//todo add a 2d ui camera

    }
}

spawn_3d_cam :: proc() -> rl.Camera3D {
    look_target := g.character.position + g.character.up * 2
    return rl.Camera3D {
        position= look_target + linalg.mul(g.character.look_quat, Vec3 {0,0,CAMERA_DISTANCE}),
        target = look_target, // target 0,0
        up=g.character.up,
        fovy = 45.0,
        projection = rl.CameraProjection.PERSPECTIVE,
    }
}

create_physics_mannager :: proc() -> Physics_Manager {
    ok := jolt.Init()
    log.debug(ok)
    assert(ok == true, "Failed to init Jolt Physics")
    g_context = context
    jolt.SetTraceHandler(proc "c" (message: cstring) {
        context = g_context
        log.debugf("JOLT: %v", message)
    })
    jobSystem := jolt.JobSystemThreadPool_Create(nil)

    object_layer_pair_filter := jolt.ObjectLayerPairFilterTable_Create(2)
    jolt.ObjectLayerPairFilterTable_EnableCollision(object_layer_pair_filter, PHYS_LAYER_MOVING, PHYS_LAYER_NON_MOVING)
    jolt.ObjectLayerPairFilterTable_EnableCollision(object_layer_pair_filter, PHYS_LAYER_MOVING, PHYS_LAYER_MOVING)

    broad_phase_layer_interface := jolt.BroadPhaseLayerInterfaceTable_Create(2, 2)
    jolt.BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer(broad_phase_layer_interface, PHYS_LAYER_MOVING, PHYS_BROAD_LAYER_MOVING)
    jolt.BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer(broad_phase_layer_interface, PHYS_LAYER_NON_MOVING, PHYS_BROAD_LAYER_NON_MOVING)

    object_vs_broad_phase_layer_filter := jolt.ObjectVsBroadPhaseLayerFilterTable_Create(broad_phase_layer_interface, 2, object_layer_pair_filter, 2)
    physics_system := jolt.PhysicsSystem_Create(&{
        maxBodies = 65536,
        numBodyMutexes = 0,
        maxBodyPairs = 65536,
        maxContactConstraints = 65536,
        broadPhaseLayerInterface = broad_phase_layer_interface,
        objectLayerPairFilter = object_layer_pair_filter,
        objectVsBroadPhaseLayerFilter = object_vs_broad_phase_layer_filter,
    })

    g_body_iface := jolt.PhysicsSystem_GetBodyInterface(physics_system)


    manager := Physics_Manager  {
        jobSystem=jobSystem,
        objectLayerPairFilter=object_layer_pair_filter,
        broadPhaseLayerFilter=broad_phase_layer_interface,
        objectVsBroadPhaseLayerFilter=object_vs_broad_phase_layer_filter,
        bodyInterface=g_body_iface,
        physicsSystem=physics_system,
    }
    return manager
}

destroy_physics_mannager :: proc(physicsManager:^Physics_Manager) {
    jolt.JobSystem_Destroy(physicsManager.jobSystem)
    jolt.PhysicsSystem_Destroy(physicsManager.physicsSystem)
    jolt.Shutdown()
}

add_floor :: proc(pm:^Physics_Manager) -> jolt.BodyID {
    floor_extent := Vec3 {100, 0.05, 100}
    floot_position := Vec3 {0, -0.05, 0}
    floor_shape := jolt.BoxShape_Create(&floor_extent, 0)
    defer jolt.Shape_Destroy(auto_cast floor_shape)
    floor_settings := jolt.BodyCreationSettings_Create3(
        shape = auto_cast floor_shape,
        position = &floot_position,
        rotation = &QUAT_IDENTITY,
        motionType = .Static,
        objectLayer = PHYS_LAYER_NON_MOVING,
    )
    floor_body_id := jolt.BodyInterface_CreateAndAddBody(pm.bodyInterface, floor_settings, .Activate)
    jolt.BodyCreationSettings_Destroy(floor_settings)
    return floor_body_id
}

rgb_to_gray :: proc(color:rl.Color) -> f32 {
    r_f := f32(color.r) / 255.0
    g_f := f32(color.g) / 255.0
    b_f := f32(color.b) / 255.0
    //     #define GRAY_VALUE(c) ((float)(c.r + c.g + c.b)/3.0f) rl gray value
    return 0.2126*r_f + 0.7152*g_f + 0.0722*b_f
}

rgb_to_gray_ray :: proc(color:rl.Color) -> f32 {
    return (f32(color.r + color.g+ color.b) / 3.0)
}

//todo add an offset xy for gen
generate_chunk :: proc(sample_count:u32,max_height:f32,cell_size:f32,chunk_offset:Vec3){
    log.debug("in height map floor")
    points := make([dynamic]f32 , sample_count*sample_count)
    // defer delete(points)
    mat: = make([dynamic]u8 , sample_count*sample_count)
    defer delete(mat)
    log.debug("generating points")


    chunk_world_size :=auto_cast (f32(sample_count-1)*cell_size)

    height_map := rl.GenImagePerlinNoise(
        auto_cast sample_count,
        auto_cast sample_count,
        auto_cast chunk_offset.x * i32(sample_count - 1),
        auto_cast chunk_offset.z * i32(sample_count - 1),
        cell_size,
    )

    defer rl.UnloadImage(height_map)
    texture := rl.LoadTextureFromImage(height_map)
    for y:u32 =0;y<sample_count; y+=1{
        for x:u32 =0;x<sample_count; x+=1{
            color := rl.GetImageColor(height_map,auto_cast x,auto_cast y)
            noise_val := rgb_to_gray(color)
            points[y*sample_count+x] = noise_val
            mat[y*sample_count+x] = 0 // default mat
        }
    }
    // log.debug(points[:])
    log.debug("about to create floor settings")

    floor_scale := Vec3{cell_size,max_height,cell_size}
    height_settings := jolt.HeightFieldShapeSettings_Create(
        samples=raw_data(points),
        offset=&{0,0,0},
        scale=&floor_scale,
        sampleCount=sample_count,
        materialIndices=raw_data(mat),
    )
    log.debug("created height settings")

    floor_position := Vec3 {
        (chunk_offset.x * chunk_world_size),
        0,
        (chunk_offset.z * chunk_world_size),
    }

    floor_shape := jolt.HeightFieldShapeSettings_CreateShape(height_settings)
    defer jolt.Shape_Destroy(auto_cast floor_shape)

    floor_settings := jolt.BodyCreationSettings_Create3(
        shape = auto_cast floor_shape,
        position = &floor_position,
        rotation = &QUAT_IDENTITY,
        motionType = .Static,
        objectLayer = PHYS_LAYER_NON_MOVING,
    )

    log.debug("creating body")
    floor_body_id := jolt.BodyInterface_CreateAndAddBody(g.physicsManager.bodyInterface, floor_settings, .Activate)
    jolt.BodyCreationSettings_Destroy(floor_settings)

    mesh := rl.GenMeshHeightmap(height_map, {
        chunk_world_size,
        max_height,
        chunk_world_size,
    })

    model := rl.LoadModelFromMesh(mesh)
    model.materials[0].maps[0].texture = texture


    g.chunks.floor[g.chunk_load_index]=floor_body_id
    g.chunks.points[g.chunk_load_index]  =points
    g.chunks.texture[g.chunk_load_index] =texture
    g.chunks.position[g.chunk_load_index]= floor_position
    g.chunks.scale[g.chunk_load_index]   = floor_scale
    g.chunks.sample_size[g.chunk_load_index]=sample_count
    g.chunks.cell_size[g.chunk_load_index]=cell_size
    g.chunks.model[g.chunk_load_index]=model
    g.chunk_load_index +=1
    if g.chunk_load_index > MAX_LOADED_CHUNKS {
        g.chunk_load_index = 0
    }

}

add_box :: proc(box: Box) {
    box := box

    box_shape := jolt.BoxShape_Create(&box.extent, 0)
    defer jolt.Shape_Destroy(auto_cast box_shape)
    box_settings := jolt.BodyCreationSettings_Create3(
        shape = auto_cast box_shape,
        position = &box.position,
        rotation = &box.rotation,
        motionType = .Dynamic,
        objectLayer = PHYS_LAYER_MOVING,
    )
    box.body_id = jolt.BodyInterface_CreateAndAddBody(g.physicsManager.bodyInterface, box_settings, .Activate)
    jolt.BodyCreationSettings_Destroy(box_settings)

    box.prev_position = box.position
    box.prev_rotation = box.rotation

    append(&g.boxes, box)
}


add_character :: proc(pm:^Physics_Manager,spawn_pos:Vec3) {
    spawn_pos:=spawn_pos
    // capsule shape with origin at the bottom
    capsule_shape := jolt.RotatedTranslatedShape_Create(
        position = &{ 0, CHARACTER_CAPSULE_HALF_HEIGHT, 0 },
        rotation = &QUAT_IDENTITY,
        // todo we can change the shape to be what ever we want
        shape = auto_cast jolt.CapsuleShape_Create(CHARACTER_CAPSULE_HALF_HEIGHT, CHARACTER_CAPSULE_RADIUS),
    )

    settings: jolt.CharacterVirtualSettings; jolt.CharacterVirtualSettings_Init(&settings)
    settings.base.shape = auto_cast capsule_shape
    settings.innerBodyShape = auto_cast capsule_shape // "inner shape" that actually participates in physics (e.g. reacts to raycast and stuff)

    character := jolt.CharacterVirtual_Create(&settings, &spawn_pos, &QUAT_IDENTITY, 0, pm.physicsSystem)

    // use static var so the pointers survive
    @static listener_procs: jolt.CharacterContactListener_Procs
    listener_procs = {
        OnContactAdded = proc "c" (context_ptr: rawptr, character: ^jolt.CharacterVirtual, other_body_id: jolt.BodyID, _: jolt.SubShapeID, contact_point: ^Vec3, contact_normal: ^Vec3, contact_settings: ^jolt.CharacterContactSettings) {
            for &floor in g.chunks.floor {
                if other_body_id == floor do return

            }

            context = (cast(^runtime.Context)context_ptr)^

            log.debugf("Contact added: %v", other_body_id)
        },
        OnContactPersisted = proc "c" (context_ptr: rawptr, character: ^jolt.CharacterVirtual, other_body_id: jolt.BodyID, _: jolt.SubShapeID, contact_point: ^Vec3, contact_normal: ^Vec3, contact_settings: ^jolt.CharacterContactSettings) {
            for &floor in g.chunks.floor {
                if other_body_id == floor do return
            }

            context = (cast(^runtime.Context)context_ptr)^

            log.debugf("Contact persisted: %v", other_body_id)
        },
        OnContactRemoved = proc "c" (context_ptr: rawptr, character: ^jolt.CharacterVirtual, other_body_id: jolt.BodyID, _: jolt.SubShapeID) {
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

@(export)
game_init :: proc() {
	g = new(Game_Memory)
	g.run = true
	g.physicsManager = create_physics_mannager()
	// g.floor = add_floor(&g.physicsManager)
	generate_chunk(700,300,10,{0,0,0})
	generate_chunk(700,300,10,{1,0,0})
	generate_chunk(700,300,10,{0,0,1})
	generate_chunk(700,300,10,{1,0,1})
	generate_chunk(700,300,10,{-1,0,0})
	generate_chunk(700,300,10,{0,0,-1})
	generate_chunk(700,300,10,{-1,0,-1})
	generate_chunk(700,300,10,{-1,0,1})
	generate_chunk(700,300,10,{1,0,-1})
	g.render_texture = rl.LoadRenderTexture(SCREEN_WIDTH, SCREEN_HEIGHT  )


	for i := 0; i < 20;i +=1 {
        // add_box({ position = { 0,    0.75, -3   }, extent = 0.75, rotation = 1, color = rl.RED })
        // add_box({ position = { 0.75, 2.5,  -3   }, extent = 0.5,  rotation = 1, color = rl.BLUE })
        add_box({ position = { 0.25, 300,    -2.5 }, extent = 0.25, rotation = 1, color = rl.GREEN })
        add_box({ position = { 0.25, 300,    -2.5 }, extent = {1, 0.25, 0.5}, rotation = 1, color = rl.PURPLE })
	}
	add_character(&g.physicsManager,{0,500,0})
	log.debug("We made it out of add charecter")
	game_hot_reloaded(g)
}
@(export)
game_update :: proc() {
	update()
    dt := rl.GetFrameTime()
    g.physicsManager.fixed_update_accumulator += dt
    for g.physicsManager.fixed_update_accumulator >= fixed_step {
     	g.physicsManager.fixed_update_accumulator -= fixed_step
        physics_update()
    }
	draw()

	// Everything on tracking allocator is valid until end-of-frame.
	free_all(context.temp_allocator)
}
@(export)
game_shutdown :: proc() {
    // log.destroy_console_logger(context.logger,allocator=context.allocator)
    destroy_physics_mannager(&g.physicsManager)
    delete(g.boxes)
    unload_chunk(&g.chunks)
	free(g)
}

unload_chunk :: proc(chunks:^Chunk) {
    for i := 0;i<MAX_LOADED_CHUNKS;i+=1 {
        delete(chunks.points[i])
        rl.UnloadModel(chunks.model[i])

    }
    // delete(floor.points)
    //todo causing leaks
    // rl.UnloadModel(floor.height_map_model)
    // img:= rl.LoadImageFromTexture(g.floor.height_map_texture)
    // rl.ExportImage(img,"./heightmap.png")
    // rl.UnloadImage(img)
    // rl.UnloadTexture(g.floor.height_map_texture)
}

@(export)
game_init_window :: proc() {
	rl.SetConfigFlags({.WINDOW_RESIZABLE, .VSYNC_HINT})
	rl.InitWindow(1280, 720, "Odin + Raylib + Hot Reload template!")
	rl.DisableCursor()

	rl.SetWindowPosition(200, 200)
	rl.SetTargetFPS(1000)
	rl.SetExitKey(nil)
}




@(export)
game_should_run :: proc() -> bool {
	when ODIN_OS != .JS {
		// Never run this proc in browser. It contains a 16 ms sleep on web!
		if rl.WindowShouldClose() {
			return false
		}
	}

	return g.run
}


@(export)
game_shutdown_window :: proc() {
	rl.CloseWindow()
}

@(export)
game_memory :: proc() -> rawptr {
	return g
}

@(export)
game_memory_size :: proc() -> int {
	return size_of(Game_Memory)
}

@(export)
game_hot_reloaded :: proc(mem: rawptr) {
	g = (^Game_Memory)(mem)

	// Here you can also set your own global variables. A good idea is to make
	// your global variables into pointers that point to something inside `g`.
}

@(export)
game_force_reload :: proc() -> bool {
	return rl.IsKeyPressed(.F5)
}

@(export)
game_force_restart :: proc() -> bool {
	return rl.IsKeyPressed(.F6)
}

// In a web build, this is called when browser changes size. Remove the
// `rl.SetWindowSize` call if you don't want a resizable game.
game_parent_window_size_changed :: proc(w, h: int) {
	rl.SetWindowSize(i32(w), i32(h))
}
