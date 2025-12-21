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

import "core:fmt"
import "core:math"
// import "core:math/linalg"
import jolt "../libs/jolt"
import "base:runtime"
import "core:log"
import "core:math/linalg"
import rl "vendor:raylib"
import rlgl "vendor:raylib/rlgl"
// import "core:math/noise"

PIXEL_WINDOW_HEIGHT :: 180


PHYS_LAYER_MOVING :: jolt.ObjectLayer(0)
PHYS_LAYER_NON_MOVING :: jolt.ObjectLayer(1)
PHYS_BROAD_LAYER_MOVING :: jolt.BroadPhaseLayer(0)
PHYS_BROAD_LAYER_NON_MOVING :: jolt.BroadPhaseLayer(1)

CHARACTER_CAPSULE_HALF_HEIGHT: f32 : 1
CHARACTER_CAPSULE_RADIUS: f32 : 0.3
MAX_LOADED_CHUNKS :: 9
SCREEN_WIDTH :: 1280
SCREEN_HEIGHT :: 720
SHADER_COUNT :: 1

Game_Memory :: struct {
	run:              bool,
	physicsManager:   Physics_Manager,
	character:        Character,
	boxes:            [dynamic]Box,
	chunks:           Chunk,
	render_texture:   rl.RenderTexture,
	chunk_load_index: int,
	shaders:          [SHADER_COUNT]rl.Shader,
	render_width:     int,
	render_height:    int,
}
//todo merge the floor meshes
// todo do we want array of structs
Chunk :: struct {
	floor:       [MAX_LOADED_CHUNKS]jolt.BodyID,
	points:      [MAX_LOADED_CHUNKS][dynamic]f32,
	texture:     [MAX_LOADED_CHUNKS]rl.Texture,
	position:    [MAX_LOADED_CHUNKS]Vec3,
	scale:       [MAX_LOADED_CHUNKS]Vec3,
	sample_size: [MAX_LOADED_CHUNKS]u32,
	cell_size:   [MAX_LOADED_CHUNKS]f32,
	model:       [MAX_LOADED_CHUNKS]rl.Model,
}

Physics_Manager :: struct {
	jobSystem:                     ^jolt.JobSystem,
	physicsSystem:                 ^jolt.PhysicsSystem,
	objectLayerPairFilter:         ^jolt.ObjectLayerPairFilter,
	broadPhaseLayerFilter:         ^jolt.BroadPhaseLayerInterface,
	objectVsBroadPhaseLayerFilter: ^jolt.ObjectVsBroadPhaseLayerFilter,
	bodyInterface:                 ^jolt.BodyInterface,
	fixed_update_accumulator:      f32,
}

Box :: struct {
	// for interpolating physics
	prev_position: Vec3,
	prev_rotation: Quat,
	position:      Vec3,
	rotation:      Quat,
	extent:        Vec3,
	color:         rl.Color,
	body_id:       jolt.BodyID,
}


Vec3 :: [3]f32
Quat :: quaternion128

QUAT_IDENTITY: Quat = 1
VEC3_ZERO: Vec3 = 0

//todo insert this into charecter
MOVE_SPEED :: 10
SPRINT_MOD :: 10
AIR_MOVE_SPEED :: 0.5
AIR_DRAG :: 0.9
JUMP_SPEED :: 20
LOOK_SENSITIVITY :: 0.2
CAMERA_DISTANCE :: 10


fixed_step: f32 = 1.0 / 60.0 // do we need this here or should we put this in the update


g: ^Game_Memory
g_context: runtime.Context // should we store this

update :: proc() {
	g.render_width = int(rl.GetRenderWidth())
	g.render_height = int(rl.GetRenderHeight())
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
	g.character.look_quat = linalg.quaternion_from_pitch_yaw_roll(
		linalg.to_radians(g.character.look_pitch),
		linalg.to_radians(g.character.look_yaw),
		0,
	)
}

physics_update :: proc() {
	charecter_physics_update()

	for &box in g.boxes {
		box.prev_position = box.position
		box.prev_rotation = box.rotation
		jolt.BodyInterface_GetPositionAndRotation(
			g.physicsManager.bodyInterface,
			box.body_id,
			&box.position,
			&box.rotation,
		)
	}

	// update normal physics
	jolt.PhysicsSystem_Update(
		g.physicsManager.physicsSystem,
		fixed_step,
		1,
		g.physicsManager.jobSystem,
	)
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
			rl.DrawCapsule(
				g.character.position,
				g.character.position + g.character.up * CHARACTER_CAPSULE_HALF_HEIGHT * 2,
				CHARACTER_CAPSULE_RADIUS,
				16,
				8,
				rl.ORANGE,
			)
			fixed_interpolation_delta := g.physicsManager.fixed_update_accumulator / fixed_step
			for &box in g.boxes {
				pos := linalg.lerp(box.prev_position, box.position, fixed_interpolation_delta)
				rot := linalg.quaternion_slerp(
					box.prev_rotation,
					box.rotation,
					fixed_interpolation_delta,
				)
				angle, axis := linalg.angle_axis_from_quaternion(rot)

				rlgl.PushMatrix()
				rlgl.Translatef(pos.x, pos.y, pos.z)
				rlgl.Rotatef(linalg.to_degrees(angle), axis.x, axis.y, axis.z)
				rl.DrawCubeV(0, box.extent * 2, box.color)
				rl.DrawCubeWiresV(0, box.extent * 2, rl.BLACK)
				rlgl.PopMatrix()
				// log.debug(g.floor.position)
			}
			for i := 0; i < MAX_LOADED_CHUNKS; i += 1 {
				rl.DrawModel(
					g.chunks.model[i],
					{g.chunks.position[i].x, g.chunks.position[i].y, g.chunks.position[i].z},
					1,
					rl.RED,
				)

			}
		}
	}
	{
		rl.BeginDrawing()
		defer rl.EndDrawing()
		{
			// rl.BeginShaderMode(g.shaders[0])g
			// defer rl.EndShaderMode()
			rl.DrawTextureRec(
				g.render_texture.texture,
				{0, 0, f32(g.render_texture.texture.width), f32(-g.render_texture.texture.height)},
				{0, 0},
				rl.WHITE,
			)
		}
		// DrawTextureRec(target.texture, (Rectangle){ 0, 0, (float)target.texture.width, (float)-target.texture.height }, (Vector2){ 0, 0 }, WHITE);
		rl.DrawText(
			fmt.ctprintf(
				"X:%f Y:%f Z:%f",
				g.character.position.x,
				g.character.position.y,
				g.character.position.z,
			),
			0,
			0,
			20,
			rl.BLACK,
		)
		rl.DrawFPS(0, 20)
		//todo add a 2d ui camera

	}
}

spawn_3d_cam :: proc() -> rl.Camera3D {
	look_target := g.character.position + g.character.up * 2
	return rl.Camera3D {
		position   = look_target + linalg.mul(g.character.look_quat, Vec3{0, 0, CAMERA_DISTANCE}),
		target     = look_target, // target 0,0
		up         = g.character.up,
		fovy       = 45.0,
		projection = rl.CameraProjection.PERSPECTIVE,
	}
}



add_floor :: proc(pm: ^Physics_Manager) -> jolt.BodyID {
	floor_extent := Vec3{100, 0.05, 100}
	floot_position := Vec3{0, -0.05, 0}
	floor_shape := jolt.BoxShape_Create(&floor_extent, 0)
	defer jolt.Shape_Destroy(auto_cast floor_shape)
	floor_settings := jolt.BodyCreationSettings_Create3(
		shape = auto_cast floor_shape,
		position = &floot_position,
		rotation = &QUAT_IDENTITY,
		motionType = .Static,
		objectLayer = PHYS_LAYER_NON_MOVING,
	)
	floor_body_id := jolt.BodyInterface_CreateAndAddBody(
		pm.bodyInterface,
		floor_settings,
		.Activate,
	)
	jolt.BodyCreationSettings_Destroy(floor_settings)
	return floor_body_id
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
	box.body_id = jolt.BodyInterface_CreateAndAddBody(
		g.physicsManager.bodyInterface,
		box_settings,
		.Activate,
	)
	jolt.BodyCreationSettings_Destroy(box_settings)

	box.prev_position = box.position
	box.prev_rotation = box.rotation

	append(&g.boxes, box)
}



@(export)
game_init :: proc() {
	g = new(Game_Memory)
	g.run = true
	g.physicsManager = create_physics_mannager()
	// g.floor = add_floor(&g.physicsManager)
	// load_chunk_from_file("Mountain Range.png",300,5,{0,0,0})
	generate_chunk(1000, 300, 5, {0, 0, 0})
	generate_chunk(1000, 300, 5, {1, 0, 0})
	generate_chunk(1000, 300, 5, {0, 0, 1})
	generate_chunk(1000, 300, 5, {1, 0, 1})
	generate_chunk(1000, 300, 5, {-1, 0, 0})
	generate_chunk(1000, 300, 5, {0, 0, -1})
	generate_chunk(1000, 300, 5, {-1, 0, -1})
	generate_chunk(1000, 300, 5, {-1, 0, 1})
	generate_chunk(1000, 300, 5, {1, 0, -1})
	g.render_texture = rl.LoadRenderTexture(SCREEN_WIDTH, SCREEN_HEIGHT)
	g.shaders[0] = rl.LoadShader("pixlizer", "assets/shaders/pixlizer.fs")
	width_index := rl.GetShaderLocation(g.shaders[0], "renderWidth")
	heigth_index := rl.GetShaderLocation(g.shaders[0], "renderHeight")
	g.render_width = int(rl.GetRenderWidth())
	g.render_height = int(rl.GetRenderHeight())
	rl.SetShaderValue(g.shaders[0], width_index, &g.render_width, rl.ShaderUniformDataType.FLOAT)
	rl.SetShaderValue(g.shaders[0], heigth_index, &g.render_height, rl.ShaderUniformDataType.FLOAT)

	for i := 0; i < 20; i += 1 {
		// add_box({ position = { 0,    0.75, -3   }, extent = 0.75, rotation = 1, color = rl.RED })
		// add_box({ position = { 0.75, 2.5,  -3   }, extent = 0.5,  rotation = 1, color = rl.BLUE })
		add_box({position = {0.25, 300, -2.5}, extent = 0.25, rotation = 1, color = rl.GREEN})
		add_box(
			{
				position = {0.25, 300, -2.5},
				extent = {1, 0.25, 0.5},
				rotation = 1,
				color = rl.PURPLE,
			},
		)
	}
	add_character(&g.physicsManager, {0, 500, 0})
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
	rl.UnloadShader(g.shaders[0])
	free(g)
}

unload_chunk :: proc(chunks: ^Chunk) {
	for i := 0; i < MAX_LOADED_CHUNKS; i += 1 {
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
	rl.SetConfigFlags({.WINDOW_RESIZABLE,}) // .VSYNC_HINT
	rl.InitWindow(1280, 720, "Odin + Raylib + Hot Reload template!")
	rl.DisableCursor()

	rl.SetWindowPosition(200, 200)
	rl.SetTargetFPS(10000)
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
