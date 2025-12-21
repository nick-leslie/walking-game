package game

import jolt "../libs/jolt"
import rl "vendor:raylib"
import "core:log"
import "core:strings"


rgb_to_gray :: proc(color: rl.Color) -> f32 {
	r_f := f32(color.r) / 255.0
	g_f := f32(color.g) / 255.0
	b_f := f32(color.b) / 255.0
	//     #define GRAY_VALUE(c) ((float)(c.r + c.g + c.b)/3.0f) rl gray value
	return 0.2126 * r_f + 0.7152 * g_f + 0.0722 * b_f
}

rgb_to_gray_ray :: proc(color: rl.Color) -> f32 {
	return f32(color.r + color.g + color.b) / 3.0
}

//todo add an offset xy for gen
generate_chunk :: proc(sample_count: u32, max_height: f32, cell_size: f32, chunk_offset: Vec3) {
	log.debug("in height map floor")
	height_map := rl.GenImagePerlinNoise(
		auto_cast sample_count,
		auto_cast sample_count,
		auto_cast chunk_offset.x * i32(sample_count - 1),
		auto_cast chunk_offset.z * i32(sample_count - 1),
		cell_size,
	)

	defer rl.UnloadImage(height_map)
	gen_chunk_from_image(height_map, max_height, cell_size, chunk_offset)
}


load_chunk_from_file :: proc(
	file_name: string,
	max_height: f32,
	cell_size: f32,
	chunk_offset: Vec3,
) {
	//cringe ass clone
	cstring_name := strings.clone_to_cstring(file_name)
	defer delete(cstring_name)
	height_map := rl.LoadImage(cstring_name)
	defer rl.UnloadImage(height_map)
	gen_chunk_from_image(height_map, max_height, cell_size, chunk_offset)

}


gen_chunk_from_image :: proc(
	height_map: rl.Image,
	max_height: f32,
	cell_size: f32,
	chunk_offset: Vec3,
) {
	if height_map.width != height_map.width {
		return
	}
	sample_count: u32 = u32(height_map.width)
	texture := rl.LoadTextureFromImage(height_map)

	points := make([dynamic]f32, sample_count * sample_count)
	// defer delete(points)
	// mat := make([dynamic]u8, sample_count * sample_count)
	// defer delete(mat)
	log.debug("generating points")


	chunk_world_size := auto_cast (f32(sample_count - 1) * cell_size)


	for y: u32 = 0; y < sample_count; y += 1 {
		for x: u32 = 0; x < sample_count; x += 1 {
			color := rl.GetImageColor(height_map, auto_cast x, auto_cast y)
			noise_val := rgb_to_gray(color)
			points[y * sample_count + x] = noise_val
		}
	}
	// log.debug(points[:])
	log.debug("about to create floor settings")
	floor_scale := Vec3{cell_size, max_height, cell_size}
	height_settings := jolt.HeightFieldShapeSettings_Create(
		samples = raw_data(points),
		offset = &{0, 0, 0},
		scale = &floor_scale,
		sampleCount = sample_count,
		materialIndices = nil,
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
	floor_body_id := jolt.BodyInterface_CreateAndAddBody(
		g.physicsManager.bodyInterface,
		floor_settings,
		.Activate,
	)
	jolt.BodyCreationSettings_Destroy(floor_settings)

	mesh := rl.GenMeshHeightmap(height_map, {chunk_world_size, max_height, chunk_world_size})

	model := rl.LoadModelFromMesh(mesh)
	model.materials[0].maps[0].texture = texture


	g.chunks.floor[g.chunk_load_index] = floor_body_id
	g.chunks.points[g.chunk_load_index] = points
	g.chunks.texture[g.chunk_load_index] = texture
	g.chunks.position[g.chunk_load_index] = floor_position
	g.chunks.scale[g.chunk_load_index] = floor_scale
	g.chunks.sample_size[g.chunk_load_index] = sample_count
	g.chunks.cell_size[g.chunk_load_index] = cell_size
	g.chunks.model[g.chunk_load_index] = model
	g.chunk_load_index += 1
	if g.chunk_load_index > MAX_LOADED_CHUNKS {
		g.chunk_load_index = 0
	}
}
