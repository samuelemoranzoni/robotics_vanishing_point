-- Minimal two-lane road scene for camera-based lane perception.
--
-- Run this in CoppeliaSim:
--   Modules > Developer tools > Commander > Commander
--   dofile('/Users/samuelemoranzoni/Desktop/usi/robotics/lab/project_new/scripts/create_minimal_two_lane_scene.lua')
--
-- The generated scene is saved here:
--   /Users/samuelemoranzoni/Desktop/usi/robotics/lab/project_new/scenes/minimal_two_lane_scene.ttt

sim = require 'sim'

local PROJECT_ROOT = '/Users/samuelemoranzoni/Desktop/usi/robotics/lab/project_new'
local BASE_WORKSPACE = '/Users/samuelemoranzoni/Desktop/usi/robotics/lab/robotics-lab-usi-robomaster'
local SCENE_PATH = PROJECT_ROOT .. '/scenes/minimal_two_lane_scene.ttt'

local ROAD_LENGTH =200.0
local LANE_WIDTH = 1.10
local ROAD_WIDTH = LANE_WIDTH * 2.0
local ROAD_TOP_Z = 0.04
local PAINT_Z = ROAD_TOP_Z + 0.010

-- This RoboMaster model uses a non-standard frame: local X is vertical.
-- This orientation makes the robot sit correctly on the road.
local ROBOT_ORIENTATION = {0, -math.pi / 2, 0}
local ROBOT_GROUND_CLEARANCE = 0.005
local ROBOT_START_X = -98.0
local ROBOT_START_Y = -LANE_WIDTH / 2.0

-- Static high camera used as a clean perception sensor.
-- Negative pitch points the vision sensor down toward the road.
local HIGH_CAMERA_POSITION = {ROBOT_START_X - 0.5, ROBOT_START_Y, 2.0}
local HIGH_CAMERA_ORIENTATION = {math.rad(0), math.rad(-58), math.rad(90)}

local createdObjects = {}

local function removeObjectOrModel(handle)
    if handle == nil or handle < 0 then
        return
    end
    local ok = pcall(sim.removeModel, handle)
    if not ok then
        pcall(sim.removeObject, handle)
    end
end

local function removeExisting(path)
    while true do
        local handle = sim.getObject(path, {noError = true})
        if handle == nil or handle < 0 then
            break
        end
        removeObjectOrModel(handle)
    end
end

local function setColor(handle, color)
    sim.setShapeColor(handle, '', sim.colorcomponent_ambient_diffuse, color)
end

local function setShapePhysics(handle, respondable)
    sim.setObjectInt32Param(handle, sim.shapeintparam_static, 1)
    sim.setObjectInt32Param(handle, sim.shapeintparam_respondable, respondable and 1 or 0)
end

local function cuboid(alias, size, position, color, yaw, respondable)
    local handle = sim.createPrimitiveShape(sim.primitiveshape_cuboid, size, 0)
    sim.setObjectAlias(handle, alias)
    sim.setObjectPosition(handle, position, sim.handle_world)
    sim.setObjectOrientation(handle, {0, 0, yaw or 0}, sim.handle_world)
    setColor(handle, color)
    setShapePhysics(handle, respondable == true)
    table.insert(createdObjects, handle)
    return handle
end

local function modelBottomOffsetZ(handle)
    local minX = sim.getObjectFloatParam(handle, sim.objfloatparam_modelbbox_min_x)
    local maxX = sim.getObjectFloatParam(handle, sim.objfloatparam_modelbbox_max_x)
    local minY = sim.getObjectFloatParam(handle, sim.objfloatparam_modelbbox_min_y)
    local maxY = sim.getObjectFloatParam(handle, sim.objfloatparam_modelbbox_max_y)
    local minZ = sim.getObjectFloatParam(handle, sim.objfloatparam_modelbbox_min_z)
    local maxZ = sim.getObjectFloatParam(handle, sim.objfloatparam_modelbbox_max_z)
    local matrix = sim.getObjectMatrix(handle, sim.handle_world)
    local bottom = math.huge

    for _, x in ipairs({minX, maxX}) do
        for _, y in ipairs({minY, maxY}) do
            for _, z in ipairs({minZ, maxZ}) do
                local p = sim.multiplyVector(matrix, {x, y, z})
                bottom = math.min(bottom, p[3])
            end
        end
    end

    return bottom
end

local function buildRoad()
    -- Dark road surface. The robot drives along the world X axis.
    cuboid('road_surface', {ROAD_LENGTH, ROAD_WIDTH, 0.08}, {0, 0, 0}, {0.03, 0.03, 0.03}, 0, true)

    -- Two-lane road = three continuous lane boundaries:
    -- left boundary, center divider, right boundary.
    for index, y in ipairs({-LANE_WIDTH, 0.0, LANE_WIDTH}) do
        cuboid(
            'white_lane_boundary_' .. index,
            {ROAD_LENGTH - 0.4, 0.055, 0.014},
            {0, y, PAINT_Z},
            {1.0, 1.0, 1.0},
            0,
            false
        )
    end

end

local function tryLoadRobot()
    removeExisting('/rm0')

    local candidateModels = {
        BASE_WORKSPACE .. '/src/robomaster_example/models/robomaster_ep_tof_v2.ttm',
        BASE_WORKSPACE .. '/src/robomaster_example/models/robomaster_ep_tof.ttm',
        sim.getStringParam(sim.stringparam_application_path) .. '/models/robots/mobile/RoboMasterEP.ttm',
    }

    for _, path in ipairs(candidateModels) do
        local ok, handle = pcall(sim.loadModel, path)
        if ok and handle and handle >= 0 then
            sim.setObjectAlias(handle, 'rm0')
            sim.setObjectPosition(handle, {0, 0, 0}, sim.handle_world)
            sim.setObjectOrientation(handle, ROBOT_ORIENTATION, sim.handle_world)

            local spawnZ = ROAD_TOP_Z - modelBottomOffsetZ(handle) + ROBOT_GROUND_CLEARANCE
            sim.setObjectPosition(handle, {ROBOT_START_X, ROBOT_START_Y, spawnZ}, sim.handle_world)
            sim.addLog(sim.verbosity_scriptinfos, 'Loaded RoboMaster model: ' .. path)
            return handle
        end
    end

    sim.addLog(sim.verbosity_warnings, 'Could not load a RoboMaster model automatically.')
    return -1
end

local function addRobotVisibilityMarker(robotHandle)
    if robotHandle == nil or robotHandle < 0 then
        return
    end

    -- Small visual-only marker above the robot.
    -- It makes rm0 easy to see from the external camera without changing the model physics.
    local marker = sim.createPrimitiveShape(sim.primitiveshape_cuboid, {0.24, 0.24, 0.055}, 0)
    sim.setObjectAlias(marker, 'robot_visibility_marker')
    sim.setObjectPosition(marker, {ROBOT_START_X, ROBOT_START_Y, ROAD_TOP_Z + 0.26}, sim.handle_world)
    sim.setObjectOrientation(marker, {0, 0, 0}, sim.handle_world)
    setColor(marker, {0.0, 0.85, 1.0})
    setShapePhysics(marker, false)

    -- keepInPlace=true preserves the world pose while making the marker follow rm0.
    sim.setObjectParent(marker, robotHandle, true)
    table.insert(createdObjects, marker)
end

local function addHighLaneCamera()
    -- A virtual camera placed above and behind the robot start pose.
    -- It is intentionally higher and tilted downward so the two lane boundaries
    -- and the center divider are clearly visible for a minimal perception setup.
    local options = 1 + 2 + 128 -- explicit handling + perspective + hide volume when not selected
    local intParams = {640, 360, 0, 0}
    local floatParams = {0.02, 40.0, math.rad(72), 0.1, 0.1, 0.1, 0, 0, 0, 0, 0}
    local camera = sim.createVisionSensor(options, intParams, floatParams)

    sim.setObjectAlias(camera, 'high_lane_camera')
    sim.setObjectPosition(camera, HIGH_CAMERA_POSITION, sim.handle_world)

    -- Oblique forward/down view. The blue frustum is the field of view:
    -- it should intersect the road, not point into the sky.
    sim.setObjectOrientation(camera, HIGH_CAMERA_ORIENTATION, sim.handle_world)

    table.insert(createdObjects, camera)
    return camera
end

local function setupSceneCamera()
    local camera = sim.getObject('/DefaultCamera', {noError = true})
    if camera ~= nil and camera >= 0 then
        -- External view for checking the whole road immediately after opening.
        sim.setObjectPosition(camera, {-10.0, -8.0, 7.0}, sim.handle_world)
        sim.setObjectOrientation(camera, {math.rad(60), 0, math.rad(-30)}, sim.handle_world)
        sim.setObjectFloatParam(camera, sim.camerafloatparam_far_clipping, 100.0)
        sim.setObjectFloatParam(camera, sim.camerafloatparam_near_clipping, 0.01)
    end
end

-- Start from a clean scene.
local defaultFloor = sim.getObject('/Floor', {noError = true})
if defaultFloor ~= nil and defaultFloor >= 0 then
    sim.removeObject(defaultFloor)
end

removeExisting('/road_surface')
removeExisting('/white_lane_boundary_1')
removeExisting('/white_lane_boundary_2')
removeExisting('/white_lane_boundary_3')
removeExisting('/high_lane_camera')
removeExisting('/robot_visibility_marker')

buildRoad()
local robotHandle = tryLoadRobot()
addRobotVisibilityMarker(robotHandle)
addHighLaneCamera()
setupSceneCamera()

for viewIndex = 0, 7 do
    pcall(sim.cameraFitToView, viewIndex, createdObjects, 0, 1.20)
end

sim.saveScene(SCENE_PATH)
sim.addLog(sim.verbosity_scriptinfos, 'Minimal two-lane scene saved at: ' .. SCENE_PATH)
