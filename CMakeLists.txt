cmake_minimum_required(VERSION 3.13)
project(lr35902_emu)

# Set Zig as the compiler
set(CMAKE_C_COMPILER zig cc)
set(CMAKE_CXX_COMPILER zig c++)

# Find Zig package
find_program(ZIG_EXECUTABLE zig)
if(NOT ZIG_EXECUTABLE)
    message(FATAL_ERROR "Zig compiler not found!")
endif()

# Set build directory for Zig
set(ZIG_BUILD_DIR ${CMAKE_BINARY_DIR}/zig-cache)

# Create custom target for building the Zig project
add_custom_target(lr35902_emu ALL
        COMMAND ${ZIG_EXECUTABLE} build
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Building LR35902 emulator with Zig"
)

# Installation rules
install(FILES
        ${CMAKE_SOURCE_DIR}/build/lr35902_emu
        DESTINATION bin
        PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
        GROUP_READ GROUP_EXECUTE
        WORLD_READ WORLD_EXECUTE
)

# Testing configuration
enable_testing()
add_test(NAME lr35902_emu_test
        COMMAND ${ZIG_EXECUTABLE} test
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)

# Add custom clean target
add_custom_target(clean-all
        COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_clean.cmake
)

# Generate the clean script
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/cmake_clean.cmake
        "file(REMOVE_RECURSE ${CMAKE_BINARY_DIR}/zig-cache)\n"
        "file(REMOVE_RECURSE ${CMAKE_BINARY_DIR}/zig-out)\n"
)

# Create a build.zig if it doesn't exist
if(NOT EXISTS "${CMAKE_SOURCE_DIR}/build.zig")
    file(WRITE "${CMAKE_SOURCE_DIR}/build.zig"
            "const std = @import(\"std\");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const exe = b.addExecutable(.{
        .name = \"lr35902_emu\",
        .root_source_file = b.addPath(\"src/main.zig\"),
        .target = target,
        .optimize = optimize,
    });

    b.installArtifact(exe);

    const run_cmd = b.addRunArtifact(exe);
    run_cmd.step.dependOn(b.getInstallStep());

    const run_step = b.step(\"run\", \"Run the emulator\");
    run_step.dependOn(&run_cmd.step);

    const unit_tests = b.addTest(.{
        .root_source_file = b.addPath(\"src/main.zig\"),
        .target = target,
        .optimize = optimize,
    });

    const run_unit_tests = b.addRunArtifact(unit_tests);
    const test_step = b.step(\"test\", \"Run unit tests\");
    test_step.dependOn(&run_unit_tests.step);
}"
    )
endif()

# Create project structure if it doesn't exist
file(MAKE_DIRECTORY ${CMAKE_SOURCE_DIR}/src)

# Optional: Define compile definitions
add_definitions(-DZIG_BUILD)

# Optional: Set build type specific flags
set(CMAKE_BUILD_TYPE_INIT "Release")
set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE_INIT} CACHE STRING
        "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel."
        FORCE)