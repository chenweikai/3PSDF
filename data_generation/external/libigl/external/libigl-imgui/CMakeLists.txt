cmake_minimum_required(VERSION 3.1)

add_library(imgui
		../imgui/imgui.cpp
		../imgui/imgui.h
		../imgui/imgui_demo.cpp
		../imgui/imgui_draw.cpp
		../imgui/imgui_internal.h
		../imgui/imgui_widgets.cpp
		../imgui/imstb_rectpack.h
		../imgui/imstb_textedit.h
		../imgui/imstb_truetype.h
		../imgui/examples/imgui_impl_glfw.cpp
		../imgui/examples/imgui_impl_opengl3.cpp
)

target_include_directories(imgui PRIVATE ../imgui)
target_include_directories(imgui PUBLIC . .. ../imgui/examples)
target_compile_definitions(imgui PUBLIC -DIMGUI_IMPL_OPENGL_LOADER_GLAD)

target_link_libraries(imgui PRIVATE glad glfw)
