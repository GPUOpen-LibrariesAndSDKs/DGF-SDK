add_library(imgui STATIC)
TARGET_SOURCES(imgui
    PRIVATE
    include/imconfig.h
    include/imgui/imgui.h
    include/imgui/imgui_impl_dx12.h
    include/imgui/imgui_impl_win32.h
    include/imgui_internal.h
    include/imgui_stdlib.h
    include/imstb_rectpack.h
    include/imstb_textedit.h
    include/imstb_truetype.h
    
    src/imgui.cpp
    src/imgui_demo.cpp
    src/imgui_draw.cpp
    src/imgui_impl_dx12.cpp
    src/imgui_impl_win32.cpp
    src/imgui_stdlib.cpp
    src/imgui_tables.cpp
    src/imgui_widgets.cpp
    )
target_include_directories(imgui PUBLIC include PUBLIC include/imgui)