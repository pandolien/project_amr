# 공통 경고 옵션 (gcc/clang/MSVC)
function(amr_set_warnings target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4 /permissive-)
    else()
        target_compile_options(${target} PRIVATE
            -Wall -Wextra -Wpedantic -Wshadow -Wconversion)
    endif()
endfunction()
