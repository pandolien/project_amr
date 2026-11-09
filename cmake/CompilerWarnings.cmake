# 공통 경고 옵션 (gcc/clang)
function(amr_set_warnings target)
    target_compile_options(${target} PRIVATE
        -Wall -Wextra -Wpedantic -Wshadow -Wconversion)
endfunction()
