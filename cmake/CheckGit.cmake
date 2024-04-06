function(check_git_write git_hash)
    file(WRITE ${CMAKE_BINARY_DIR}/git-state.txt ${git_hash})
endfunction()

function(check_git_read git_hash)
    if (EXISTS ${CMAKE_BINARY_DIR}/git-state.txt)
        file(STRINGS ${CMAKE_BINARY_DIR}/git-state.txt CONTENT)
        list(GET CONTENT 0 var)

        set(${git_hash} ${var} PARENT_SCOPE)
    endif ()
endfunction()


function(check_git_version)
    message("${CMAKE_SOURCE_DIR}")
    # Get the latest abbreviated commit hash of the working branch
    execute_process(
            COMMAND git log -1 --format=%h
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
            OUTPUT_VARIABLE GIT_HASH
            OUTPUT_STRIP_TRAILING_WHITESPACE
    )

    check_git_read(GIT_HASH_CACHE)

    if (NOT DEFINED GIT_HASH_CACHE)
        set(GIT_HASH_CACHE "INVALID")
    endif ()

    # Only update the git_version.cpp if the hash has changed. This will
    # prevent us from rebuilding the project more than we need to.
    if (NOT ${GIT_HASH} STREQUAL ${GIT_HASH_CACHE})
        # Set che GIT_HASH_CACHE variable the next build won't have
        # to regenerate the source file.
        check_git_write(${GIT_HASH})

        # Configure config file
        configure_file(
                "${CMAKE_SOURCE_DIR}/include/grstapse/config.hpp.in"
                "${CMAKE_BINARY_DIR}/include/grstapse/config.hpp"
                @ONLY
        )
    endif ()

endfunction()

check_git_version()