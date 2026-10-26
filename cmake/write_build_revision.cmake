if(NOT DEFINED SOURCE_DIR OR NOT DEFINED OUTPUT_FILE)
 message(FATAL_ERROR "SOURCE_DIR and OUTPUT_FILE are required")
endif()

execute_process(
 COMMAND git -C "${SOURCE_DIR}" rev-parse --verify HEAD
 RESULT_VARIABLE revision_result
 OUTPUT_VARIABLE revision
 OUTPUT_STRIP_TRAILING_WHITESPACE
 ERROR_QUIET)
if(NOT revision_result EQUAL 0 OR revision STREQUAL "")
 set(revision "unknown")
endif()

execute_process(
 COMMAND git -C "${SOURCE_DIR}" status --porcelain --untracked-files=no
 RESULT_VARIABLE status_result
 OUTPUT_VARIABLE status
 OUTPUT_STRIP_TRAILING_WHITESPACE
 ERROR_QUIET)
if(status_result EQUAL 0 AND NOT status STREQUAL "")
 string(APPEND revision "+dirty")
endif()

get_filename_component(output_directory "${OUTPUT_FILE}" DIRECTORY)
file(MAKE_DIRECTORY "${output_directory}")
set(temporary "${OUTPUT_FILE}.tmp")
file(WRITE "${temporary}"
 "#pragma once\n\n#define LOST_SIGNAL_BUILD_REVISION \"${revision}\"\n")
execute_process(
 COMMAND "${CMAKE_COMMAND}" -E copy_if_different
 "${temporary}" "${OUTPUT_FILE}"
 COMMAND_ERROR_IS_FATAL ANY)
file(REMOVE "${temporary}")
