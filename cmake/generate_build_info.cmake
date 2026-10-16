set(revision "unknown")
set(dirty false)

find_package(Git QUIET)
if(Git_FOUND AND EXISTS "${SOURCE_DIR}/.git")
 execute_process(
 COMMAND "${GIT_EXECUTABLE}" -C "${SOURCE_DIR}" rev-parse HEAD
 OUTPUT_VARIABLE git_revision
 OUTPUT_STRIP_TRAILING_WHITESPACE
 RESULT_VARIABLE revision_result
 ERROR_QUIET)
 if(revision_result EQUAL 0 AND NOT git_revision STREQUAL "")
 set(revision "${git_revision}")
 endif()
 execute_process(
 COMMAND "${GIT_EXECUTABLE}" -C "${SOURCE_DIR}" status --porcelain --untracked-files=normal
 OUTPUT_VARIABLE git_status
 OUTPUT_STRIP_TRAILING_WHITESPACE
 RESULT_VARIABLE status_result
 ERROR_QUIET)
 if(status_result EQUAL 0 AND NOT git_status STREQUAL "")
 set(dirty true)
 endif()
endif()

foreach(value revision PROJECT_VERSION_STRING COMPILER_ID COMPILER_VERSION SYSTEM_NAME SYSTEM_PROCESSOR BUILD_TYPE)
 string(REPLACE "\\" "\\\\" ${value} "${${value}}")
 string(REPLACE "\"" "\\\"" ${value} "${${value}}")
endforeach()

set(contents "#pragma once\n\nnamespace build_info {\n")
string(APPEND contents "inline constexpr const char* project_version = \"${PROJECT_VERSION_STRING}\";\n")
string(APPEND contents "inline constexpr const char* source_revision = \"${revision}\";\n")
string(APPEND contents "inline constexpr bool source_dirty = ${dirty};\n")
string(APPEND contents "inline constexpr const char* compiler_id = \"${COMPILER_ID}\";\n")
string(APPEND contents "inline constexpr const char* compiler_version = \"${COMPILER_VERSION}\";\n")
string(APPEND contents "inline constexpr const char* system_name = \"${SYSTEM_NAME}\";\n")
string(APPEND contents "inline constexpr const char* system_processor = \"${SYSTEM_PROCESSOR}\";\n")
string(APPEND contents "inline constexpr const char* build_type = \"${BUILD_TYPE}\";\n")
string(APPEND contents "} // namespace build_info\n")

get_filename_component(output_directory "${OUTPUT_FILE}" DIRECTORY)
file(MAKE_DIRECTORY "${output_directory}")
set(temporary "${OUTPUT_FILE}.tmp")
file(WRITE "${temporary}" "${contents}")
execute_process(COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${temporary}" "${OUTPUT_FILE}")
file(REMOVE "${temporary}")
