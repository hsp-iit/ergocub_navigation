####################################################################################################
#                                                                                                  #
#                    Generate YARP Thrift interface for bimanual controller                        #
#                                                                                                  #
####################################################################################################

find_package(YARP REQUIRED)                                                                         # Won't work otherwise!
 
set(generated_libs_dir "${PROJECT_SOURCE_DIR}")

############################ Invoke YARP IDL macro to generate files ###############################

yarp_idl_to_dir(INPUT_FILES ControlInterface.thrift 
                OUTPUT_DIR ${generated_libs_dir}
                SOURCES_VAR src                                                                     # Location of generated source files
                HEADERS_VAR include                                                                 # Location of generated header files
                INCLUDE_DIRS_VAR include_dirs)                                                      # Paths to include listed in ${include_dirs}
