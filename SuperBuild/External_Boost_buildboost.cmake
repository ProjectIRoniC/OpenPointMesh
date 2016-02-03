#
# Follow the boost suggestions
#

if(WIN32)
set(XXX "
# Needed for PCL
		--with-atomic	
        --with-chrono 
        --with-context 
        --with-date_time 
        --with-exception 
        --with-filesystem
		--with-format
        --with-graph 
        --with-graph_parallel 
        --with-iostreams 
		--with-locale
        --with-log 
        --with-math 
        --with-mpi 
		--with-program_options
        --with-python 
        --with-random 
        --with-regex 
        --with-serialization 
        --with-signals
        --with-system
        --with-thread
		--with-timer 
        --with-wave
# Not Needed
        --without-test 
"
)
  execute_process(COMMAND ./b2 install --prefix=${BOOST_INSTALL_DIR}
    WORKING_DIRECTORY ${BUILD_DIR} RESULT_VARIABLE build_result)

else(WIN32)

  execute_process(COMMAND ./b2 install
    WORKING_DIRECTORY ${BUILD_DIR} RESULT_VARIABLE build_result)

endif(WIN32)

return(${build_result})
