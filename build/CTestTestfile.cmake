# CMake generated Testfile for 
# Source directory: /Users/simoneborchetta/Documents/Uni/progetto-fisica-senza-refusi
# Build directory: /Users/simoneborchetta/Documents/Uni/progetto-fisica-senza-refusi/build
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
if(CTEST_CONFIGURATION_TYPE MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
  add_test([=[progetto.t]=] "/Users/simoneborchetta/Documents/Uni/progetto-fisica-senza-refusi/build/Debug/progetto.t")
  set_tests_properties([=[progetto.t]=] PROPERTIES  _BACKTRACE_TRIPLES "/Users/simoneborchetta/Documents/Uni/progetto-fisica-senza-refusi/CMakeLists.txt;68;add_test;/Users/simoneborchetta/Documents/Uni/progetto-fisica-senza-refusi/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
  add_test([=[progetto.t]=] "/Users/simoneborchetta/Documents/Uni/progetto-fisica-senza-refusi/build/Release/progetto.t")
  set_tests_properties([=[progetto.t]=] PROPERTIES  _BACKTRACE_TRIPLES "/Users/simoneborchetta/Documents/Uni/progetto-fisica-senza-refusi/CMakeLists.txt;68;add_test;/Users/simoneborchetta/Documents/Uni/progetto-fisica-senza-refusi/CMakeLists.txt;0;")
elseif(CTEST_CONFIGURATION_TYPE MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
  add_test([=[progetto.t]=] "/Users/simoneborchetta/Documents/Uni/progetto-fisica-senza-refusi/build/RelWithDebInfo/progetto.t")
  set_tests_properties([=[progetto.t]=] PROPERTIES  _BACKTRACE_TRIPLES "/Users/simoneborchetta/Documents/Uni/progetto-fisica-senza-refusi/CMakeLists.txt;68;add_test;/Users/simoneborchetta/Documents/Uni/progetto-fisica-senza-refusi/CMakeLists.txt;0;")
else()
  add_test([=[progetto.t]=] NOT_AVAILABLE)
endif()
