add_test([=[HelloTest.PrintPositiveTest]=]  K:/Study_Material_AC/Control_Projects/Control_System_Projects/P1_Cruise_Control/build/hello_test.exe [==[--gtest_filter=HelloTest.PrintPositiveTest]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[HelloTest.PrintPositiveTest]=]
  PROPERTIES
    
    _CMAKE_TEST_BUILD_DEPENDS [==[K:/Study_Material_AC/Control_Projects/Control_System_Projects/P1_Cruise_Control/build/hello_test.exe]==]
    DEF_SOURCE_LINE [==[K:\Study_Material_AC\Control_Projects\Control_System_Projects\P1_Cruise_Control\tests\hello_test.cpp:6]==]
    WORKING_DIRECTORY [==[K:/Study_Material_AC/Control_Projects/Control_System_Projects/P1_Cruise_Control/build]==]
    SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==]
    
)
add_test([=[HelloTest.PrintNegativeTest]=]  K:/Study_Material_AC/Control_Projects/Control_System_Projects/P1_Cruise_Control/build/hello_test.exe [==[--gtest_filter=HelloTest.PrintNegativeTest]==] --gtest_also_run_disabled_tests)
set_tests_properties([=[HelloTest.PrintNegativeTest]=]
  PROPERTIES
    
    _CMAKE_TEST_BUILD_DEPENDS [==[K:/Study_Material_AC/Control_Projects/Control_System_Projects/P1_Cruise_Control/build/hello_test.exe]==]
    DEF_SOURCE_LINE [==[K:\Study_Material_AC\Control_Projects\Control_System_Projects\P1_Cruise_Control\tests\hello_test.cpp:12]==]
    WORKING_DIRECTORY [==[K:/Study_Material_AC/Control_Projects/Control_System_Projects/P1_Cruise_Control/build]==]
    SKIP_REGULAR_EXPRESSION [==[\[  SKIPPED \]]==]
    
)
set(hello_test_TESTS [==[HelloTest.PrintPositiveTest]==] [==[HelloTest.PrintNegativeTest]==])
