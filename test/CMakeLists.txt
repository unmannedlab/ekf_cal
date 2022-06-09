
# @todo(hartzerj): Add for loop here and more tests
ament_add_gtest(test_MathHelper test_MathHelper.cpp)
if(TARGET test_MathHelper)
    target_link_libraries(test_MathHelper ${${PROJECT_NAME}_lib})
endif()
