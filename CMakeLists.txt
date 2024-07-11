cmake_minimum_required(VERSION 3.5)

project(MyPluginPassivityHumanoid LANGUAGES CXX VERSION 1.0.0)

include(CTest)
enable_testing()

# Find mc_rtc if we are not building inside mc_rtc itself
if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()

# Configure and install the plugin configuration
configure_file(etc/MyPluginPassivityHumanoid.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/MyPluginPassivityHumanoid.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/MyPluginPassivityHumanoid.yaml" DESTINATION "${MC_PLUGINS_RUNTIME_INSTALL_PREFIX}/etc")

set(plugin_SRC
  src/MyPluginPassivityHumanoid.cpp
  )

set(plugin_HDR
  src/MyPluginPassivityHumanoid.h
)


# The add_plugin macro is provided by mc_rtc
add_plugin(MyPluginPassivityHumanoid "${plugin_SRC}" "${plugin_HDR}")

find_package(jrl-qp REQUIRED)
target_link_libraries(MyPluginPassivityHumanoid PUBLIC jrl-qp::jrl-qp)


if(BUILD_TESTING)
  add_subdirectory(tests)
endif()
